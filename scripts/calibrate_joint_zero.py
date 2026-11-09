#!/usr/bin/env python3
"""Sample joint positions at the URDF zero pose and generate firmware offsets."""

import argparse
import math
import os
import re
import statistics
import struct
import sys
import time
from pathlib import Path
from typing import Optional


FRAME_SIZE = 94
HEADER = b"\xA8\xE6"
TAIL = b"\xC3\xF7"
JOINT_NAMES = ("LEFT_FRONT", "RIGHT_FRONT", "LEFT_REAR", "RIGHT_REAR")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xFFFF


class FeedbackParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []

        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                keep = 1 if self.buffer.endswith(HEADER[:1]) else 0
                if keep:
                    del self.buffer[:-1]
                else:
                    self.buffer.clear()
                break

            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break

            candidate = bytes(self.buffer[:FRAME_SIZE])
            received_crc = candidate[90] | (candidate[91] << 8)
            if candidate[92:94] != TAIL or crc16(candidate[:90]) != received_crc:
                del self.buffer[0]
                continue

            frames.append(candidate)
            del self.buffer[:FRAME_SIZE]

        return frames


class VofaJustFloatParser:
    """Parse this firmware's 9-float VOFA JustFloat frames (36 data + 4 tail bytes)."""

    FRAME_SIZE = 40

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        tail_offset = self.FRAME_SIZE - 4
        tail = b"\x00\x00\x80\x7f"

        while len(self.buffer) >= self.FRAME_SIZE:
            if self.buffer[tail_offset:self.FRAME_SIZE] == tail:
                candidate = bytes(self.buffer[:self.FRAME_SIZE])
                values = struct.unpack_from("<9f", candidate)
                if all(math.isfinite(value) for value in values):
                    frames.append(candidate)
                del self.buffer[:self.FRAME_SIZE]
            else:
                # Re-align after startup noise or a dropped byte.
                del self.buffer[0]

        return frames


class PosixSerial:
    """Read serial data on POSIX directly, or on Windows through pyserial."""

    def __init__(self, device: str, baud: int, timeout: float = 0.2):
        self.device = device
        self.baud = baud
        self.timeout = timeout
        self.fd: Optional[int] = None
        self.serial_port = None

    def __enter__(self):
        if os.name == "nt":
            try:
                import serial
            except ImportError as error:
                raise RuntimeError("Windows serial support requires pyserial: py -m pip install pyserial") from error
            self.serial_port = serial.Serial(self.device, self.baud, timeout=self.timeout)
            return self

        import select
        import termios
        import tty

        speed = getattr(termios, f"B{self.baud}", None)
        if speed is None:
            raise ValueError(f"Unsupported baud rate: {self.baud}")

        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        tty.setraw(self.fd, termios.TCSANOW)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def read(self, size: int) -> bytes:
        if self.serial_port is not None:
            return self.serial_port.read(size)
        if self.fd is None:
            return b""
        import select
        readable, _, _ = select.select([self.fd], [], [], self.timeout)
        if not readable:
            return b""
        return os.read(self.fd, size)

    def __exit__(self, exc_type, exc_value, traceback):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def load_existing_offsets(header_path: Path):
    content = header_path.read_text(encoding="utf-8")
    offsets = {}
    for name in JOINT_NAMES:
        pattern = rf"{name}_OFFSET_RAD\s*=\s*([-+0-9.eE]+)f"
        match = re.search(pattern, content)
        if not match:
            raise ValueError(f"Cannot find {name}_OFFSET_RAD in {header_path}")
        offsets[name] = float(match.group(1))
    return offsets


def generate_header(header_path: Path, offsets):
    lines = [
        "#pragma once",
        "",
        "/* Generated by scripts/calibrate_joint_zero.py, units: rad. */",
        "namespace JointCalibration",
        "{",
    ]
    for name in JOINT_NAMES:
        lines.append(f"inline constexpr float {name}_OFFSET_RAD = {offsets[name]:.9f}f;")
    lines.extend(("} // namespace JointCalibration", ""))

    header_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = header_path.with_suffix(header_path.suffix + ".tmp")
    temp_path.write_text("\n".join(lines), encoding="utf-8")
    temp_path.replace(header_path)


def main():
    repo_root = Path(__file__).resolve().parents[1]
    default_header = repo_root / "RL_leg/User/Module/Chassis/JointCalibration.hpp"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device, e.g. COM3 on Windows or /dev/ttyUSB0 on Linux")
    parser.add_argument("--protocol", choices=("vofa", "board"), default="board",
                        help="board reads UART7 BoardTX (current firmware default); vofa reads USART3 JustFloat")
    parser.add_argument("--baud", type=int, default=115200,
                        help="serial baud rate (current firmware default: 115200)")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="maximum seconds to wait for all samples (default: 30)")
    parser.add_argument("--max-std", type=float, default=0.02,
                        help="maximum standard deviation in rad (default: 0.02)")
    parser.add_argument("--header", type=Path, default=default_header)
    args = parser.parse_args()

    if args.samples < 2 or args.max_std <= 0 or args.timeout <= 0:
        parser.error("--samples must be >= 2; --max-std and --timeout must be positive")

    try:
        existing_offsets = load_existing_offsets(args.header)
    except (OSError, ValueError) as error:
        print(f"Cannot read current offset configuration: {error}", file=sys.stderr)
        return 2

    print("请先将四个关节摆到 URDF 的 0 点姿态，并保持机器人稳定。")
    print("运行中的固件应使用当前 JointCalibration.hpp 中的偏置。")
    input("准备好了按回车读取")

    parser_state = VofaJustFloatParser() if args.protocol == "vofa" else FeedbackParser()
    samples = []
    started = time.monotonic()

    try:
        with PosixSerial(args.port, args.baud, timeout=0.2) as port:
            while len(samples) < args.samples and time.monotonic() - started < args.timeout:
                frames = parser_state.feed(port.read(256))
                for frame in frames:
                    # VOFA starts at channel 0; BoardTX leg_position begins at byte 4.
                    position_offset = 0 if args.protocol == "vofa" else 4
                    corrected = struct.unpack_from("<4f", frame, position_offset)
                    raw_positions = tuple(
                        corrected[i] + existing_offsets[name]
                        for i, name in enumerate(JOINT_NAMES)
                    )
                    if all(math.isfinite(value) for value in raw_positions):
                        samples.append(raw_positions)
                        print(f"\rValid frames: {len(samples)}/{args.samples}", end="", flush=True)
                        if len(samples) >= args.samples:
                            break
    except (OSError, RuntimeError, ValueError) as error:
        print(f"\nSerial error: {error}", file=sys.stderr)
        return 1

    elapsed = time.monotonic() - started
    print(f"\nCollected {len(samples)} frames in {elapsed:.1f}s.")
    if len(samples) < args.samples:
        print(f"Timed out before collecting all {args.samples} frames; calibration header was not changed.",
              file=sys.stderr)
        return 1

    offsets = {}
    for index, name in enumerate(JOINT_NAMES):
        values = [sample[index] for sample in samples]
        mean = statistics.fmean(values)
        deviation = statistics.pstdev(values)
        print(f"{name:>11}: offset={mean:+.6f} rad, std={deviation:.6f} rad")
        if deviation > args.max_std:
            print(f"Unstable {name} measurement; max std is {args.max_std:.6f} rad.", file=sys.stderr)
            return 1
        offsets[name] = mean

    generate_header(args.header, offsets)
    print(f"零点偏置已写入：{args.header}")
    print("请重新编译并下载固件以应用偏置。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
