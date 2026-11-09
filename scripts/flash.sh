#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
preset=Debug
build=true
dry_run=false
for arg in "$@"; do
    case "$arg" in
        Debug|Release) preset=$arg ;;
        --no-build) build=false ;;
        --dry-run) dry_run=true ;;
        -h|--help)
            echo "Usage: $0 [Debug|Release] [--no-build] [--dry-run]"
            echo "Optional: JLINK_EXE, JLINK_DEVICE (STM32F427II), JLINK_SPEED (4000 kHz), JLINK_SERIAL"
            exit 0 ;;
        *) echo "Unknown argument: $arg" >&2; exit 2 ;;
    esac
done
elf="$root/RL_leg/build/$preset/RL_leg.elf"
jlink=${JLINK_EXE:-JLinkExe}
if [[ -z ${JLINK_EXE:-} ]] && ! command -v "$jlink" >/dev/null 2>&1; then
    jlink=/opt/SEGGER/JLink/JLinkExe
fi
args=(-device "${JLINK_DEVICE:-STM32F427II}" -if SWD -speed "${JLINK_SPEED:-4000}"
      -autoconnect 1 -ExitOnError 1 -NoGui 1)
if [[ -n ${JLINK_SERIAL:-} ]]; then
    args+=(-SelectEmuBySN "$JLINK_SERIAL")
fi
if "$dry_run"; then
    echo "Firmware: $elf"
    printf 'Command: '
    printf '%q ' "$jlink" "${args[@]}"
    printf '\nJ-Link commands:\nr\nh\nloadfile "%s"\nr\ng\nq\n' "$elf"
    exit 0
fi
if "$build"; then
    "$root/scripts/build.sh" "$preset"
fi
[[ -f "$elf" ]] || { echo "Firmware missing: $elf" >&2; exit 1; }
command -v "$jlink" >/dev/null 2>&1 || { echo "JLinkExe not found; set JLINK_EXE." >&2; exit 1; }
commands=$(mktemp /tmp/rl-leg-jlink.XXXXXX)
trap 'rm -f -- "$commands"' EXIT
printf 'r\nh\nloadfile "%s"\nr\ng\nq\n' "$elf" > "$commands"
"$jlink" "${args[@]}" -CommanderScript "$commands"
