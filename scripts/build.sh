#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
preset=Debug
configure_only=false
for arg in "$@"; do
    case "$arg" in
        Debug|Release) preset=$arg ;;
        --configure-only) configure_only=true ;;
        -h|--help)
            echo "Usage: $0 [Debug|Release] [--configure-only]"
            echo "Optional: ARM_TOOLCHAIN_BIN=/path/to/bin JOBS=8"
            exit 0 ;;
        *) echo "Unknown argument: $arg" >&2; exit 2 ;;
    esac
done
if ! command -v ninja >/dev/null 2>&1 && [[ -x "$root/.tools/arm/usr/bin/ninja" ]]; then
    export PATH="$root/.tools/arm/usr/bin:$PATH"
fi
for tool in cmake ninja; do
    command -v "$tool" >/dev/null 2>&1 || { echo "Missing tool: $tool" >&2; exit 1; }
done
cd "$root/RL_leg"
cmake --preset "$preset" "-DCMAKE_MAKE_PROGRAM=$(command -v ninja)"
# clangd always uses Debug, independent of which firmware was built last.
if [[ "$preset" == Debug ]]; then
    ln -sfn build/Debug/compile_commands.json compile_commands.json
fi
if ! "$configure_only"; then
    cmake --build --preset "$preset" --parallel "${JOBS:-$(nproc)}"
    echo "Firmware: $root/RL_leg/build/$preset/RL_leg.{elf,hex,bin}"
fi
