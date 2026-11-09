# RL_leg 构建与下载

目标芯片：STM32F427IIH6，Cortex-M4F；下载器：J-Link，接口：SWD。

在本目录执行（脚本也可以从其他工作目录调用）：

```bash
./scripts/build.sh                     # Debug 编译
./scripts/build.sh Release             # Release 编译
./scripts/build.sh --configure-only    # 仅配置并生成 clangd 编译数据库
./scripts/flash.sh                     # 先编译 Debug，成功后下载并复位运行
./scripts/flash.sh Release             # 编译并下载 Release
./scripts/flash.sh --no-build           # 下载现有 Debug ELF
./scripts/flash.sh --dry-run            # 预览命令，不编译、不连接硬件
```

编译成功后，产物位于 `RL_leg/build/Debug/` 或 `RL_leg/build/Release/`：
`RL_leg.elf`、`RL_leg.hex`、`RL_leg.bin`、`RL_leg.map`。
编译失败时下载脚本立即退出，不会下载旧固件；`--no-build` 则明确使用已有固件。

VS Code 中 `Ctrl+Shift+B` 编译 Debug，运行任务 `Flash Debug (J-Link SWD)` 下载。
clangd 固定读取 Debug 编译数据库；首次使用先运行一次 Debug 配置，再执行
`clangd: Restart language server`。沿用本机用户设置中已安装的 clangd 22。

## 工具

需要 CMake >= 3.22、Ninja、ARM GNU GCC（含 newlib 和 C++ 库）、SEGGER J-Link。
当前工作区的 `.tools/arm/` 已放置本地工具链，脚本会自动使用；此目录不提交。
换机器后可以安装系统依赖：

```bash
sudo apt-get install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib clangd
```

也可以设置 `ARM_TOOLCHAIN_BIN=/path/to/arm-gnu-toolchain/bin` 后在新的构建目录配置，
或先移走旧的 `RL_leg/build` 缓存再配置。VS Code 的 clangd 路径应按新机器调整，建议使用较新版本。
J-Link 默认从 PATH 或 `/opt/SEGGER/JLink/JLinkExe` 查找。

```bash
JOBS=8 ./scripts/build.sh
JLINK_EXE=/path/to/JLinkExe JLINK_SPEED=1000 ./scripts/flash.sh
JLINK_SERIAL=123456789 ./scripts/flash.sh
```

`JLINK_DEVICE` 默认 `STM32F427II`；`JLINK_SPEED` 单位为 kHz。

## 当前源码阻塞

CMake 配置已通过，`Callback.cpp` 的 clangd 检查为 0 个错误。
完整编译仍需要恢复业务模块 `RobotContext.hpp`、`ChassisController.hpp` 及其实现。
它们被 Control/Motor/Serial/Communication 任务引用，但当前目录没有这些文件。
不要使用空实现替代它们生成固件。

此外，`Core/Src/freertos.c` 仍引用 `AlgTask`、`CanTask`、`UartTask`，
现有任务实现名为 `Control`、`Motor`、`Serival`、`Communication`、`UI`。
恢复缺失模块后还需要确认任务对应关系并完成 FreeRTOS 接入，然后才能验证最终链接和上板运行。
当前仅预览过下载命令，未实际烧录。
