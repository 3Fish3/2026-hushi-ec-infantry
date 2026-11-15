---
date: 2026-11-15
updated: 2026-11-15
tags: [robomaster, infantry, windows, vscode, build, flash]
---

# Windows 编译与烧录说明

日常改完代码，按 **Ctrl+Shift+B** 编译步兵程序。需要写入开发板时，选择 **终端 → 运行任务 → 步兵：编译并烧录到开发板**。后者会先编译，再通过 CMSIS-DAP/SWD 写入、校验并复位运行。

本文对应仓库中的[任务配置](../../../.vscode/tasks.json)、[编辑器设置](../../../.vscode/settings.json)与[本机预设示例](../../../CMakeUserPresets.example.json)。以下路径均相对于 `infantry` 仓库根目录。

## 1. 从哪里点

在 VS Code 顶部选择 **终端 → 运行任务**，输入“步兵”筛选。本项目提供四个入口：

| 任务名称 | 什么时候用 | 实际执行 | 是否连接开发板 |
|---|---|---|---|
| **步兵：准备编译（检查环境）** | 首次配置、修改工具链或 SDK 路径后排查环境 | 读取 `local-app`，检查构建所需环境并生成编译规则 | 否 |
| **步兵：编译程序** | 日常保存代码后；默认快捷键 **Ctrl+Shift+B** | 先准备编译，再编译并链接完整 APP | 否 |
| **步兵：重新编译（清理旧结果）** | 需要清理已有编译产物再完整编译 | 先准备编译，再执行 `--clean-first` | 否 |
| **步兵：编译并烧录到开发板** | 准备把当前程序写入开发板 | 依次准备、编译、调用 OpenOCD 写入与校验，随后复位运行 | **是** |

**烧录请到“运行任务”里找。** “终端 → 运行生成任务”主要用于构建任务，本项目只有“步兵：编译程序”被设置为默认构建入口；烧录任务不属于该组，所以不能只在“运行生成任务”的列表里找它。

当前四个任务都直接运行程序，并用依赖关系连接准备、编译、烧录步骤。配置中的 `process`、`dependsOn` 和任务分组含义可查 [VS Code 官方任务说明](https://code.visualstudio.com/docs/debugtest/tasks)；具体命令以本仓库的任务文件为准。

如果刚更新配置后列表还没有这四项，可按 **Ctrl+Shift+P → 开发人员：重新加载窗口** 刷新，再检查打开的目录及 `.vscode/tasks.json` 所在位置。不要改用 **C/C++：gcc.exe 生成活动文件**。

## 2. 打开 RM 还是 infantry

VS Code 的单文件夹工作区设置位于所打开文件夹根部的 `.vscode` 中。打开一个父目录，不会自动把每个子项目的 `.vscode` 当作独立工作区设置加载。参见 [VS Code 工作区设置说明](https://code.visualstudio.com/docs/configure/settings)。

本机目前支持两种打开方式：

| 打开的文件夹 | 生效的任务与设置 | CMake 源目录、任务工作目录 |
|---|---|---|
| `infantry` | `infantry/.vscode/` | 当前打开的 `infantry` |
| `RM` | `RM/.vscode/`，这是本机补充的便利入口 | 当前目录下的 `infantry` |

本次报错时打开的是 `RM`，当时这里没有 `.vscode`。已经存在的 `infantry/.vscode` 因此没有给这个窗口提供步兵构建入口，CMake Tools 也没有按子仓库的设置定位工程。现在父目录设置用 `${workspaceFolder}/infantry` 指向步兵，父目录任务也在这个目录执行；其他项目原有文件保持不变。

**`RM/.vscode` 位于 infantry 仓库之外，不随克隆分享。新电脑直接打开克隆后的 `infantry` 文件夹即可使用仓库内的入口。** 如果自行打开更高层父目录，需要在那一层另行配置项目位置，不能只复制任务后保留错误的工作目录。

## 3. 首次使用需要准备什么

电脑需要 VS Code 的 CMake Tools、C/C++ 扩展，以及 CMake、Ninja、ARM GNU 工具链、STM32CubeF4 SDK 和 Python。烧录另需 OpenOCD；`cmake` 和 `openocd` 应能在 VS Code 的终端中被找到。安装或修改系统路径后，已经打开的 VS Code 可能需要重新启动才能取得新环境。

先把仓库根目录的 [CMakeUserPresets.example.json](../../../CMakeUserPresets.example.json) 复制为 `CMakeUserPresets.json`；已有本机文件就保留，按需核对对应预设。修改下面四项为本机实际位置：

| 字段 | 要填写的内容 |
|---|---|
| `CMAKE_MAKE_PROGRAM` | `ninja.exe` 的路径 |
| `ARM_GNU_ROOT` | ARM GNU 工具链根目录，其 `bin` 下应有 `arm-none-eabi-gcc`、`arm-none-eabi-g++` 等程序 |
| `EC_STM32CUBE_F4_ROOT` | STM32CubeF4 固件包根目录，包含 `Drivers`、`Middlewares` 等目录 |
| `Python3_EXECUTABLE` | 本项目 Python 解释器；Windows 虚拟环境通常为 `.venv/Scripts/python.exe` |

示例中的安装路径只是填写示范，不代表电脑已经安装相应软件。项目 Python 依赖见 [requirements-dev.txt](../../../requirements-dev.txt)。在仓库根目录的 PowerShell 中，首次创建环境可执行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

已有项目虚拟环境时不必重新创建。`CMakeUserPresets.json` 存放各自的安装路径，已被仓库 `.gitignore` 排除；共享的是示例文件。CMake 官方也将项目共享预设与个人本机预设区分管理，详见 [CMake Presets 手册](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html)。

### local-app 为什么还保留英文

`local-app` 是供任务调用的固定标识，中文显示名称为 **步兵正式程序（编译与烧录用）**；相应构建预设显示为 **编译步兵正式程序**。中文名称方便选择，标识保持不变，避免改显示文字时把命令引用改断。

四个中文任务已经明确调用 `local-app`，运行任务时不需要先手工选择某个 kit。如果要使用 CMake Tools 自身的配置、代码提示入口，可在命令面板选择上述配置预设和构建预设。该预设使用[ARM 工具链文件](../../../cmake/arm-none-eabi.cmake)，目标是 `infantry_app`，输出目录是 `build/app-arm-Release`。

本机另有的 `local-host` 用于电脑端测试，`local-imu` 用于 IMU 静态标定；它们不是正式步兵 APP 的构建入口。新克隆的示例只包含 `local-app`。

## 4. 配置、编译、链接、烧录分别做什么

整个过程可以按下面的顺序理解：

```text
本机预设 + CMakeLists.txt
          ↓ 配置：确定工具链、SDK、源码、宏、头文件路径和编译规则
源码 .c / .cpp / .s
          ↓ 编译与汇编：生成 ARM 目标文件
目标文件 + 库 + 链接脚本
          ↓ 链接：组合程序并安排 Flash / RAM 地址
infantry_app.elf
          ↓ OpenOCD 经 CMSIS-DAP / SWD 写入、校验
开发板 Flash
          ↓ 复位运行
STM32 开始执行程序
```

**配置**由 `cmake --preset local-app` 完成。它会检查工具链等依赖，并在构建目录生成 Ninja 使用的规则。成功意味着构建环境和规则已经建立，尚不代表全部源码已经编译通过；这个步骤可能执行编译器探测并写入构建目录，但不连接开发板。

**编译**由 ARM 交叉编译器把每个源文件转换成目标文件。交叉编译是指“在 Windows 电脑上运行编译器，生成给 ARM 芯片执行的机器码”。项目不止一个 `main.c`，还包括板级初始化、HAL、RTOS、框架和业务代码。CMake 负责把各文件需要的头文件目录、宏定义和编译选项交给编译器。

**链接**把这些目标文件和库组合成完整程序，同时按链接脚本安排入口、代码、常量及数据的位置。链接成功后，正式 APP 的主要产物为：

```text
build/app-arm-Release/boards/infantry_f407/infantry_app.elf
```

ELF 既有程序内容，也有地址和符号信息。它是开发板固件，不能作为 Windows 的 `.exe` 双击运行。点击“编译程序”包含编译和链接；若源码没有变化，Ninja 可以复用已有结果，不会每次全部重编。

**重新编译**先清理构建系统记录的旧编译产物，再进行编译和链接。当前任务的 `--clean-first` 不删除源码，也不等于删除全部 CMake 配置缓存；更换工具链时仍需核对本机预设与缓存是否一致。

**烧录**由 OpenOCD 读取上述 ELF，控制调试器把程序写入芯片 Flash。任务显式带有 `verify reset exit`，因此写入后会校验，再复位运行并退出。普通编译、准备编译以及 CMake 的“安装”都不是烧录。

## 5. 怎样连接和判断烧录结果

先接好 CMSIS-DAP 与开发板的 SWD 接口，并确保开发板供电与接线正确。烧录后程序会立即复位运行，应保持电机处于停止状态，并保留现有物理急停。连接说明见[接口与接线注意](接口与接线注意.md)。

**SWD 与 USART 是两条不同通道：** 本任务通过 CMSIS-DAP 的 SWD 接口写 Flash；USART 串口用于运行后的通信、参数或采集。即使同一个 USB 调试器同时提供这两种功能，串口已连接也不代表 SWD 接线正确。当前任务不选择 COM 口，不通过串口发送运动命令。

操作顺序：

1. 保存源码。
2. 选择 **终端 → 运行任务 → 步兵：编译并烧录到开发板**。
3. 先观察准备与编译结果。当前任务将编译设为烧录前置步骤，编译失败应先处理错误。
4. 查看 OpenOCD 输出，确认写入和校验成功、没有后续错误，并确认任务正常退出。

`Verified OK` 或相应的校验成功信息，表示 OpenOCD 检查的固件写入内容与目标一致。它不等于传感器、轮向、PID 或整车行为已经验证；如果后续复位步骤报错，也不能只凭前面的校验成功就认为整项任务完成。

当前任务使用 `interface/cmsis-dap.cfg` 与 `target/stm32f4x.cfg`，通过 SWD 操作 STM32F4，适配器速度为 950 kHz。这是本项目任务的连接配置，不是任意接线长度或调试器都已验证的通用保证。任务没有指定探针序列号；同时连接多只 CMSIS-DAP 时，应先明确使用哪一只，避免目标选择不清。

## 6. 这次两个错误为什么发生

### CMake 任务提示终端启动失败、退出代码 −1

本次 CMake Tools 日志显示没有选中 kit，构建驱动没有正常建立；当时又打开了未配置的 `RM` 父目录。这解释了为什么菜单动作没有进入正确的步兵构建流程。**单独一个 −1 不能证明 PowerShell 或终端程序损坏，也不是所有同类报错的统一原因。**

现在的中文任务直接以 `process` 方式运行 `cmake --preset local-app` 和对应构建命令，并明确工作目录。其工具链选择来自预设，不再依赖之前那个未建立的 CMake Tools 任务驱动。CMake Tools 仍用于读取工程信息、代码提示和跳转。

### gcc 编译 main.c 时找不到 main.h

随后调用的 **C/C++：gcc.exe 生成活动文件** 使用了电脑端 GCC，只编译编辑器当前打开的 CubeMX `main.c`，目标还是 Windows 可执行文件。它没有采用本项目的 CMake 规则，因此没有取得 `main.h` 所在目录及完整工程依赖。

给该命令手工补一个 `-I` 只能越过眼前的头文件错误，仍然缺少正确的 ARM 目标、其他源码、启动文件、链接脚本及必要配置，不能由此生成本项目固件。应回到 **步兵：编译程序**。C/C++ 扩展在这里继续负责编辑和代码分析，不使用它的单文件构建入口代替整个工程。

## 7. 遇到失败，先看哪一步

| 看到的现象 | 优先判断 | 下一步 |
|---|---|---|
| 找不到中文任务 | 当前窗口没有读取对应层级的任务文件 | 核对第 2 节的打开目录 |
| 找不到 `local-app` 或预设文件 | 工作目录不在 `infantry`，或尚未建立本机预设 | 检查根目录 `CMakeUserPresets.json` 及预设名称 |
| `cmake` / `openocd` 无法启动 | 可执行程序未安装、路径未加入当前进程环境，或任务工作目录无效 | 在同一个 VS Code 终端检查命令位置，必要时重启 VS Code |
| 找不到 Ninja、ARM 编译器、SDK、Python | 本机预设中的路径不对应实际安装 | 修改第 3 节的对应字段，再运行“准备编译” |
| 编译器报某行代码错误 | 已经进入源码编译 | 找输出中最早的实际错误及文件行号，不只看末尾的 `build stopped` |
| 链接时报 `undefined reference`、内存区域溢出等 | 目标文件已生成，但完整程序无法链接 | 检查缺失实现、链接依赖或内存布局，烧录不能解决它 |
| OpenOCD 找不到探针或目标 | 进入了硬件连接阶段 | 检查供电、SWD 接线、USB 探针选择及占用情况 |
| 写入或校验失败 | 固件尚未被确认正确写入 | 保存完整 OpenOCD 输出，检查连接和目标状态，不把文件存在当作烧录成功 |

排查时保留终端中的实际命令和第一条错误。若只有界面提示而没有构建输出，可在 **查看 → 输出** 中选择 CMake / CMake Tools 相关频道，查看当次配置与驱动记录。

## 8. 不经过任务菜单的备用命令

下面命令在 PowerShell 中执行。先把当前位置切到 `infantry` 仓库根目录；如果终端当前在 `RM`，执行 `Set-Location .\infantry` 一次即可。不要在已经位于 `infantry` 时再进入一个不存在的同名子目录。

查看预设并编译：

```powershell
cmake --list-presets=all
cmake --preset local-app
if ($LASTEXITCODE -ne 0) { throw "准备编译失败，请先检查上面的错误。" }
cmake --build --preset local-app
if ($LASTEXITCODE -ne 0) { throw "编译失败，请先检查上面的错误。" }
```

需要清理后重编时，在准备编译成功后改用：

```powershell
cmake --build --preset local-app --clean-first
```

下面是任务中实际使用的烧录命令，**执行它会写入开发板并复位运行**。仅在本次编译成功、已确认目标与连接状态后执行：

```powershell
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -f target/stm32f4x.cfg -c "adapter speed 950" -c "program {build/app-arm-Release/boards/infantry_f407/infantry_app.elf} verify reset exit"
```

## 9. 验证范围与后续维护

本次处理进行了工作区与任务配置检查，并验证本机 ARM 编译链路；**没有执行烧录任务，没有连接电机开展运动验证**。任务中存在烧录命令不代表它已经执行，编译成功也不代表新固件已经在开发板运行。

后续修改任务名称、预设标识、输出目录或烧录文件时，应同步核对本说明和[任务配置](../../../.vscode/tasks.json)。尤其不要只修改 `binaryDir`，却让烧录任务继续指向旧目录的 ELF。其他构建方式与板级资源见[生成与构建](生成与构建.md)。
