# 摩擦补偿机械速度方案：离线实现检查点

## 目的与状态

用户授权改进摩擦补偿，选择“运动时按速度、静止时按角度误差”的方案，后续要求继续修改。目标是减少补偿对速度环制动/I/D力矩过零的敏感性，同时保留静止起动力矩。振荡的唯一根因尚未由数据证明；不能由闭环目标→航向的四频幅相直接推出开环相位裕度或断言临界稳定。现有250Hz记录也不能排除1kHz瞬时保护，早期中止的准确Stop原因未上报。

独立工作树：`/home/kekeyuyu/projects/步兵/infantry/.claude/worktrees/friction-comp-rootcause`，分支`worktree-friction-comp-rootcause`。创建时基于旧main `0d0597e`，先前会话产生WIP提交`fd23df2`，随后以合并提交`f6ca0fb`纳入已烧录扫频源码`2b7d8e8`。最终实现、接口说明和回归已按用户授权保存为本地提交`23b2684`；本记录与实测证据另作本地归档，未推送。不要仅取早期WIP作为最终实现，应沿完整特性分支恢复。实现与离线验证阶段没有硬件操作；后续另获授权完成Flash恢复及PID实测，具体结果与限制见下文和后续交接。

## 实现

- `infantry_execution.cpp`：取DM原生输出轴反馈速度，表示相对机构的机械运动；不使用世界航向速率。世界稳向时底盘旋转、轴在转但世界航向不变，仍需补偿。
- `q = 2*VMAX/4095`为12bit速度量化步长，抑制零点半LSB偏移；用`sign(v_motor)*clamp((abs(v_motor)-q)/V,0,1)`得到机械速度项。
- 用`torque_direction`把机械项映射到世界力矩符号；静止起动项是`clamp(angle_kp_live*shortest_arc_error/V,-1,1)`，不含角度I/D或速度PID输出。
- 两项按`m + (1-abs(m))*start`连续交接，乘摩擦幅值F后与原PID力矩相加，仍按原速度环输出限幅，再映射到MIT电机力矩。力矩Sweep仍在原电机侧位置注入，角度Sweep excitation Nm字段不变。
- `F=0`默认关闭，原PID/标定/地址/保护/使能失能策略不变。新模型仍可能因摩擦幅值不准或其他环路问题振荡；尚未实机验证改善。
- 参数`0x0A13`仍为Nm幅值；按用户选择，`0x0A14`改名`gimbal_friction_speed_rad_s`，单位改为rad/s，默认0.05，范围0.001..DM VMAX。旧`friction_band_nm`完全删除，无隐藏起动力矩band。旧Nm配置值不能原样回填；通用主机客户端没有自动单位迁移。

## 验证

本轮先添加真实Cycle/App/Executor/软件CAN回归，在当前未完成实现上执行方向契约，明确失败于`actual.torque_command_nm == expected`，日志`.tmp/friction-red.log`。未重新执行用户拒绝的临时替换/恢复源码实验。

新增/更新覆盖：
- 独立注入正/负/零DM机械速度与世界角速度，覆盖小陀螺稳向、两种电机方向映射。
- 零速量化偏移不产生虚假助力；速度带内部分补偿、带外满幅。
- 速度PID需求为零时，静止能按角度误差正反起动；到达目标补偿为零；在线角度Kp更新影响实际起动项。
- 运动中PID制动时补偿分量维持机械方向。
- 原不对称输出限幅、MIT Kp/Kd为零、关补偿等价、停止/故障回归保留。
- 参数新名称/单位、速度量程而非Nm量程、非法值拒绝和写后回读。

执行：
```sh
cmake --build build/host -j4
ctest --test-dir build/host --output-on-failure -j4
cmake --build build/infantry-app-arm-Release --target infantry_app -j4
```

最终结果：**46/46 CTest通过**；正式ARM APP构建及`check_infantry_app.py`镜像审计pass。Python测试使用原项目`.venv/bin/python`，未安装依赖。ARM SDK复用原项目`build/deps/STM32CubeF4-v1.28.3`，构建输出均在本worktree。链接有newlib nosys I/O未实现警告，既有审计仍通过。最终差异`git -c core.whitespace=cr-at-eol diff --check 2b7d8e8`通过；执行器已恢复原CRLF，避免整文件噪声。

日志：`.tmp/friction-build.log`、`.tmp/friction-full-ctest.log`、`.tmp/friction-arm-configure.log`、`.tmp/friction-arm-build.log`。

## 新镜像

相对worktree：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.{elf,bin,hex}`。

- 固件标识：`app-infantry-friction-v2`（通过ARM CMake EC_FIRMWARE_ID设置）。
- Flash 230856 B，RAM118208/131072 B；CCM未用，`.data`42528 B。
- ELF SHA256：`d57dcb07da6d7d706a1f700ba438c86156a57cdaf97b4046ea1c16e1db132863`。
- BIN SHA256：`19c3b641263a53b3d4107dbd4cc117079e1dd2301d7e0a7415c2f474accd294c`。
- 该镜像现已完成烧录和完整Flash校验（见下文最终恢复记录）；尚未进行新模型动力验证，不能声称车上振荡已修复。

## 烧录过程：先失败，后完整恢复成功

用户随后授权“烧录使用”。在新鲜STOP、无持权/执行、DM Disabled且无故障的预检下，使用固定DAP332107148313、1MHz开始烧录。擦写前成功保存原Flash 1048576字节，SHA256 `bcd87c30866f31ca80f972c130a17dda10ec022b7ea906a9e1b11ce536b9efed`，可作回退映像。随后GDB load期间报`cannot read IDR`、`error writing to flash at address 0x08000000`，退出1；未执行完整verify或reset run，不能认定新镜像已正确写入。

用户确认重插与检查后，100kHz诊断曾成功确认reset-halt和CPUID 0x410fc241。但随后实际恢复流程在OpenOCD初始化阶段再次`cannot read IDR`，GDB尚未启动、这次未进行新写入。连接仍不稳定，停止自动重试，目标/Flash当前状态未知，暂不启动电机或切运行挡。

证据均在本worktree：`.tmp/friction-v2-flash-20261114-055928/`含原1MiB映像、expected-flash.bin、GDB/OpenOCD日志及failure.json；`.tmp/friction-v2-recovery-20261114-060526/`含低速恢复连接失败记录。先修复SWD线缆/共地/供电/探针连接，再获得稳定控制并恢复写入、完整校验。没有通过校验前不能把新固件标为可用；没有对新版本做动力验证。

用户再次明确要求烧录后，执行100kHz恢复流程成功：同一会话中确认`TARGET_STATE:halted`，写入新ELF，完整230856字节Flash映像校验通过（含`.data` LMA `0x0802dfa8`的42528字节），`.text matched`，GDB退出0并reset run。证据：本worktree `.tmp/friction-v2-recovery-20261114-061928/`中的`result.json`、`gdb.log`、`openocd.log`及`postflash.json`。

烧录后921600串口读回：STOP=True、source=none、authority=False、执行域为空；DM online=True、enabled=False、faulted=False、反馈年龄15ms。参数恢复默认：角度Kp2/Kd0，速度Kp0.35，F=0，`0x0A14`值0.05且上限30（新rad/s语义）。没有在线写入旧值，没有发运动目标或START。SWD/GDB/串口均已关闭；上述状态是当次新鲜读回，不代替后续实时核验。

## 后续PID实测入口

用户另行授权后完成多轮单频调试，记录见`2026-11-14_friction-v2-pid-trials.md`。带F0.25Nm的Kp3候选随后被报告遥控回中保持抖动，说明Sweep结束失能不等于持续保持验收。主控复位后在STOP恢复Kp3/速度Kp0.6、保持F0，用户反馈这一配置可以；处理及量化缺口见`2026-11-14_rc-hold-oscillation.md`。源码默认2/0.35/F0不变，用户手感接受不等于带补偿模型已稳定或完成整车验收。

## 下一步

1. 新镜像已经完整恢复并校验，后续动力调整以最新PID实测记录为准，不重复执行失败恢复。本地提交已获授权并执行，远端推送与后续合并不在本次授权中；不把WIP和合并历史误当最终交付。
2. 烧录与动力对照分别取得授权，先新鲜右中STOP、DM Disabled；Flash完整校验包含`.data`的LMA映像。
3. 新版本默认F=0；配置时明确`0x0A14`新单位rad/s，先读当前生效PID再做单变量对照，不自动沿用旧RAM数值。
4. `2026-11-14_angle-sweep-next.md`登记的5项报告/计划缺陷仍不在本次修复范围；尤其缺少Sweep Stop上报，不能声称已知道早期中止的唯一原因。
