# 维护模式有限时长角度正弦修复

## 检查点

- 仓库：`/home/kekeyuyu/projects/步兵/infantry`。
- 分支：`feat/m2-sweep-rc-return-20261113`，本轮实现基于 `5311302`，随本次源码提交归档。最新继续工作入口见 `docs/handoffs/2026-11-14_angle-sweep-next.md`。
- 本轮仅修改源码、主机工具、测试和说明；没有访问串口、烧录、使能电机或进行动力试验。
- 原问题是旧 `InfantryCycle::set_sweep()` 依赖参数和运行挡自动启动无限正弦，缺少计划结束和统一终态失能；维护模式还存在目标提交来源/空快照冲突。已将角度试验统一接入现有有限时长 `sweep::Engine`，不再扩展旧路径。

## 当前行为

- 所有试验均在左下＋右下、新鲜遥控和维护会话持续持权下显式 START。
- `0x49 op1`：电机侧力矩正弦，幅值单位 Nm，上限0.03；原有路径保留。
- `0x49 op2`：世界航向目标正弦，幅值单位 rad，上限0.2。捕获 START 时的当前世界航向，每控制周期经 APP `accept_targets(..., Source::maintenance)` 提交目标，再由原级联 PID / MIT 纯力矩路径执行。主机仅续维护心跳，不反复覆盖为基准目标。
- 两种模式共用一个引擎且互斥；1–6个0.2–5Hz严格递增频点，每频3–12周期，总时长不超过120s；首尾各一个周期线性包络。
- 角度模式的 requested/applied excitation Nm 恒为0；真实闭环输出仍在 torque_command_nm。
- 完成/取消/故障统一走原失能路径并锁存；显式新 START 可重新武装。普通遥控恢复仍须物理右中挡、新鲜Disabled确认，并重新捕获当前方向，不恢复旧目标。
- 旧 `set_sweep()` 和参数 `0x0A10/0x0A11` 已退役；旧ID不再枚举，写入返回 unknown_param。没有更改 PID、标定、CAN地址或其它设备配置。
- schema2、72字节记录和250Hz采样不变；新增 bit9 `sweep_angle`，终态保留，清锁存后清除。
- 主机新增 `--mode angle --amplitude-rad ...`，参数单位混用在开串口前拒绝；角度报告逐频剔除淡入/淡出周期并拟合幅值、增益、滞后角、残差。设计要求usable至少两个稳态周期，但复核发现当前实现误用了整段时长，尚待修正；其余报告缺陷见最新交接。

## 验证

先运行真实封帧→会话→Cycle→App→Executor→软件CAN集成测试，确认修复前在 op2 START 的 ACK 断言失败（当时 `tests/infantry_direction_tests.cpp:819`），日志 `.tmp/maintenance-angle-red-resume.log`。

修复后执行：

```sh
cmake --build build/host -j4
ctest --test-dir build/host --output-on-failure -j4
cmake --build build/infantry-app-arm-Release --target infantry_app -j4
```

- 全量 CTest：**46/46通过**，包含原生C++/Python采集帧互操作。
- 实际协议集成覆盖：op2接受、越界拒绝、重放ACK不重启、op1/op2互斥、真实采纳目标振荡、无底盘目标、角度力矩注入字段为0、自然结束，以及遥控断联、主机维护心跳超时、IMU缺测、DM缺测、物理STOP、参考代次变化、CAN拒发、协议CANCEL后的撤输出。
- 引擎测试覆盖多频包络精确求值、单位隔离、两模式故障保护、arming和微秒回绕；记录器测试核对bit9实际字节位置。
- 原无限正弦回归测试迁移到新维护试验入口；保留热写不打断、补偿输出符号/过零/限幅、失联不自启、参考失效撤输出及遥控恢复断言。原16/17参数的旧接受断言改为明确拒绝，以覆盖退役契约。
- 正式 ARM 编译/链接与 `check_infantry_app.py` 镜像审计：**pass**。
- 链接输出含 newlib nosys `_close/_lseek/_read/_write` 未实现警告；镜像审计仍通过，未留下未使用的I/O或heap stub。
- 改动遵循原文件CRLF；使用 `git -c core.whitespace=cr-at-eol diff --check` 核验无额外空白错误。

日志：`.tmp/maintenance-angle-build.log`、`.tmp/maintenance-angle-full-ctest.log`、`.tmp/maintenance-angle-arm-build.log`。

## 构建产物

路径：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.{elf,bin,hex}`。

- ELF SHA256：`16b501a5585976da3de776cd01ad63d08f889caecab5ca011533d22d0c57c88d`。
- BIN SHA256：`94cc008e49bba4726833cc2f62a636f28297461656c5846291c7a4ccc5a473a9`。
- Flash镜像：230416 B；RAM链接占用118208/131072 B（剩余12864 B）；CCMRAM未用。
- `.data`：42528 B，Flash LMA `0x0802ddf0`；`.bss`：71072 B；heap512 B、主栈4096 B预留在链接预算内。
- 尚未进行本镜像Flash读回或运行时栈峰值验证；板上仍不是本轮产物。后续若获授权烧录，完整Flash校验必须包含`.data`的LMA映像。

## 下一步

2026-11-14用户要求先推送代码并交接，暂停继续修复。独立复核已返回报告端误判等待修项；本次源码是可恢复的开发检查点，不是已完成实机验收的发布版本。下一步先按最新交接补失败回归并修正，之后再申请烧录与动力试验授权。最终通过实测增益/滞后/残差判断响应慢与振荡，再决定PID调整。本轮没有将离线通过当作实机响应改善。

使用与协议详见 `docs/boards/infantry_f407/M2多频力矩Sweep.md`。
