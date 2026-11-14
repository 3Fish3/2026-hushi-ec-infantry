# Pitch MIT 电机位置模式：最终软件检查点

- 日期：2026-11-14。
- 工作区：`/home/kekeyuyu/projects/步兵/infantry/.claude/worktrees/friction-comp-rootcause`。
- 分支：`worktree-friction-comp-rootcause`。本轮及前序改动未提交、未推送；Git写操作由用户保留。

## 用户决定与实现

- 用户选择“电机MIT位置闭环”，不是切换电调模式。Pitch为达妙J4310，CAN1命令 `0x001`／反馈 `0x011`，DM槽1；Yaw仍为DM槽0、CAN1 `0x005`，世界稳向不改。
- Pitch目标由当前位置接管，摇杆按电机轴角速度×dt积分出位置目标，回中保持；电调收到 `{pos=采用角+零偏, vel=0, kp=20, kd=0.5, tau=0}`。Kp20/Kd0.5是待调值，不是旧工程实测参数。没有向电调写零位。
- 删除Pitch纯速度及IMU保持外环的状态、配置与分段律；关节白名单只接受position。保留此前要求的反向操纵效果：Yaw sign=-1；Pitch新位置路径去掉旧速度律自带的负号后使用sign=+1，物理方向与旧 `(-raw)×(-1)` 一致。普通／小陀螺位置增量都为线性映射，上限0.806342rad/s（电机轴，非枪管俯仰角）。
- 用户后来要求实测端点各向外扩10%跨距，代码现为 `1.698291..2.982795 rad`，超出实测端点，外扩部分机械行程尚未验证。Pitch的IMU限位已移除；越界时有运行授权和有效目标才采用最近边界，电机故障、STOP或失联仍撤输出。外扩版已烧录，见下方记录。
- 新增DM1在线调参独立落槽验证，修正原先槽1参数写入槽0的错误。双DM设备页254B对应业务上限256B，UART发送缓冲按 `uart_wire_max` 530B承载最坏转义单帧。

## 验证

外扩10%版本：Host 46/46及ARM离线审计通过；2026-11-14经用户授权烧录并复位，完整Flash校验233320B通过，含 `.data` LMA `0x0802e948`、42528B。ELF SHA256 `d2600bfe3f807940124b0fdd2674a6343ac49e86d27487e8e12f20006005a910`；证据 `pitch-flash-20261114-154504/` 位于会话临时目录。写前写后均确认STOP、无持权/执行、两DM在线禁用无故障；参数回读Pitch Kp20/Kd0.5，Yaw Kp3/0.6，F0。未发运动指令，未做外扩行程实车验收。

内缩限位及越界返回版本：Host 46/46通过，ARM构建与离线镜像审计通过，用户授权后已烧录并复位运行；完整Flash校验233320B通过，含 `.data` LMA `0x0802e948`、42528B。ELF SHA256 `2817660c7d5a4c72917c2df0ff6cadeee1c19d5d069dcfecfc63ec96373bc20f`；证据位于会话临时目录 `pitch-flash-20261114-153813/`。烧录前后串口检查均为STOP、无持权/执行、两电机在线且未使能无故障；未发运动命令，尚未进行实车验证。以下原始验证与烧录记录属于之前保留IMU限位的版本。

- 新回归真实RED：位置模式缺少方向限位、反馈恢复沿用旧保持目标；修正后通过。保留并验证了原有Yaw/Sweep测试正文。
- **Host全量46/46通过**：位置捕获不跳零、推杆方向、回中保持、STOP与恢复、电机／遥控／IMU失联、IMU±15°限位与反向返回、DM1增益不改DM0、协议position接受／velocity拒绝。
- **ARM构建及正式镜像审计pass**。固件label：`app-infantry-pitch-mit-position`，Flash234464B。
- 产物：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.{elf,bin,hex}`。
- ELF SHA256：`da2a70d97f0b7b57111ba41b5330740330049937f6412dc860e4f0160ea2af04`。
- BIN SHA256：`aa06f20d922fcd236bb12f463459d5cc5366d07fca10e77f0f73417fd556aaa1`。
- 日志：本会话 `$CLAUDE_JOB_DIR/tmp/mit-position-{build,ctest,arm}.log`。

## 硬件状态与下一步

- **用户随后明确授权“烧录”；本MIT位置版已完成烧录、完整校验及复位运行。**写前和写后均通过实时串口检查：右中STOP、无持权、所有域不执行，Yaw及Pitch在线、Disabled、无故障；未发运动目标、ENABLE或START，未做实机位置保持验收。
- 使用DAP `332107148313`、SWD 100kHz；写前保存1MiB Flash备份。完整校验234464B通过，包括 `.data` Flash LMA `0x0802edc0`、42528B；`.text`比较matched，GDB退出0。
- 完整Flash校验映像SHA256：`50a641773d60e901f5253ff5b2fad0cbb5d6b839bf09a1573e18cdf2ff773538`。证据目录：`/home/kekeyuyu/.claude/jobs/13f1f33a/tmp/pitch-flash-20261114-144355/`，含 `preflight.json`、`result.json`、`postflash.json`、GDB/OpenOCD日志和 `previous-full-flash.bin`。该目录属本会话临时产物。
- 写后参数回读通过：Pitch Kp20/Kd0.5/TTL20ms；Yaw角度Kp3、速度Kp约0.6、两环Ki/Kd0，摩擦补偿0、平滑带约0.05。辅助脚本已修正为双DM禁用检查和当前参数ID。
- 下一步仅在取得对应动力测试授权后进行遥控位置保持与方向、机械行程验收；Kp20/Kd0.5仍为待调值。
- 新固件Pitch MIT参数为 `0x0A01/0x0A02`，整车Yaw参数在槽11（0x0B0A、0x0B0D、0x0B13、0x0B14），不得按旧槽10回填整车参数。
