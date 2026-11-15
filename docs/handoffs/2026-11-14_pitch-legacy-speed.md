# 旧 Pitch 速度参数迁移检查点

> 历史记录：本文记录了曾误认 Pitch 为 M3508 的排查过程及旧版本状态，不是当前部署配置。实际 Pitch 为 CAN1 达妙 J4310；当前控制、限位与烧录状态以 [MIT 位置模式检查点](2026-11-14_pitch-mit-position.md) 为准。下文对 CAN2/0x201 的 Pitch 身份判断后来已被用户纠正。

- **日期**：2026-11-14
- **任务**：将旧步兵 M3508 丝杆 Pitch 的有效速度控制配置适配到新框架并烧录；固化用户确认的 Yaw 默认参数。
- **工作区**：`/home/kekeyuyu/projects/步兵/infantry/.claude/worktrees/friction-comp-rootcause`
- **分支与状态**：`worktree-friction-comp-rootcause`，基于本地 `b0410bc`，本轮修改未提交、未推送。用户保留 Git 写操作权限；不能借烧录任务擅自提交。

## 一、 已确认事实与修改

- 旧有效配置来自 `gxnu_hushi_ec/app/gimbal/gimbal.hpp:136–152`（旧仓库实际绝对路径 `/home/kekeyuyu/projects/步兵/gxnu_hushi_ec`）：M3508/C620，CAN2 ID5，19:1，内部测速，仅速度环，Kp=4.5，Ki/Kd=0。`setRef(-cmd.pitch)`；无有效位置环。
- 新 Kp 为 `4.5 × 19 × 180/π / 16384 = 0.2989983611`，对应输出轴 rad/s 到满量程电流比例，±1 对应 ±20 A。旧误差死区 10 转子 deg/s 换成 0.0091859434 rad/s；反馈低通上一值权重 0.1，沿新框架控制周期滤波、首样本直接采用测量，不复制旧驱动每帧从零起滤波的实现。
- `infantry_config.hpp` 回填速度参数，Pitch 不要求位置 PID；Yaw 固化用户接受的角度 Kp3、速度 Kp0.6，两环 I/D0，F0，保留原 ±6 rad/s、±2 N·m 限幅。
- `infantry_types.hpp/config.cpp/app.cpp/protocol.cpp/execution.cpp/cycle.cpp` 区分步兵速度 Pitch 与其它位置关节。丝杆不恢复单圈关节参考、不发布电机圈数为俯仰角；Cycle 不再按位置角范围裁剪速度。
- IMU 限位沿用旧 ±15°，取新 `roll_pitch_yaw_rad[0]`（旧名称 Pitch 实为 X轴 roll），高限阻正速度、低限阻负速度、允许反向返回。IMU 姿态失效、过期或非有限时撤输出。`InertialFeedback::reference_valid` 由 `set_yaw_alignment()` 控制，仅表示 Yaw 航向对齐，不参与 X轴倾角判定。不得清空限位配置绕过该契约。
- `VelocityLoopConfig::error_deadband_rad_s` / `DjiAxisConfig::velocity_deadband_rad_s` 默认0保持旧通用行为，启用时带内含边界输出零，保留授权与滤波历史。
- 遥控与板级映射已完成：raw≤±50给零，普通 50..550 用 −raw×0.05 deg/s、超过550用 −raw×0.07，小陀螺用 −raw×0.05；普通满杆0.8063421144 rad/s，回中零速度，不创建位置保持。底盘/发射/键鼠零量程不顺带启用。

## 二、 验证状态与决策

- 最先写部署契约并实际得到 RED：Pitch 配置不完整。早期完整 Host 构建成功，45/46测试通过，仅板级旧断言仍要求 Pitch incomplete；这不是本轮最终验证结论。
- 已新增部署/App 的 IMU X轴方向限位与失效测试、速度环死区测试、UART协议模式测试及真实 DR16→Cycle→DjiAxis→CAN2 `0x1FF` 首槽测试。新链测试预期满杆原始电流约 ±3950 count；覆盖回中、多圈无需参考、CAN/IMU/遥控失联和物理右中 STOP。
- 中途测试编写的 API 参数顺序与 canonical STOP 用法已修正。App 输入是 Cycle 已规范化的挡位：canonical down=物理右中 STOP，canonical mid/mid=物理左下+右下维护。完整CAN链测试曾真实失败于 Pitch 不执行；源码核对发现 `reference_valid` 仅表示 Yaw 对齐，而非 X轴倾角有效性，已移除该误依赖。测试明确验证 Yaw 未对齐时有效倾角仍可用，同时保留姿态/样本无效、过期、非有限值与限位覆盖，不靠手工注入 Yaw 对齐掩盖启动问题。
- 遥控零量程在键鼠非零 rad/count 配置下也不得回退成位置意图，已补回归。
- **最终 Host 全量46/46通过**，包含新部署、遥控、协议、速度死区、CAN实际字节、IMU/遥控/CAN失联与STOP契约。日志 `pitch-final-host-build.log`、`pitch-final-ctest.log` 在本会话临时目录。
- **最终 ARM APP 编译与正式镜像审计 pass**，固件标识 `app-infantry-pitch-speed`。Flash 233568 B，RAM118664/131072 B（data42528+BSS76136），保留既有 newlib nosys I/O未实现链接警告。SDK 只读复用原仓库 `build/deps/STM32CubeF4-v1.28.3`；没有改依赖或系统环境。日志 `pitch-final-arm-build.log`。
- 最终产物 `build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.{elf,bin,hex}`。ELF SHA256：`ae2eb368c02cc9ef1aaaa0dfa69f77bede26eb27429fd65d285d37e30030722a`；BIN SHA256：`d18ddb2e24dd134603121b51c6065f5823a7fd909f44f5c625463c59cd6c131a`。根据 PT_LOAD 重建的完整Flash校验映像覆盖233568 B，含 `.data` 的 Flash LMA `0x0802ea40`、42528 B；校验映像 SHA256 `ca63745255ee1f63efe415621761c4af2e64628c61f916cf3f0766c6b27e3161`。本镜像随后已完成烧录与完整Flash校验，见下文。

## 三、 烧录结果与剩余风险

- 最初两次921600串口 HELLO 超时，均未进行SWD擦写。用户随后再次明确要求“烧录把”；本次串口恢复，新鲜预检确认 STOP=True、无持权/执行、DM在线/Disabled/无故障后才写入。
- **烧录成功**：2026-11-14，同一固定探针332107148313、100kHz，reset halt后备份原1MiB Flash，GDB加载上述ELF，完整233568字节Flash映像校验通过（含 `.data` LMA `0x0802ea40` 的42528字节），`.text matched`，GDB退出0，然后reset run。
- 复位后串口回读：STOP=True、无持权、无执行域；DM在线、Disabled、无故障，反馈年龄2ms。Pitch Kp=0.2989983559、Ki/Kd=0、最大速度0.8063421249rad/s；Yaw角度Kp3、速度Kp0.6000000238、两环I/D0、F0、速度带0.05rad/s，均在float32容差内与默认值一致。
- 证据目录：`/home/kekeyuyu/.claude/jobs/13f1f33a/tmp/pitch-flash-20261114-120537/`，含 `preflight.json`、`result.json`、`postflash.json`、`gdb.log`、`openocd.log`、完整校验映像和原1MiB备份。备份SHA256：`a44cb382202aa50d73c899092b7781f4cb8b80b6c01e442d3cef78d1dac00078`。这是会话临时目录，备份与详细日志未纳入Git。
- 未发送运动目标、enable或Sweep START，没有开展Pitch动力试验。未改探针固件、驱动或其它进程；源码及本记录仍未提交、未推送。

### 烧录后用户报告Pitch不动：只读排查

- 用户报告Pitch不能动后，运行本会话临时脚本 `pitch_diagnose.py`，连续6轮读取域状态、请求、设备反馈与统计，无参数写入或运动命令。证据：`/home/kekeyuyu/.claude/jobs/13f1f33a/tmp/pitch-diagnose-20261114-121002.json`。
- 6轮均显示 remote持权、STOP=False，但Pitch `online=False`、反馈年龄不可用、速度/电流/温度均无有效测量，`execute=False`、reason=`fault_dependency`，当前没有Pitch请求。配置确为CAN2 ID5、反馈ID `0x205`，设备assembled=True；四轮与Yaw在线。
- 回查本次烧录preflight与postflash原始证据，Pitch两次也均离线，说明该反馈缺失在本次写入前已存在。先前烧录简报只指出DM在线/Disabled，未单独报告Pitch离线，是交付漏项。
- 源码确认CAN2有初始化、正常模式、bank14全通过滤、RX0中断和executor路由。尚未通过原始CAN逐帧观测确认CAN2是否有其它ID，也未确认实际电调供电/线缆/ID，因此不把离线唯一归因为接线或电调故障。下一步先在STOP核对电调供电、实际总线/ID与CAN链路，不加大PID、不绕过反馈撤输出。

### STOP下进一步总线观测

- 用户切回STOP并要求继续。6轮域诊断确认STOP=True、无持权、Pitch仍离线，证据 `pitch-diagnose-20261114-121300.json`。
- 用已有只读monitor类型1快照采样12轮及48轮（921600，不改固件、不发运动命令）。证据在本会话临时目录 `pitch-bus-monitor-20261114-121827.json` 与 `pitch-bus-monitor-20261114-121906.json`。
- 在线总线ID数量均为8，接收丢帧计数0。除已匹配CAN1四轮 `0x201..0x204` 与Yaw `0x005`，实际捕捉到 **CAN1/0x207** 和 **CAN2/0x201**。因此CAN2接收链路确有数据，不是整个CAN2未启动；已配置的Pitch CAN2/0x205仍无有效反馈。CAN2/0x201按C620协议对应ID1，但该帧所属物理电机尚未确认，不能直接把它认作Pitch并改映射或发电流。
- 监控是“最后一帧”抽样，不是完整ID抓包；尚未捕获第8个在线ID，也不能据此断言不存在其它帧。下一步确认Pitch电调实际ID与总线归属，再作最小映射变更；不靠增大PID或关闭反馈校验处理。

### 摇杆反向 + IMU 角度保持（2026-11-14 晚）

- 用户反馈“右杆 X/Y 正反是反的”：新增 `yaw_stick_sign`/`pitch_stick_sign`（追加在 `RemoteMappingConfig` 末尾，保持既有聚合初始化顺序），只翻转云台 yaw/pitch 操纵方向，不动摩擦轮/拨弹/自转档判读；板级部署置 -1/-1。
- 用户选择方案①“松杆保持俯仰角”：`DmSlot::hold_angle_kp`（pitch=3.0，待整定）。执行层 `step_dm_velocity` 增加 IMU 角度外环：摇杆非零时按 `value×dt` 积分角度目标，零杆时 `err×Kp` 给速度；反馈用 `roll_pitch_yaw_rad[0]`，**不用电机圈数**；首次接管以当前实测倾角为目标；输出受轴速度量程限幅，角度目标受关节 ±15° 限位 clamp；IMU 无效/过期/非有限时撤输出并 disable，不静默退化成纯速度。
- 测试: Host **46/46 通过**（新用例改为允许外环落地速度在满杆内、并按 12bit 量化容差校验）。ARM 构建与镜像审计 pass，label `app-infantry-pitch-hold`，Flash 237376 B，`.data` LMA `0x0802f920`，完整校验映像 SHA256 `309ba04eb205e4266b6fef6d200a949ee830c76fce912dc593c1190aeaa6aa61`。
- **已烧录并完整校验**（证据 `pitch-flash-20261114-135508/`）。回读: Pitch 槽10 CAN1/0x011 在线 age 0ms、温度27/26；Yaw 在线；参数 mit_kp=0/mit_kd=1/ttl20ms，Yaw 3/0.6/F0/0.05；STOP、无持权、无执行。
- 未做动力验收。`kd=1`、`hold_angle_kp=3` 均为待调值；方向与 ±15° 限位需在急停就位下逐项确认。

### 实物Pitch是达妙J4310（CAN1）：接入遥控速度控制并烧录

- 用户确认 Pitch 接在 **CAN1**（不是 CAN2），并把电机设为 CAN ID 1 / Master 0x11。源码改为 `dm[1]`：bus=1、can_id=0x001、master_id=0x011。
- 用户要求“可以动起来，用遥控器操作”。实现为**速度域**：MIT `{pos=0, vel=目标速度, kp=0, kd=1, τ=0}`，由电机内部 kd 产生跟踪/阻尼力矩，不把电机圈数当俯仰角；不使用旧工程那段被注释的 DM 增益。
- 新增 `InfantryExecutor::step_dm_velocity`：独立使能→确认 Enabled→下发 MIT；撤输出走 disable，重用 yaw 槽同一 DM 调度周期但使能/latch 分开 (`dm_velocity_*`)。`send_dm_frame(slot,…)` 按槽取总线并只累加发送统计，不与 yaw 的 `pitch_stats_` 混。
- 应用层速度量程改为按承载通道取（DJI 或 DM 槽）；`DmSlot` 新增 `max_axis_velocity_rad_s`（46.2 deg/s）与 `feedback_only`（本版未启用）。摇杆量程恢复 46.2 deg/s，分段律仍由映射器实现。
- 测试: Host **46/46 通过**，含新的真实 `DR16→Cycle→执行器→MIT 编码` 用例（速度目标=负满杆、kp=0、kd>0、τ≈0、位置≈0、回中零速、STOP 撤输出）。修正了两处旧断言副作用：yaw 缺标定用例不再用全局 `dm_command_sent` 判断、缺省 DM 反馈用例改为只看 yaw 统计。
- 镜像: label `app-infantry-pitch-dm-velocity`，Flash 236368 B，`.data` LMA `0x0802f530`/42528 B，完整校验映像 SHA256 `0fa7653d2584c22dd194434bf82c9f71be0f74987776025b22baae2c845d16a9`。**已烧录并完整校验**（证据 `pitch-flash-20261114-134601/`）。
- 烧录后回读: Pitch 槽10 CAN1/0x001/反馈0x011 **online=true、age 1ms**、温度27/25°C；参数 `mit_kp=0`、`mit_kd=1`、ttl 20ms；Yaw 3/0.6、F0、0.05 不变。当前 STOP、无持权、无执行。
- 未做动力验收: kd=1 是**待调值**（不是旧工程实测），方向/大小/±15° IMU 限位需在急停就位下逐项核对。

### 实物Pitch是达妙J4310：改第二DM槽并烧录（反馈接入阶段）

- 用户纠正: Pitch 实物是达妙 J4310 且**未接到 C 板**; 之前把 `0x201` 当成 Pitch 反馈是错的, CAN2 上 `0x201/0x202` 属于其它电机, 该映射已撤销。
- 用户把 Pitch 设为 CAN ID 1 / Master ID 0x11 后要求“改下然后烧录”。按用户选择采用**只接入反馈**方式: MIT kp/kd/力矩全 0, 不使能、不下发 MIT、不发 disable。
- 结构改动: `dm_slot_count` 由 1 改为 2; `DmSlot` 新增 `feedback_only` 标志; yaw 仍在 `dm[0]`, pitch 在 `dm[1]`(CAN2, 0x001/0x011, feedback_only)。`stop_action_for`、Cycle 反馈刷新、健康判定、`joint_feedback`、参数槽、设备页、观测视图、部署摘要均由“只看 dm[0]”改为按槽遍历。参数槽位编号由 `domain_count` 改为 `dji_slot_count+dm_slot_count`, 协议载荷上限由 240 提到 256(UART 帧同上限)以容纳 11 槽设备页。
- 配置校验: `feedback_only` 的 DM 槽明确保持 `incomplete`, 不取得执行资格; 摇杆 pitch 量程置 0, 不产生目标（否则整包会因一个不可执行域被拒）。测试先行失败于旧断言, 修正后 **Host 46/46 通过**, ARM 构建与镜像审计 pass。
- 镜像: label `app-infantry-dm-pitch-rx`, Flash 234568 B, `.data` LMA `0x0802ee28`/42528 B。ELF SHA256 `ee1edbc47577b0c1ff6977589a8994fe0920a0e4cc26c5ac1454baa1bb3080fe`, BIN `0239802957f590bc6f2db5f5e9b93a938006d9b4fc7147de9ffbb40c7e2517ed`, 完整Flash映像 `3e0dea87b572d33a675b846c210542d7aacd360702f2412404bbc728c1a70107`。
- **烧录成功**: 新鲜 STOP/无持权/DM Disabled 预检后, 100kHz 写入并完整校验 234568 B(含 `.data`), `.text matched`, reset run。证据 `pitch-flash-20261114-132017/`。
- **Pitch 仍离线**: 复位后只读回读, 设备页槽10(pitch) 已正确显示 CAN2/协议128/device_id 1/反馈ID 0x011, 但 `online=false`、无反馈年龄; Yaw 槽9 在线。200 次长窗抽样 CAN2 只有 `0x201`/`0x202`, 无 `0x011`, 也无 `0x001`。参数回读正确: pitch MIT kp/kd=0、ttl=20ms, yaw 3/0.6、F0、0.05。
- 结论: 固件侧配置已就绪, 但总线上没有达妙反馈。需核对 Pitch 电机的 CANH/CANL 是否接到 C 板 CAN2、24V 供电、Master ID 是否真的为 0x11。反馈接入阶段固件不使能也不发帧, 因此**不会主动唤醒未接/未上电的电机**。
- 未发送运动目标、enable 或 START; 未做动力验收。源码与本文档未提交、未推送。

- 用户明确要求“改成201把，看看接受成功”。已把共享部署配置与板级接线表改为CAN2 C620 ID1，反馈 `0x201`、电流发送 `0x200` 第一槽，PID/传动/限位不变；未改电调自身ID。镜像审计地址摘要同步从 `0x502` 改成 `0x102`。
- 更新部署与真实CAN链测试，先实际RED于仍是ID5的部署断言，改配置后全量Host **46/46通过**；ARM构建、镜像审计pass。最终固件label `app-infantry-pitch-id1`，Flash233568 B，`.data` LMA `0x0802ea40`、42528 B。ELF SHA256 `8263c9ddcb2132016b731ab1874111c71fa89542c516a46012a3e02ac33f808a`，BIN SHA256 `cf6f8e710f86014942fea50022462f857cf40e4a9134e684c5db27013669ac40`，完整Flash映像SHA256 `cbd47965ebdc3ac94c77905bc48fcf3c3fc7daf1c28dd8ae07e644d6ca78aea8`。日志为会话tmp下 `pitch-id1-*`，`pitch-image.json`已指向本ID1版。
- 首次烧录尝试在串口预检阶段退出：指定串口及 `/dev/serial/by-id/` 均不存在，未执行SWD擦写。失败现场目录 `pitch-flash-20261114-123526/`。用户接回USB后再次要求烧录，本次实时STOP/无持权/DM Disabled预检通过。
- **ID1版烧录成功**：证据目录 `/home/kekeyuyu/.claude/jobs/13f1f33a/tmp/pitch-flash-20261114-123728/`。固定探针100kHz、备份当前1MiB Flash、写上述新ELF、完整233568 B校验（含 `.data`）与 `.text matched`，GDB退出0后reset run；PID回读与上述基线一致。烧录前备份SHA256 `2284ebe36141a048c5c5972f3ad83012354d66ce1e2697f1e81db2210991d2b8`。
- **反馈接收成功**：烧录后另开只读会话连续6轮，Pitch均为CAN2/反馈 `0x201`、online=True，反馈年龄0–1ms、速度0rad/s、温度26°C。所有轮次STOP=True、authority=False、Pitch execute=False。证据 `pitch-diagnose-20261114-123823.json` 位于同一会话tmp目录。旧ID5映射不匹配导致反馈缺失的诊断已由新映射上线验证闭环。
- 未发送运动目标、enable或Sweep START，未做Pitch动力验收。源码未提交、未推送；后续先核对机构方向与限位，不用反馈在线代替动力结论。

## 四、 下一步

1. 本轮参数迁移、离线验证、烧录和默认参数回读均已完成，不重复烧录或重做旧摩擦试验。后续修改源码必须重新构建与固定新hash。
2. 下一阶段是单独获授权的Pitch动力核对：在物理急停、机构安全和新鲜反馈条件下确认正反方向、回中与±15°方向限位。烧录成功不代替该动力验收。
3. 若需要恢复，当前会话临时目录有原Flash备份及烧录脚本；任何新的擦写仍先核验实时STOP/Disabled，不使用本记录中的旧快照替代。
