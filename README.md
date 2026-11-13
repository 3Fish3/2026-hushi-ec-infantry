# 步兵电控工程（infantry）

基于 Hushi EC 核心框架（取自哨兵重构工程）建立的**单云台步兵**电控工程。
手写代码 C++20，实时路径无动态分配、无异常；注释使用简体中文。

## 需求与契约

**方向契约**：底盘前进方向跟随云台前方；小陀螺时云台使用 IMU 保持相对于地面的朝向。世界稳向双 PID 已接入主机控制链并通过合成闭环测试，实车参数与板级接入仍未完成。

底盘平移方向只消费"云台相对底盘的关节角"这一个量；不使用 IMU 航向、
不使用单圈相位、不在两套角度域之间混用。

实现要点（`InfantryConfig::chassis_follow_gain` 非零时生效）：

- 本地遥控的平移意图按**云台系**表达，进应用层前旋到机体系：`v_body = R(+ψ)·v_gimbal`，
  `ψ = chassis_follow_gain × 云台关节角`。`adopted_twist_` 与轮式里程计仍是机体系，不动。
- 云台关节角无效时**不产生平移意图**（宁可不动，不按错的参考走）。
- 步兵 `main_yaw` 目标使用云台 IMU 的世界航向（`YawFrame::local_inertial`）。单云台方向直接来自校准后的姿态及枪管水平投影，不依赖哨兵双云台的 `serial` 反解。
- 角度 PID（rad → rad/s）与速度 PID（rad/s → N·m）输出 MIT 纯力矩命令，位置、速度、MIT Kp/Kd 均为零。使能确认、停止和 IMU 参考重建沿用真实执行链；IMU 无效时不以关节位置保持冒充世界稳向。
- 可在线调节沿 PID 力矩方向的平滑启动补偿，默认幅值为零（关闭）；叠加后仍受速度环原力矩上限约束。参数及实测边界见[启动补偿说明](docs/boards/infantry_f407/步兵稳向接入与观测.md#云台摩擦启动补偿)。
- `single_gimbal_imu` 必须提供安装旋转、静态零偏、水平投影阈值和两组 PID；缺失时 yaw 域不完整、不使能。测试中的合成 PID 不写入部署表。旧工程角度单位为度，且采用误差微分/死区，与当前 PID 不等价，不直接复制其数值。
- 达妙反馈展开的 25 rad 周期仍需实机核对。仿真显式采用该假设，已覆盖多圈闭环；纯力矩命令不发送展开后的位置目标，不需要对 MIT 位置目标取模。失联后原累计圈数无法仅凭一帧恢复，真实设备边界及重连行为尚待实测。

## 正弦整定试验的中断语义

已开始的正弦试验在停止挡、遥控失联、反馈/参考失效后取消，不因恢复运行挡自动换基准起振；
需要明确重新启动试验。停止挡可配置尚未开始的试验。正常运行且反馈连续有效时，增益热写和观测开关
不会主动撤销 yaw 目标。试验取消不等于永久失能，普通控制仍遵循既有仲裁。
详见[中断调查记录](docs/handoffs/2026-11-12_sweep-interruption-investigation.md)。

## 当前状态（如实标注）

| 项 | 状态 |
|---|---|
| 核心库（ec_core）主机编译与契约测试 | 通过 |
| 步兵部署配置（`infantry_deployment_config`） | 配置校验 `startable=1`；yaw IMU/PID 标定及 pitch 位置环参数缺项，两个域尚不可驱动 |
| 方向契约（前进 = 云台前方）主机测试 | 通过（`infantry_direction_contracts`） |
| 板级固件 | 已接入步兵装配表；ARM 编译及镜像检查通过，未烧录 |
| 实车轮向、零位、量程 | **未标定** |
| 烧录、动力、发弹 | **未执行，需单独授权** |

## 装配映射（来自旧工程工作树，非实测）

| 域 | 设备 | 协议 | 总线 / ID |
|---|---|---|---|
| `chassis_lf/rf/lb/rb` | M3508 + C620 ×4 | DJI 电流 | CAN1 / 1–4 |
| `main_yaw` | 达妙 J4310 | DM MIT | CAN1 / 0x005（收发同 ID） |
| `pitch` | M3508 丝杆 | DJI 速度 | CAN2 / 5 |
| `fire_yaw`、摩擦轮、拨弹 | **未装配** | — | — |

几何：轴距/轮距 400 mm，轮半径 60 mm，云台居中。
`report.complete` 在步兵恒为 false——框架要求全部 10 个域齐备；`startable` 仅表示配置可装配，
不代表各域参数齐全或已经通过实车验证。

## 步兵遥控映射

在步兵装配配置启用的映射下，左拨杆上挡选遥控，右拨杆上挡为普通模式、中挡为停止、
下挡为小陀螺。普通和小陀螺模式下左摇杆控制底盘平移，右摇杆控制云台；小陀螺转速来自
`chassis_wz_max_rad_s`，需要按实车标定。停止挡不操作摩擦轮、拨弹或零位校准，
步兵模式不使用哨兵的 ch4 发射机构层。左中挡已接入基础键鼠：WASD 平移、鼠标 X/Y 调整云台，
同一遥控帧的鼠标增量只消费一次；需要配置 `keyboard_chassis_speed_mps` 和
`mouse_yaw_rad_per_count` / `mouse_pitch_rad_per_count`，默认零表示未标定。
键鼠沿用右拨杆普通/停止/小陀螺选择，不移植射击和 Z 零位校准；旧工程 Q/G、B/V 和 Shift
快捷键尚未移植。板级已接步兵装配，但操纵量程尚未标定，保持零。

## 构建与测试（主机）

```sh
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

首次使用需装开发依赖（`scipy` 等，`gm6020` 辨识用例依赖它）：

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
```

## 已解决的框架契约问题

**DM 角色对调**：哨兵里 `main_yaw` 是无绝对编码器的 GM6020（靠 IMU + 串联模型反解），
DM 是 pitch；步兵恰好相反。框架把"DM 就是 pitch"写死在调度/停止边沿/统计上，且把
"`main_yaw` 就是惯性方向"写死在感知/编排/应用降级上。现按
`main_yaw_inertial_driven(config)`（即 `config.serial` 是否存在）分流，哨兵行为逐位不变。

**关节域必须有位置环**：`mode_allowed_for` 只允许关节域用 position 模式，但
`needs_position` 原先只对两个 yaw 置位，pitch（M3508 丝杆）缺位置环 → 轴无法使能。
现改为 `is_joint(domain)`，与模式白名单自洽。

**达妙命令/反馈同 ID**：框架原先拒绝 `can_id == master_id`，该假设来自 DJI
电调分组（命令组与反馈 ID 必然不同）。达妙 MIT 协议收发方向分离、载荷结构不同，
同 ID 合法；旧工程实测 TX/RX 均 `0x005` 可正常收发。已修正 `valid_config`
与布局校验，仅保留**跨设备**的 ID 互相占用检查，并同步更新测试断言。

## 未完成（不能声称可用）

1. **实车标定**：轮向、滚动方向符号、`chassis_follow_gain` 符号、yaw 机械零位。
2. **稳向标定参数**：IMU 安装/静态零偏、力矩方向和实车 PID 未填；pitch 位置环参数缺项。
   板级装配和 SWD 观测已接入，见[步兵稳向接入与观测](docs/boards/infantry_f407/步兵稳向接入与观测.md)。
3. **实车验证**：烧录、上电、动力、转云台、小陀螺长时间运行均未执行。

## 参考

- 上游框架文档：`docs/architecture`、`docs/guides`、`docs/modules`
- 旧工程（只作参数与能力参考，不继承实现）：`../gxnu_hushi_ec`
