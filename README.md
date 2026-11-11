# 步兵电控工程（infantry）

基于 Hushi EC 核心框架（取自哨兵重构工程）建立的**单云台步兵**电控工程。
手写代码 C++20，实时路径无动态分配、无异常；注释使用简体中文。

## 需求与契约

**唯一方向契约**：任意情况下，云台前方 = 前进方向。

底盘平移方向只消费"云台相对底盘的关节角"这一个量；不使用 IMU 航向、
不使用单圈相位、不在两套角度域之间混用。

> 注：`robots/sentinel/app/sentinel_kinematics` 的上游实现仍以 `main_yaw` 的
> 惯性/关节双路参考为核心；步兵首版的**方向契约与配置**已按单一关节参考建立，
> 但板级方向通路（`sentinel_cycle` 的 twist 装配）尚未改造成单参考，见"未完成"。

## 当前状态（如实标注）

| 项 | 状态 |
|---|---|
| 核心库（ec_core）主机编译与契约测试 | 通过 |
| 步兵部署配置（`infantry_deployment_config`） | 通过，`startable=1` |
| 板级整车固件 | **未实现** |
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
`report.complete` 在步兵恒为 false——框架要求全部 10 个域齐备，
可运行判据是 `startable`。

## 构建与测试（主机）

```sh
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host --target ec_infantry_deployment_tests ec_dm_tests ec_core_tests ec_control_tests ec_axis_tests
ctest --test-dir build/host -R "infantry_deployment|dm_contracts|core_contracts|control_contracts|axis_contracts"
```

## 已解决的框架契约问题

**达妙命令/反馈同 ID**：框架原先拒绝 `can_id == master_id`，该假设来自 DJI
电调分组（命令组与反馈 ID 必然不同）。达妙 MIT 协议收发方向分离、载荷结构不同，
同 ID 合法；旧工程实测 TX/RX 均 `0x005` 可正常收发。已修正 `valid_config`
与布局校验，仅保留**跨设备**的 ID 互相占用检查，并同步更新测试断言。

## 未完成（不能声称可用）

1. **板级方向通路**：`sentinel_cycle` 的 twist 仍按上游双 yaw 参考装配，
   尚未改造成"只用 `main_yaw` 关节角"的单参考实现。
2. **实车标定**：轮向、滚动方向符号、yaw 机械零位、关节限幅。
3. **CAN 收发与任务调度**：未接板级。
4. **25 rad 回绕**：为旧代码与实测观察到的行为，边界外策略需独立验证。

## 参考

- 上游框架文档：`docs/architecture`、`docs/guides`、`docs/modules`
- 旧工程（只作参数与能力参考，不继承实现）：`../gxnu_hushi_ec`
