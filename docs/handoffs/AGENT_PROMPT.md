请阅读本仓库 `docs/handoffs/` 下的两份文件后开始工作：

- `2026-11-11_infantry-stabilization-handoff.md`（交接记录）
- `2026-11-11_infantry-stabilization-problems.md`（问题报告）

---

```
Project: infantry — RoboMaster 单云台步兵电控（基于 Hushi EC 框架的独立副本）
Path: /home/kekeyuyu/projects/步兵/infantry
Stack: C++20 / STM32F407 / HAL / FreeRTOS / CMake+Ninja / arm-none-eabi-gcc
Branch: main（HEAD 9c1a141，但当前能力全部在未提交工作区，见下）

## Goal

这台车只有一个云台。需求是：
1. 小陀螺时底盘自转，云台要用 IMU 保持相对于地面的朝向（世界稳向）；
2. 前进方向始终等于云台枪管方向；
3. 底盘平移方向由 yaw 编码器给出的"云台相对底盘角"换算，不用 IMU 世界角。

## Context

哨兵框架被完整复制为本项目，并把 `sentinel` 全部改名为 `infantry`。
哨兵原本假设 main_yaw 没有绝对编码器、靠 IMU + fire_yaw/pitch 双云台反解；
步兵恰好相反：main_yaw 就是达妙绝对编码器，只有一个云台。
因此步兵走独立的单云台 IMU 航向路径，不复用 `serial` 双云台反解。

已实现的控制链：
- 世界稳向：云台 IMU 航向 → 角度 PID → 速度 PID → 达妙 MIT **纯力矩**命令
  （位置/速度/MIT Kp/Kd 全为 0，避免世界角被当成电机位置）
- 底盘方向：yaw 关节角做云台系 → 机体系旋转；关节角无效即停止平移

## What's Been Done

- 板级装配从空壳改为实体：CAN1 四轮 C620(1-4)、CAN1 达妙 yaw(0x005)、CAN2 C620 pitch(5)
- 板级与主机共用同一份 constexpr 装配工厂 `infantry_deployment_config()`
- 世界稳向双 PID 接入执行层，含 `world_ready` 检查、使能确认、disable 重试
- 步兵遥控：右上普通/右中停止/右下小陀螺；左中键鼠（基础 WASD + 鼠标）
- SWD 观测符号 `g_ec_infantry_yaw`（目标/实测航向、关节角、力矩、反馈年龄）
- 主机 ctest 40/40 通过；ARM 固件编译并通过镜像审计（BIN 206,704 B）

## Current State

⚠️ 工作区有约 195 项未提交变更。HEAD 仍指向旧的 sentinel 目录树，
clone 仓库拿不到当前代码。**第一件事就是整理并提交。**

未跟踪的新目录：robots/infantry/、boards/infantry_f407/、docs/robots/infantry/、
docs/boards/infantry_f407/、tests/infantry_*.cpp、examples/infantry_*.cpp
暂存区残留：6 个哨兵旧文件的删除记录

参数缺失（有意为之，不是 bug）：
- single_gimbal_imu 未填 → main_yaw 域 incomplete → 世界稳向不执行
- pitch 位置环增益未填 → pitch 不可驱动
- deployment_mapping() 量程全 0 → 摇杆/键鼠不产生运动目标

## What's Left

1. 整理仓库：确认新目录完整后，把重命名与功能改动一次性提交；验证 clone 后能构建并通过 40 项测试。
2. 准备完整 STM32CubeF4 1.28.3 SDK（当前 build/deps 下的副本未入 Git、部分子模块需重建），跑通 ARM 构建。
3. 实板只读核对（不使能电机）：烧录后 SWD 读 g_ec_infantry_yaw，
   手动转云台核对 imu_estimated_yaw_rad 与 joint_angle_rad 的方向和单位；
   静止采集陀螺零偏；确认达妙在线/使能。
4. 单轴低力矩调试：填入 IMU 安装/零偏、力矩方向、PID；物理急停保障下先只让 yaw 出力。
5. 最后才做小陀螺整车验证：低速自转 → 叠加平移 → 长时间观察漂移。

## Key Decisions

- 用纯力矩而非 MIT 位置目标：达妙反馈是 ±12.5 rad 有窗口，世界角当位置会在跨界时产生大误差。
- IMU 世界航向只用于云台稳向；编码器相对角只用于底盘换算，两链不互相替代。
- 未标定即缺项：参数留空/为零，域判 incomplete 不执行；合成 PID 只存在于测试文件。

## Key Files

- robots/infantry/app/infantry_config.hpp:204 — 装配工厂 + SingleGimbalImuConfig
- robots/infantry/app/infantry_execution.cpp:616 — world_yaw/world_ready/双PID→力矩
- robots/infantry/app/infantry_perception.cpp — 单云台 IMU 航向投影
- robots/infantry/app/infantry_cycle.cpp:133 — 云台系→机体系旋转；:200 — 拨杆仲裁适配
- robots/infantry/app/infantry_remote.cpp — 步兵挡位与键鼠
- boards/infantry_f407/include/board_app_config.hpp:83 — 板级装配
- boards/infantry_f407/include/board_app_observation.h:73 — g_ec_infantry_yaw
- tests/infantry_direction_tests.cpp — 方向/超时/稳向闭环/倾斜/缺标定
- docs/boards/infantry_f407/步兵稳向接入与观测.md — 构建与观测说明

## Gotchas

- 不要把测试里的合成 PID 抄进部署配置当"能跑的参数"。
- 不要把世界航向写进 MIT 位置字段。
- g_ec_infantry_yaw 非停机读取要前后各读一次 sequence，只接受相同偶数。
- 步兵右中=停止、右下=小陀螺，与框架原有上下语义相反；mapper 和仲裁器视图都要改。
- 主机测试需 -DPython3_EXECUTABLE="$PWD/.venv/bin/python"（gm6020 用例依赖 scipy）。
- 六状态 EKF 的 Z 轴零偏不可观测，长期世界 yaw 仍会漂，没有外部航向参考。
- 从未烧录、从未上电；所有通过项都是离线验证。
```
