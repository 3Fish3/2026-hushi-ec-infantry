# 步兵电控工程（infantry）

基于 Hushi EC 核心框架（来自哨兵重构工程）建立的**单云台步兵**电控工程。
手写代码 C++20，实时路径无动态分配、无异常；注释使用简体中文。

## 需求与契约

**唯一方向契约**：任意情况下，云台前方 = 前进方向。

实现方式为**单一参考系**：底盘平移方向变换只消费"云台相对底盘的机械角"这一个量，
不使用 IMU 航向、不使用单圈相位、不在两套角度域之间混用。

## 当前状态（如实标注）

| 项 | 状态 |
|---|---|
| 核心库（ec_core）主机编译与契约测试 | 通过 |
| 步兵几何/方向契约测试 | 通过（主机，合成轮位） |
| 板级整车固件 | **未实现** |
| 实车轮向、零位、摩擦轮/拨弹映射 | **未标定** |
| 烧录、动力、发弹 | **未执行，需单独授权** |

主机测试与交叉编译不构成实机验收。

## 目录

```
include/ec/, src/           通用核心：时间、PID、速度/位置环、轴门面、运动学、姿态、CAN 编解码
robots/infantry/app/        步兵层
  infantry_config.hpp/.cpp    装配资料（电机协议/总线/ID、几何）——来自旧工程，非实测
  infantry_direction.hpp      云台系 → 底盘系几何变换
  infantry_yaw_angle.hpp      达妙 25 rad 回绕 → 连续关节角
robots/sentinel/            上游哨兵业务层，保留作参考，步兵不依赖其控制路径
tests/                      契约测试
docs/                       文档（含 sentinel 既有资料）
```

## 构建与测试（主机）

```sh
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host --target ec_infantry_direction_tests ec_core_tests ec_control_tests ec_axis_tests ec_dm_tests
ctest --test-dir build/host -R "infantry|core_contracts|control_contracts|axis_contracts|dm_contracts"
```

## 坐标与符号约定

- 底盘：`+X` 前、`+Y` 左、`+Z` 上；yaw 逆时针为正。
- 遥控：`forward_mps` 沿枪口前向，`right_mps` 沿枪口右侧。
- 变换：`body = R(-yaw) · gimbal`，其中 `yaw` 为云台相对底盘的逆时针夹角。
- 变换对 `yaw + 2πn` 免疫，因此任意圈数下前进方向仍正确。

## 已知不一致（需现场确认，勿按默认值上电）

1. **达妙 yaw 地址**：旧工作树使用 `command_id = master_id = 0x005`，
   而框架 `DmDevice` 拒绝两者相同。需确认实际电调地址或驱动契约。
2. **轮向与滚动方向**：`infantry_config.cpp` 只记录协议/总线/ID；轮位、滚动方向、
   增益符号均未标定，测试中的轮布局为合成数据。
3. **拨弹电机型号**：旧工程注释标注 M2006 但使用达妙 MIT 协议，
   需核对实物后再配置。
4. **25 rad 回绕**：为旧代码与实测观察到的行为，边界外的限幅/回绕策略需独立验证。

## 参考

- 上游框架文档：`docs/architecture`、`docs/guides`、`docs/modules`
- 旧工程（只作参数与能力参考，不继承实现）：`../gxnu_hushi_ec`
