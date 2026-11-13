# 正弦试验执行中断调查

## Context

2026-11-12，用户要求先调查增益/正弦参数写入前后出现的 `target=0、torque=0` 和试验中心变化，暂不继续扫频。当前分支 `main`，保留既有未提交工作区；本次没有暂存、提交、烧录、修改实机增益或提高力矩。

## 实板证据与结论边界

- 通过现有 `InfantryClient` 握手、读取状态，看到来源 remote、持权、yaw execute=true，正弦幅度参数回读为 0.2 rad。
- 尝试写幅度 0 以停激励，返回 `NOT_PERMITTED`，未绕过限制。该时刻 remote 正在持权，拒绝符合 `requester_may_configure()` 的既有规则。
- 随后只读串口 12 秒：120 条完整 CAL，无坏行；全部 execute=1、reason=0、target_valid=1，reprime=0。没有复现此前那次停顿。
- 早先将 CAL 第 17 索引列当成 target_valid 是错列：该列为 reason；字段索引 19 才是 target_valid。当前调查按字段名解析。
- 目前没有原始停顿时段的完整原因字段记录，**不能认定原始停顿由写参数造成，也不能认定是遥控失联**。

## 主机复现

在 `tests/infantry_direction_tests.cpp` 追加两个可区分的场景：

1. 遥控、IMU、电机反馈连续有效时，按实板相同次序执行参数事务：关观测、speed_kp=1.2、amp=0.2、frequency=0.5、开观测。检查每周期 yaw execute、目标存在性、reason、授权 generation、reference_epoch；通过。说明此参数生效核心路径本身不会撤销执行。
2. 不写任何参数，运行正弦后让遥控帧过期。执行正确撤销，但旧代码保持 sweep.enabled=true，重连后自动捕获新基准再激励。断言试验必须退出在旧实现失败：`!rig.cycle.sweep().enabled`。

这个场景复现了“停顿后换基准自动起振”的确定代码缺陷，**不是对原实板停顿原因的追认**。

## 本次修复

- `robots/infantry/app/infantry_cycle.cpp`：已开始的试验遇到遥控失联、人工停止、关节/IMU无效、电机故障、执行拒绝或参考代次改变时，取消 sweep，清基准与旧 yaw 保持目标。不因恢复运行挡自动重启正弦。
- `robots/infantry/app/infantry_cycle.hpp`：显式关闭/重新启用试验时重置基准，并记录首次捕获时的 yaw 参考代次。
- 仍允许在停止挡配置尚未开始的试验，首次明确进入可用运行态后捕获基准。
- 试验中断只取消正弦激励；恢复普通控制时仍按既有遥控、仲裁和反馈规则工作，不是永久失能锁存。
- 参数表的幅度回读是已配置值，不等价于试验仍在运行；中断后需要明确重新启动，不凭幅度非零假定已运行。

## 验证

```sh
cmake --build build/host
ctest --test-dir build/host --output-on-failure
cmake --build build/infantry-app-arm-Release --target infantry_app
```

结果：主机构建退出码 0，41/41 测试通过；ARM 构建及镜像审计退出码 0。新增测试还覆盖停止挡、IMU 失效、参考代次改变、电机超时和重连后不自动恢复，明确重新配置后可再次启动。

## 下一步

1. 用户先将右拨杆置中挡，确认停止。左中挡是键鼠，不是待机。
2. 在停机条件下确认正弦已关闭，再决定是否烧录此次修复；本轮未烧录，板上仍是原试验固件，幅度停止命令此前被拒绝。
3. 后续若再次出现停顿，保存 CAL 原始流和完整行：t、execute、reason、tvalid、target、reprime，结合协议 status 的来源/持权状态定位。不靠 target=0 单独判断原因为何，也不使用包含无效目标的样本估算幅频响应。
