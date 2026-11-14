---
date: 2026-11-14
updated: 2026-11-14
tags: [robomaster, infantry, yaw, pid]
---

# J4310 主云台航向 PID 离线整定

这项工具用步兵主云台的力矩 Sweep 数据，给出速度内环与航向外环的候选参数。主云台为达妙 J4310（CAN1 `0x005`）。C 板的航向外环输出目标角速度，速度内环根据 IMU 的世界航向角速度输出力矩，电机以 MIT 纯力矩模式执行。采集程序通过同一个串口执行试验和保存数据，拟合脚本直接处理电脑上的 CSV，全程无需连接步兵。

## 使用

在仓库根目录使用已安装 `requirements-dev.txt` 的 Python 环境。保留合作分支 [feat/m2-sweep-rc-return-20261113](https://github.com/3Fish3/2026-hushi-ec-infantry/tree/feat/m2-sweep-rc-return-20261113) 的 `infantry_sweep_trial.py` 生成的 schema 2 力矩模式 CSV，以及每份 CSV 同目录的 `*.csv.report.json`。

```powershell
python tools/fit_infantry_yaw_pid.py run01.csv run02.csv --speed-hz 1.5 --angle-hz 0.5 --output yaw-pid.json
```

文件名和频率是用法示例。两个频率指希望达到的速度内环和航向外环交越频率，均需落在有效测量频段内，且速度内环至少为航向外环的两倍。采集时固定底盘，记录固件身份、实际生效的两环增益和机械配置。组合多份记录时保持装配和试验工况一致。

## 数据与结果

- 需要至少 4 个合格的不同频点。每个频点去掉首尾淡入淡出后须保留至少两个稳态周期，原始计划至少 4 周期，5–6 周期更易保留充足的稳态数据。
- 脚本检查 CSV 中的自然结束状态、记录连续性和力矩限幅。参与拟合的频点要求同频正弦分量 RMS 至少为拟合残差 RMS 的 3 倍。归档中的旧实测只有 0.5 Hz、每组 3 周期，数据量达不到出参条件。
- 拟合使用力矩激励时的总提交力矩和 IMU 航向角速度，估计包含延迟的一阶响应，并逐一留出频点检查拟合误差。提交力矩是已进入 CAN 发送队列的命令值。候选增益须在固件参数范围内，且两环的模型相位裕度均至少为 45°。
- 当前固件两个环的积分限幅均为 0。脚本整定两环的 `Kp`，`Ki` 和 `Kd` 均给 0。结果是待复测的候选参数。独立角度 Sweep 可用来对比旧参数与候选参数的跟踪滞后、振荡及力矩峰值。脚本输出 JSON 报告，板上参数需通过原有流程单独验证与写入。

源码见[离线拟合脚本](../../../tools/fit_infantry_yaw_pid.py)与[固件电机和 PID 配置](../../../robots/infantry/app/infantry_config.hpp)。
