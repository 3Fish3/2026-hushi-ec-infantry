# GitHub 步兵改版源码对照

## 一、范围与固定版本

- 用户提供：<https://github.com/588ttggx/infantry>。
- 本轮固定检查 `main` 的提交 `9642e55704824590082dc48f3e9e477f272e5bac`，提交说明为 `feat: add infantry IMU calibration and maintenance docs`；父提交 `16b8e1f5cd8229631435598bc93e360cb5a40c53`。
- 当前工程：`/home/kekeyuyu/projects/步兵/infantry`。仅对照源码，未合并或改动控制代码、未执行下载仓库的构建/测试/脚本、未访问硬件。源码归档经已配置的本机代理下载并安全解包到当前会话临时目录。

## 二、已核对的新增内容

- 最新提交的 17 个变更文件集中在独立 IMU 标定及文档，没有在该提交中改动 `robots/infantry/app` 的控制实现。
- 新增 `boards/infantry_f407/imu_calibration.cmake`、`src/imu_calibration.cpp`、`tools/imu_calibration.py`、`tools/infantry_imu_rtt.cfg`，以及标定文档和 Python 测试。
- `EC_BUILD_INFANTRY_IMU_CALIBRATION` 是独立构建角色，不与 APP/CAPTURE/S1 混用；其源文件列表不链接 CAN、FreeRTOS 或电机执行链。
- 标定固件直接读取 BMI088 原始三轴角速度、加速度及温度，以约 100 Hz 发送 `IMU1,seq,t_us,gx,gy,gz,ax,ay,az,temp_c` 到标准布局 RTT 通道 0。RTT 缓冲为 8192 字节；整行丢弃时序号仍推进，主机可检测丢样。
- 主机工具只使用 Python 标准库，连接本机 OpenOCD RTT TCP 9090；保留原始数据并输出有符号传感器坐标系零偏、噪声、温度及采样统计，支持离线 replay。
- 数据检查覆盖序号连续性、uint32 回绕、采样间隔 5–20 ms、平均频率 80–120 Hz、有限数、静止角速度/加速度及窗口长度。静态数据不用于证明安装矩阵、力矩方向或 PID 整定完成。

## 三、当前已经确认的兼容差异

- 两个源码树在选定的源码/配置/文档范围内，有 232 个文件逐字节相同、33 个共有文件有变化。远端独有 6 个文件，当前工程独有 27 个文件；这不是一次 Git 合并结果。
- 当前 APP 的 RTT/CAL 输出相关 `board_rtt.*`、`board_rtt_format.cpp` 及 UART 链路验证目标没有包含在远端快照中；远端 `board_app.cpp` 也没有当前 CAL 流输出代码。整仓覆盖会中断现有调参采集工具的输入。
- 核心世界稳向控制律相同：最短弧角度误差经角度 PID、速度 PID 输出 MIT 纯力矩；惯性航向与底盘平移所用编码器关节角解耦。遥控映射和 `R(+ψ)` 平移变换也已有相同实现。远端没有新增一种稳向控制律，电机装配与 CAN 地址同源。
- 远端部署工厂未填 `single_gimbal_imu`，其 yaw 域配置不完整；当前工厂已经填入安装、零偏与待整定 PID。远端 `board_app_config.hpp:73–76` 也没有当前 yaw 摇杆 1.5 rad/s 量程。直接覆盖会丢掉当前配置，而不只是换控制算法。
- 远端 `infantry_perception.cpp:152–156` 假设枪管沿云台 +X 并硬编码投影轴；当前配置/感知支持 `barrel_axis`，本车按测量采用 −Y。若直接沿用当前单位安装矩阵，远端投影的不是实际枪管，水平姿态下相差 90°；倾斜时还影响投影有效性和航向变化率几何。
- 陀螺单步上限远端为 20 ms，当前为 100 ms；这会改变长间隔触发姿态重建的边界，不能因提交时间较新而直接覆盖。两边 gyro_age 仍为 50 ms。
- 远端参数构造在 `infantry_params.cpp:113` 结束，没有当前 vehicle 槽局部号 10–20 的级联 PID、正弦、观测及启动补偿参数。当前 sweep 固定基准和中断取消逻辑、摩擦助力也不在远端。
- 当前 `apply_param`/`HostSession` 多了 `InfantryCycle*` 接入。只覆盖部分 app 文件会导致板级调用接口不匹配；整套替换则丢掉在线整定与 CAL 采集能力。本轮未做混合构建，结论来自签名与调用点静态对照。

## 四、恢复与下一步

- 对照远端文件请使用固定提交链接，避免后续 main 更新改变结论：<https://github.com/588ttggx/infantry/tree/9642e55704824590082dc48f3e9e477f272e5bac>。
- 推荐评估独立标定角色及其主机校验/回放工具，保留当前实机部署配置和控制链；本轮没有执行移植。
- 实机对照试验已暂停。此前第 2 组因 `no_authority` 中断，最后只读状态为无持权、各域不执行但人工停止标志无效；正弦/补偿参数尚未清理，详见 `2026-11-12_gimbal-friction-assist-14d24575.md`。不要把本次仓库查看当作再次开展动力试验的授权。
