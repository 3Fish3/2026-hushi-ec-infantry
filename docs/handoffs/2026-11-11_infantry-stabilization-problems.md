# 步兵单云台稳向工程 — 问题报告

- **生成时间**：2026-11-11 23:11:03
- **项目根**：`/home/kekeyuyu/projects/步兵/infantry`
- **Git 分支**：`main`（HEAD `9c1a141`）
- **代码状态**：工作区**大量未提交**（详见交接报告），不能直接 clone 得到当前代码
- **本报告范围**：如实记录已知问题、未验证项与风险。不构成"可上车"结论。

---

## 一、总体结论

**主机软件链路已跑通，实车未验证。**

| 项 | 结论 |
|---|---|
| 主机 40 项契约测试 | 通过 |
| 世界稳向闭环（合成模型） | 通过，非实测 |
| ARM 固件编译 | 通过（`app-infantry-observe`，BIN 206,704 B） |
| 镜像离线审计 | 通过（5 DJI + 1 DM 地址与装配一致） |
| 实车反馈核对 | **未做** |
| 烧录 / 上电 / 动力 | **未做** |
| 实车 PID 整定 | **未做** |

---

## 二、必须知道的功能缺口

### P1 稳向参数未填，yaw 当前不执行

部署配置 `infantry_deployment_config()`（`robots/infantry/app/infantry_config.hpp:204`）**刻意没有**填 `single_gimbal_imu`：

- IMU 安装旋转矩阵、静态零偏未知；
- 角度环 / 速度环 PID 无实车依据；
- 力矩方向符号未标定。

配置校验因此把 `main_yaw` 判为 `incomplete`（`infantry_config.cpp:382`），yaw 不取得执行资格。这是**有意为之的安全缺项**，不是遗漏。测试里的合成 PID 位于 `tests/infantry_direction_tests.cpp`，**未写入部署表**。

后果：现阶段烧录后**世界稳向不会工作**，云台不会自动保持地面朝向。

### P2 pitch 无法驱动

pitch 是 M3508 + C620，按框架约定关节域只允许位置模式，但部署配置**没有给出位置环增益**（`infantry_config.cpp` 中 `dji[4]` 传 `needs_position=false`）。pitch 域为 `incomplete`，推杆不会动。

### P3 本地操纵量程为零

`boards/infantry_f407/include/board_app_config.hpp` 的 `deployment_mapping()` 只置 `infantry_controls=true`，其余量程全为 0：

- 摇杆、键鼠均不产生运动目标；
- 小陀螺转速 `chassis_wz_max_rad_s` 也为 0。

同样是有意为之：未标定量程不得冒充已标定。**上车前必须实测回填。**

### P4 世界稳向与 IMU 六轴漂移

姿态估计使用六状态 EKF，**Z 轴零偏不可观测**（`include/ec/estimation/attitude.hpp:92`）。没有外部航向参考时，长期世界 yaw 仍会缓慢漂移。当前没有磁力计或视觉参考。合成测试 64 秒无漂移，不能代表长时间实车表现。

### P5 达妙 25 rad 窗口的实机语义未证实

- 反馈展开：按相邻样本差超过半窗补 ±整窗（`infantry_execution.cpp` 的 `dm_continuous_position`）；
- 该实现**假设**设备反馈以 25 rad 为周期回绕；
- 仿真显式采用这一假设并通过多圈闭环，**但仿真不是设备行为证据**；
- 失联后累计圈数无法仅凭一帧恢复；
- 纯力矩命令不发送展开后的位置目标，因此不涉及"MIT 位置目标取模"，但设备在边界附近的内部行为仍未核实。

### P6 故障时的降级语义有限

- IMU 失效 → yaw 撤输出并尝试 disable，**不会**让底盘整车停止；
- 底盘方向链只依赖相对关节角，IMU 掉了仍可能继续平移；
- 只有底盘轮自身必需反馈/设备故障才会四轮一起撤输出（`infantry_faults.cpp:11`）。

### P7 无硬实时/断电安全保障

- CAN 断线或主控死机时，disable 命令可能发不出去；电调自身通信超时停机设置**未核实**；
- 力矩方向、IMU 安装方向配反会形成正反馈，代码**没有**自动识别；
- 软件限幅不替代物理限位、急停或供电切断。

### P8 键鼠映射只移植了基础部分

已实现：左中挡选择键鼠、WASD 平移、鼠标 X/Y 调云台、单帧增量只消费一次。
未移植：旧工程 Q/G 小陀螺、B/V 云台启停、Shift 加速、Z 零位校准、视觉自瞄。

### P9 仓库状态未整理（交接阻塞）

工作区有 195 项变更未提交，且暂存区里仍有 6 个哨兵旧文件的删除记录，同时新目录 `robots/infantry/`、`boards/infantry_f407/` 全部是未跟踪状态。**如果不处理，别人 clone 仓库拿到的是旧的哨兵版本，编译不出当前能力。**

---

## 三、修改过的核心文件（本轮）

| 文件 | 改动 |
|---|---|
| `robots/infantry/app/infantry_config.hpp` | 新增 `SingleGimbalImuConfig`、`chassis_follow_gain`、`infantry_controls`；`infantry_deployment_config()` 改为 `constexpr` |
| `robots/infantry/app/infantry_config.cpp` | `valid_single_gimbal_imu()`；yaw 域缺项判定 |
| `robots/infantry/app/infantry_perception.cpp` | 单云台 IMU 航向投影（不使用哨兵双云台反解） |
| `robots/infantry/app/infantry_execution.cpp` | 世界稳向双 PID → MIT 纯力矩；yaw 状态观测；`world_ready` 检查 |
| `robots/infantry/app/infantry_cycle.cpp` | 云台系→机体系平移旋转；步兵拨杆映射适配仲裁器 |
| `robots/infantry/app/infantry_remote.cpp` | 步兵右拨杆（上普通/中停止/下小陀螺）、基础键鼠 |
| `boards/infantry_f407/include/board_app_config.hpp` | 板级改为使用同源步兵装配表 |
| `boards/infantry_f407/include/board_app_observation.h` | 新增 `g_ec_infantry_yaw` SWD 观测结构 |
| `boards/infantry_f407/src/board_app.cpp` | 填充 yaw 观测 |
| `tests/infantry_direction_tests.cpp` | 新增：方向、超时、稳向闭环、倾斜投影、缺标定不驱动 |

---

## 四、本轮暴露过的自身缺陷（已修）

供接手者了解踩过的坑：

1. **旋转符号写反**：`R(-ψ)` 会变成"枪管指左、车往右"。已改为 `R(+ψ)`，测试期望值同步更正。
2. **DM 展开只做一半**：曾同时用展开角比 ±12.5 判故障、又把展开角塞进 MIT 位置编码。现改为纯力矩路径，不再发位置目标。
3. **误删源码**：曾误删 IMU/应用层核心文件并破坏 CMake，已从备份恢复；恢复前副本保存在会话临时目录。
4. **点符号误用**：`Rotary`/`Rotary` 系误判已纠正为按配置字段判断。

---

## 五、验证证据（可复现）

```sh
cd /home/kekeyuyu/projects/步兵/infantry
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build/host
ctest --test-dir build/host --output-on-failure   # 40/40
```

ARM 固件（需完整 STM32CubeF4 1.28.3，见交接报告）：

```sh
cmake --build build/infantry-app-arm-Release --target infantry_app
```

产物：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.elf`（SHA256 前缀 `8e6b050b`）。

**以上均为离线验证。没有连接探针、没有烧录、没有上电。**
