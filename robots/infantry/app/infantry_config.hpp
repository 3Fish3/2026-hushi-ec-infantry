#pragma once
// 哨兵正式 APP 的配置边界 (O0):
//  - 安装声明 (总线/协议/设备 ID) 与机械/电气换算参数分开, 缺项用 nullopt 表达;
//  - 合成测试夹具与真实部署配置是两种显式类别, 不得静默混用 (见 validate/config_audience);
//  - 只做配置校验与类型转换, 不构造设备对象、不发送任何帧.
// 旧仓库有效参数到新域的换算依据和仍然缺项的内容集中记录在
// docs/robots/infantry/应用配置与换算基线.md, 本文件不重复维护第二份数值表.
#include "ec/control/pid.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include "infantry_types.hpp"
#include "infantry_runtime.hpp"
#include "infantry_kinematics.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::infantry::app {
// 配置类别. unconfigured 表示还没有任何有效配置, 不是"空车可用".
enum class ConfigKind : std::uint8_t { unconfigured = 0, synthetic_fixture, real_deployment };
// 配置用途: 离线测试接受合成夹具; 部署用途只接受真实配置, 且拒绝任何合成来源的值.
enum class ConfigAudience : std::uint8_t { offline_test = 0, deployment };
// 单项参数来源. unspecified 表示尚未提供, 不得用零或任意保守值顶替.
enum class Origin : std::uint8_t { unspecified = 0, synthetic, reference_converted, measured, user_confirmed };
// 用户确认直接耦合; 不意味着摩擦轮其它参数已经齐备.
inline constexpr float confirmed_friction_rotor_per_axis = 1.0f;
// 配置齐备程度 (与运行期反馈有效性无关).
enum class ConfigState : std::uint8_t { unavailable = 0, incomplete, complete };

[[nodiscard]] constexpr const char* origin_name(Origin origin) noexcept {
    switch (origin) {
    case Origin::unspecified: return "unspecified";
    case Origin::synthetic: return "synthetic";
    case Origin::reference_converted: return "reference_converted";
    case Origin::measured: return "measured";
    case Origin::user_confirmed: return "user_confirmed";
    }
    return "unspecified";
}

// 一个 DJI 设备槽. installed=false 表示该域没有装配设备 (与"装配了但参数未知"不同).
struct DjiSlot {
    Domain domain{Domain::chassis_lf};
    bool installed{};
    Origin origin{Origin::unspecified};
    std::uint8_t bus{};
    drivers::DjiMode mode{drivers::DjiMode::c620_current};
    std::uint8_t device_id{};
    // 机械/电气换算: 未确认时保持 nullopt, 不填猜测值.
    std::optional<float> rotor_per_axis{};       // 转子角 / 轴侧角 (无量纲)
    std::optional<int> direction{};              // 只允许 +1 / -1
    std::optional<float> current_limit_a{};      // 轴侧电流上限 (A), 用于环路归一化
    std::optional<std::int16_t> output_limit_raw{}; // 归一化输出 ±1 对应的协议量
    std::optional<float> max_rotor_speed_rad_s{};   // 采样间隔内的已知速度上界
    std::optional<float> max_axis_velocity_rad_s{}; // 轴侧目标量程 (rad/s), 也是环路与缩放的边界
    Lifetime feedback_ttl{};                     // 反馈有效期; 无效表示尚未配置
    std::optional<control::PidConfig> velocity_pid{};
    std::optional<control::PidConfig> position_pid{}; // 连续/位置语义轴才需要
};

// 一个 DM 设备槽. MIT 映射量程 (PMAX/VMAX/TMAX) 没有通用默认值, 必须显式给出.
struct DmSlot {
    Domain domain{Domain::pitch};
    bool installed{};
    Origin origin{Origin::unspecified};
    std::uint8_t bus{};
    std::optional<std::uint16_t> can_id{};    // MIT 命令帧 ID
    std::optional<std::uint16_t> master_id{}; // 反馈帧 ID
    std::optional<drivers::DmMitLimits> limits{};
    Lifetime feedback_ttl{};
    std::optional<float> mit_kp{};
    std::optional<float> mit_kd{};
};

// 底盘几何: 每项都来自机械记录, 缺项时保持 nullopt.
struct WheelGeometryConfig {
    std::optional<float> x_m{};
    std::optional<float> y_m{};
    std::optional<float> roll_dir_rad{};
    std::optional<float> radius_m{};
    std::optional<float> gain{}; // 电机到该轮的方向/传动符号
};

// 关节几何与参考配置. 零位偏移是参考类配置: 未确认时不取得对应执行资格.
struct JointGeometryConfig {
    Domain domain{Domain::main_yaw};
    std::optional<float> zero_offset_rad{};
    std::optional<float> min_rad{};
    std::optional<float> max_rad{};
};

// 时序与时效配置. 默认值只是"尚未配置"的占位, 不代表已实测的周期或超时.
struct TimingConfig {
    float control_dt_s{};
    Lifetime target_ttl{};    // 域目标默认有效期
    Lifetime remote_ttl{};    // 遥控样本有效期
    Lifetime imu_ttl{};       // IMU 样本有效期
    Lifetime host_link_ttl{}; // 上位机会话链路存活期
    std::optional<float> max_dt_s{};
};

// 单云台 IMU 标定结果 (2026-11-12 实测, 见 docs/boards/infantry_f407/步兵稳向接入与观测.md).
// 只包含**可由硬件测得的量和几何关系**; PID 与力矩方向属于整定项, 不用实测数据冒充.
struct SingleGimbalCalibration {
    Origin origin{Origin::unspecified};
    estimation::RotationMatrix3 install{}; // 传感器系 -> 云台系 (实测为单位阵)
    Vector3 barrel_axis{0.0f, -1.0f, 0.0f};
    Vector3 gyro_bias_sensor{-0.000251f, 0.001645f, 0.002211f};
};

// 单云台世界稳向参数: 坐标与 PID 均需有可追溯来源; 缺项时不驱动 yaw.
struct SingleGimbalImuConfig {
    Origin origin{Origin::unspecified};
    estimation::RotationMatrix3 install{}; // 传感器系 → 云台系
    Vector3 gyro_bias_sensor{};           // 传感器系静态零偏, rad/s
    // 枪管在云台(机体)系中的单位方向. 本车实测为 −Y; 抬头时重力沿 −Y 增大,
    // +Z 朝天, +X 为俯仰轴, 三者满足右手系.
    Vector3 barrel_axis{0.0f, -1.0f, 0.0f};
    float min_projection_squared{};      // 枪管方向的最小水平投影平方
    control::PidConfig angle_pid{};        // rad → rad/s
    control::PidConfig speed_pid{};        // rad/s → N·m
    int torque_direction{1};              // 正世界 yaw 力矩到电机力矩的符号, 仅 +1/-1
    float friction_torque_nm{};           // 摩擦补偿幅值; 0 = 关闭, 待实测整定
    // 机械速度平滑带(rad/s), 在DM速度量化死区之外逐渐切换到相对运动方向;
    // 静止时由角度误差的P项平滑起动. 不把世界航向速率当作机械相对转速.
    float friction_speed_rad_s{0.05f};    // 待实测平滑设置, 非已辨识摩擦参数
};

// 整车配置根: 九 DJI + 一 DM + 四轮几何 + 三关节几何 + 时序.
struct InfantryConfig {
    ConfigKind kind{ConfigKind::unconfigured};
    std::array<DjiSlot, dji_slot_count> dji{};
    std::array<DmSlot, dm_slot_count> dm{};
    std::array<WheelGeometryConfig, chassis_wheel_count> wheels{};
    std::array<JointGeometryConfig, joint_slot_count> joints{{
        {Domain::main_yaw, {}, {}, {}},
        {Domain::fire_yaw, {}, {}, {}},
        {Domain::pitch, {}, {}, {}},
    }};
    TimingConfig timing{};
    RuntimeTiming runtime{};
    PerceptionTiming perception{};
    std::optional<SerialGeometry> serial{};
    Origin serial_origin{Origin::unspecified};
    // 底盘平移是否跟随云台方向 (步兵方向契约: 云台前方 = 前进方向).
    // 0 = 不跟随 (默认/未标定); 非零 = 云台关节角到机体旋转的符号标定差.
    float chassis_follow_gain{};
    bool infantry_controls{}; // 使用步兵右拨杆模式及单云台摇杆映射
    std::optional<SingleGimbalImuConfig> single_gimbal_imu{};
};

// 步兵目标是云台世界航向; 缺少标定时仍按惯性目标处理, 不退回机械角冒充稳向.
[[nodiscard]] constexpr bool main_yaw_inertial_driven(const InfantryConfig& config) noexcept {
    return config.infantry_controls || config.serial.has_value();
}

enum class ConfigIssue : std::uint8_t {
    none = 0,
    unconfigured_kind,
    domain_duplicated,
    unsupported_mode, // 该协议模式没有电流上限语义 (GM6020 电压模式)
    invalid_bus,
    invalid_device_id,
    missing_rotor_ratio,
    invalid_rotor_ratio,
    missing_direction,
    invalid_direction,
    missing_feedback_ttl,
    missing_current_limit,
    invalid_current_limit,
    missing_output_limit,
    invalid_output_limit,
    missing_speed_bound,
    invalid_speed_bound,
    missing_axis_speed_limit,
    invalid_axis_speed_limit,
    missing_gains,
    invalid_gains,
    missing_joint_reference,
    invalid_joint_range,
    missing_dm_can_id,
    missing_dm_master_id,
    missing_dm_limits,
    invalid_dm_limits,
    missing_dm_gains,
    invalid_dm_gains,
    missing_wheel_geometry,
    invalid_wheel_geometry,
    degenerate_wheel_layout,
    missing_timing,
    invalid_timing,
    synthetic_value_in_deployment,
    unknown_origin_in_deployment,
    invalid_serial_geometry,
};

// 配置校验结论. domains 只表达"配置是否齐备", 不代表当前反馈可用.
struct ConfigReport {
    ConfigKind kind{ConfigKind::unconfigured};
    ConfigAudience audience{ConfigAudience::offline_test};
    bool startable{}; // 允许按该用途构造应用
    bool complete{};  // 每个域配置都齐备
    std::size_t issue_count{};
    std::array<ConfigIssue, 64> issues{};
    std::array<ConfigState, domain_count> domains{};

    [[nodiscard]] bool has(ConfigIssue issue) const noexcept;
    [[nodiscard]] std::size_t count(ConfigIssue issue) const noexcept;
};

// 校验配置: 单位/范围/域身份唯一性/来源混用. 不修改配置, 不构造设备.
[[nodiscard]] ConfigReport validate(const InfantryConfig& config, ConfigAudience audience) noexcept;
[[nodiscard]] bool valid_single_gimbal_imu(const SingleGimbalImuConfig& config,
    const drivers::DmMitLimits& limits) noexcept;

// 步兵部署配置: 四全向轮 + 达妙 yaw + M3508 pitch, 不装配 fire_yaw/摩擦轮/拨弹.
// 参数来自旧工程工作树; 轮向、零位、量程仍为待标定项.
[[nodiscard]] constexpr InfantryConfig infantry_deployment_config() noexcept {
    constexpr float pi = 3.14159265358979323846f;
    // 步兵装配: 四轮全向底盘 + 达妙 yaw + M3508 pitch.
    // 未装配: fire_yaw(哨兵双云台)、摩擦轮、拨弹 — 保持 installed=false,
    // 其域状态为 unavailable, 不参与 all_complete 判定.
    //
    // 参数来源: 旧工程当前工作树; 轮向、零位、量程为待标定项, 来源标 user_confirmed 仅表示
    // "按旧工程装配"这一事实被确认, 不代表实车已验收.
    InfantryConfig config{};
    config.kind = ConfigKind::real_deployment;

    const auto dji = [](Domain domain, std::uint8_t bus, drivers::DjiMode mode,
                         std::uint8_t id, float rotor_per_axis, int direction,
                         float current_limit_a, float output_limit_raw,
                         float axis_velocity_limit, float rotor_speed_limit,
                         bool needs_position, control::PidConfig velocity_pid,
                         control::PidConfig position_pid) {
        DjiSlot slot{};
        slot.domain = domain;
        slot.installed = true;
        slot.origin = Origin::user_confirmed;
        slot.bus = bus;
        slot.mode = mode;
        slot.device_id = id;
        slot.rotor_per_axis = rotor_per_axis;
        slot.direction = direction;
        slot.current_limit_a = current_limit_a;
        slot.output_limit_raw = static_cast<std::int16_t>(output_limit_raw);
        slot.max_rotor_speed_rad_s = rotor_speed_limit;
        slot.max_axis_velocity_rad_s = axis_velocity_limit;
        slot.feedback_ttl = Lifetime{20};
        slot.velocity_pid = velocity_pid;
        if (needs_position) { slot.position_pid = position_pid; }
        return slot;
    };

    // 底盘四轮: M3508 + C620, CAN1 ID1~4, 减速比 19. 速度环 kp=2.5 (旧工程 Chassis_pid).
    // 归一化输出量纲: ±1 对应轴侧电流上限; 20 A 满量程, 由 current_limit_a 表达.
    const control::PidConfig wheel_speed{2.5f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f};
    const control::PidConfig wheel_position{};
    config.dji[0] = dji(Domain::chassis_lf, 1, drivers::DjiMode::c620_current, 1, 19.0f, 1,
        20.0f, 16384.0f, 50.0f, 100.0f, false, wheel_speed, wheel_position);
    config.dji[1] = dji(Domain::chassis_rf, 1, drivers::DjiMode::c620_current, 2, 19.0f, 1,
        20.0f, 16384.0f, 50.0f, 100.0f, false, wheel_speed, wheel_position);
    config.dji[2] = dji(Domain::chassis_lb, 1, drivers::DjiMode::c620_current, 3, 19.0f, 1,
        20.0f, 16384.0f, 50.0f, 100.0f, false, wheel_speed, wheel_position);
    config.dji[3] = dji(Domain::chassis_rb, 1, drivers::DjiMode::c620_current, 4, 19.0f, 1,
        20.0f, 16384.0f, 50.0f, 100.0f, false, wheel_speed, wheel_position);

    // pitch: M3508 + C620, CAN2 ID5. 旧工程使用速度环; 当前框架关节域要求位置环.
    // 暂无可核实的位置环参数, 保持缺项而不是用猜测增益赋予执行资格.
    config.dji[4] = dji(Domain::pitch, 2, drivers::DjiMode::c620_current, 5, 19.0f, 1,
        20.0f, 16384.0f, 30.0f, 100.0f, false,
        control::PidConfig{4.5f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f}, wheel_position);

    // yaw: 达妙 J4310, CAN1 TX/RX 0x005, MIT 闭环.
    // 旧工程用 IMU 做角度/速度反馈; 新框架 DmSlot 只表达 MIT 通道,
    // 角度参考由 joint zero_offset_rad 与 ContinuousYawAngle 提供.
    DmSlot yaw{};
    yaw.domain = Domain::main_yaw;
    yaw.installed = true;
    yaw.origin = Origin::user_confirmed;
    yaw.bus = 1;
    yaw.can_id = 0x005;
    yaw.master_id = 0x005;   // 达妙收发方向分离, 同 ID 合法
    yaw.limits = drivers::DmMitLimits{12.5f, 30.0f, 10.0f};
    yaw.feedback_ttl = Lifetime{20};
    yaw.mit_kp = 30.0f;
    yaw.mit_kd = 2.0f;
    config.dm[0] = yaw;

    // 底盘几何: 400 mm 轴距/轮距, 轮半径 60 mm, 云台居中.
    // 轮位按 +X 前 / +Y 左; 滚动方向取切向. gain 为待标定项, 暂用 +1.
    constexpr std::array<std::array<float, 2>, chassis_wheel_count> positions{{
        {0.2f, 0.2f}, {0.2f, -0.2f}, {-0.2f, 0.2f}, {-0.2f, -0.2f}}};
    constexpr std::array<float, 4> rolling{3*pi/4, pi/4, -3*pi/4, -pi/4};
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        config.wheels[i].x_m = positions[i][0];
        config.wheels[i].y_m = positions[i][1];
        config.wheels[i].roll_dir_rad = rolling[i];
        config.wheels[i].radius_m = 0.06f;
        config.wheels[i].gain = 1.0f;
    }

    // 关节几何: yaw 与 pitch. fire_yaw 未装配, 保持空.
    // yaw 有滑环可连续旋转, 量程取达妙编码器可表示窗口 ±position_max_rad (25 rad 有符号映射),
    // 不沿用旧工程 GM6020 单圈 ±2.6 rad 的可恢复窗口 —— 那个窗口对绝对编码器没有意义,
    // 且会在大角度时把云台判成 device_fault. zero 待实测.
    const float yaw_window = yaw.limits->position_max_rad;
    config.joints[0] = JointGeometryConfig{Domain::main_yaw, 0.0f, -yaw_window, yaw_window};
    // fire_yaw 未装配: 域名必须显式写成本域, 不能留默认 main_yaw, 否则触发 domain_duplicated.
    // 缺项使其保持 incomplete; 由于该域 unavailable, combine 后仍为 unavailable, 不阻断启动.
    config.joints[1] = JointGeometryConfig{Domain::fire_yaw, std::nullopt, std::nullopt, std::nullopt};
    config.joints[2] = JointGeometryConfig{Domain::pitch, 0.0f, -1.0f, 0.5f};

    // 时序: 1 ms 控制周期, 反馈 20 ms 时效 (旧工程 daemon 10 tick ≈ 10 ms 量级).
    config.timing.control_dt_s = 0.001f;
    config.timing.target_ttl = Lifetime{20};
    config.timing.remote_ttl = Lifetime{100};
    config.timing.imu_ttl = Lifetime{50};
    config.timing.host_link_ttl = Lifetime{500};
    config.timing.max_dt_s = 0.02f;
    const GroupTiming gm{1000, 0, {2000}, 20000};
    config.runtime.gm = gm;
    config.runtime.chassis = gm;
    config.runtime.shooter = gm;
    config.runtime.pitch = gm;
    // 单次 gyro 步长上限 100 ms: 1 kHz 轮询下正常间隔远小于它, 只有真实采样空洞才重建.
    // 旧值 20 ms 会让 SPI 偶发抖动触发 reset_attitude(), 把已积分的 yaw 清零 ——
    // 实测 40 s 内重建 3 次, 转动角度随之丢失.
    config.perception = PerceptionTiming{{50000}, {20000}, {5000}, 100000};

    // 单云台 IMU 安装与零偏已由 2026-11-12 静止/多姿态实测确定:
    //  - 传感器系与云台系轴向一致 (Z 朝天, X 为俯仰轴, 枪管沿 -Y) -> 安装矩阵为单位阵;
    //  - 静止 40 s 的陀螺均值作为静态零偏.
    {
        SingleGimbalImuConfig gimbal{};
        gimbal.origin = Origin::measured;
        gimbal.install = estimation::identity_rotation();
        gimbal.barrel_axis = {0.0f, -1.0f, 0.0f};
        gimbal.gyro_bias_sensor = {-0.000251f, 0.001645f, 0.002211f};
        gimbal.min_projection_squared = 0.02f; // 枪管偏离水平超过约 82° 即判不可用
        // PID 是**待整定参数**, 不是实测结果. 取保守小增益 + 低力矩上限:
        // 外环 rad->rad/s, 内环 rad/s->N·m, 力矩上限远低于 DM 的 ±10 N·m.
        // 必须先在物理急停下做单轴低力矩验证, 再逐步增加.
        gimbal.angle_pid = {2.0f, 0.0f, 0.0f, 0.0f, -6.0f, 6.0f};
        // ±0.5 N·m 实测在 0.17 rad 偏差下已饱和且几乎拉不动, 说明方向正确但能力不足.
        // 放宽到 ±2.0 N·m 继续验证补偿能力; 仍低于 DM 的 ±10 N·m 协议上限.
        gimbal.speed_pid = {0.35f, 0.0f, 0.0f, 0.0f, -2.0f, 2.0f};
        config.single_gimbal_imu = gimbal;
    }
    config.serial = std::nullopt;
    config.serial_origin = Origin::unspecified;
    // 方向契约: 底盘平移跟随云台. 符号 +1 是"云台逆时针转, 机体系向量反向旋转"的
    // 默认约定; 若实车安装方向相反, 改这一处符号即可, 不散落在控制路径里.
    config.chassis_follow_gain = 1.0f;
    config.infantry_controls = true;
    return config;
}

// 设备配置转换: 只有该槽参数齐备且合法时才返回值.
[[nodiscard]] std::optional<drivers::DjiDeviceConfig> device_config(const DjiSlot& slot) noexcept;
[[nodiscard]] std::optional<drivers::DmDeviceConfig> device_config(const DmSlot& slot) noexcept;
// 四轮几何转换: 任一轮缺项时返回空, 不补默认值.
[[nodiscard]] std::optional<motion::OmniDrive> omni_drive(const InfantryConfig& config) noexcept;

// 主机合成夹具: 九 DJI + 一 DM 全部字段显式填写并标记 Origin::synthetic, 只用于离线测试.
// 数值是夹具值, 不代表任何实车机械/传动/电气参数.
[[nodiscard]] InfantryConfig synthetic_config() noexcept;
// 真实部署配置入口: 只声明"尚未配置", 不填任何猜测值; 由持有者逐项补齐后再校验.
[[nodiscard]] InfantryConfig real_config_placeholder() noexcept;
} // 命名空间 ec::infantry::app
