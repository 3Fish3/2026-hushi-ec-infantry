// 步兵方向契约: 唯一云台, 任何情况下云台前方 = 前进方向.
// 主机测试只验证"云台系 → 机体系"的装配与参考源选择, 不驱动真实电机;
// 实车方向符号/零位仍为标定项.
#include "infantry_cycle.hpp"
#include "infantry_config.hpp"
#include "infantry_param_apply.hpp"
#include "sim_plant.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <utility>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::drivers::Dr16Stream;
using ec::drivers::Dr16Switch;
using ec::infantry::app::ConfigAudience;
using ec::infantry::app::CycleInputs;
using ec::infantry::app::Domain;
using ec::infantry::app::InfantryApp;
using ec::infantry::app::InfantryCycle;
using ec::infantry::app::InfantryConfig;
using ec::infantry::app::InfantryExecutor;
using ec::infantry::app::InfantryPerception;
using ec::infantry::app::YawFrame;
using ec::infantry::host::SimPlant;
using ec::infantry::host::SoftwareBus;

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::uint32_t index(Domain domain) { return ec::infantry::app::domain_index(domain); }

struct FrameSpec {
    std::array<int, 4> ch{{0, 0, 0, 0}};
    int ch4{};
    Dr16Switch switch_left{Dr16Switch::mid};
    Dr16Switch switch_right{Dr16Switch::mid};
};

// 与 C 板示例一致: ch0–ch3 为 11 bit 位域减 1024, byte5 高位是拨杆, byte16..17 是第 5 通道.
std::array<std::uint8_t, 18> encode_frame(const FrameSpec& spec) {
    std::array<std::uint8_t, 18> buf{};
    std::uint64_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        const auto raw = static_cast<std::uint64_t>(spec.ch[static_cast<std::size_t>(i)] + 1024) & 0x7ffu;
        bits |= raw << (11 * i);
    }
    for (int i = 0; i < 6; ++i) {
        buf[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xffu);
    }
    buf[5] = static_cast<std::uint8_t>((buf[5] & 0x0fu) |
        ((static_cast<unsigned>(spec.switch_left) & 0x03u) << 6) |
        ((static_cast<unsigned>(spec.switch_right) & 0x03u) << 4));
    const auto raw4 = static_cast<std::uint16_t>(spec.ch4 + 1024);
    buf[16] = static_cast<std::uint8_t>(raw4 & 0xffu);
    buf[17] = static_cast<std::uint8_t>(raw4 >> 8);
    return buf;
}

int stick(float normalized) { return static_cast<int>(normalized * 660.0f); }

ec::infantry::app::RemoteMappingConfig mapping() {
    ec::infantry::app::RemoteMappingConfig config{};
    config.stick_deadzone = 0.05f;
    config.chassis_vx_max_mps = 1.0f;
    config.chassis_vy_max_mps = 1.0f;
    config.chassis_wz_max_rad_s = 2.0f;
    config.yaw_rate_max_rad_s = 1.0f;
    config.pitch_rate_max_rad_s = 1.0f;
    config.ch4 = ec::infantry::app::Ch4LayerConfig{0, 300, -300, 50, true};
    return config;
}

// 合成 PID 与 IMU 安装仅用于主机测试, 不是实车参数.
InfantryConfig test_infantry_config() {
    auto config = ec::infantry::app::infantry_deployment_config();
    ec::infantry::app::SingleGimbalImuConfig imu{};
    imu.origin = ec::infantry::app::Origin::synthetic;
    imu.min_projection_squared = 0.01f;
    imu.angle_pid = {4.0f, 0.0f, 0.0f, 0.0f, -10.0f, 10.0f};
    imu.speed_pid = {0.5f, 0.0f, 0.0f, 0.0f, -4.0f, 4.0f};
    config.single_gimbal_imu = imu;
    return config;
}

// 步兵编排测试台: 步兵装配 + 合成标定 + 软件总线 + 仿真负载.
struct Rig {
    InfantryConfig config{test_infantry_config()};
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    InfantryApp app{config, ConfigAudience::offline_test};
    InfantryExecutor executor{config, ConfigAudience::offline_test, bus1, bus2};
    InfantryPerception perception{config, ConfigAudience::offline_test};
    InfantryCycle cycle{app, executor, perception, mapping(), config.timing.remote_ttl};
    SimPlant plant{};
    Dr16Stream stream{};
    Tick now{1000};
    float dt_s{0.001f};
    bool advance_plant{false};
    bool imu_enabled{true};
    float world_gyro_z{};
    std::optional<ec::drivers::Dr16Sample> pending{};

    explicit Rig(InfantryConfig value = test_infantry_config())
        : config(std::move(value)), app(config, ConfigAudience::offline_test),
          executor(config, ConfigAudience::offline_test, bus1, bus2),
          perception(config, ConfigAudience::offline_test),
          cycle(app, executor, perception, mapping(), config.timing.remote_ttl) {
        const std::uint8_t priming = 0;
        stream.push({&priming, 1});
        static_cast<void>(stream.on_idle());
    }

    void feed(const FrameSpec& spec) {
        const auto frame = encode_frame(spec);
        stream.push({frame.data(), frame.size()});
        const auto sample = stream.on_idle();
        if (sample) { pending = *sample; }
    }

    void cycle_once() {
        CycleInputs in{};
        in.now = now;
        in.dt_s = dt_s;
        if (pending) {
            in.remote_sample = *pending;
            in.remote_received_at = now;
            pending.reset();
        }
        in.locally_ready = true;
        if (imu_enabled) {
            in.imu_gyro_rad_s = std::array<float, 3>{0.0f, 0.0f, world_gyro_z};
            in.imu_accel_mps2 = std::array<float, 3>{0.0f, 0.0f, 9.80665f};
            in.imu_sampled_at = now;
        }
        plant.emit_feedback(config,
            [this](std::uint8_t bus, const ec::transport::CanFrame& frame, Tick at) {
                executor.route(bus, frame, at, ec::MicroTick{at.ms * 1000u});
            },
            now);
        bus1.clear();
        bus2.clear();
        static_cast<void>(cycle.control(in));
        cycle.commit(in);
        if (advance_plant) { plant.integrate(config, bus1, bus2, dt_s); }
        ++now.ms;
    }
};

// 云台前方 = 前进方向: psi 是云台相对机体的关节角, 平移意图在云台系表达.
// 期望机体系量 = R(+psi) * (v, 0).
void forward_matches_gimbal(float psi, float expect_vx, float expect_vy) {
    Rig rig;
    rig.plant.dm.position_rad = psi; // 绝对编码器读数 (zero_offset = 0)
    // 先给一次回中帧建立本地方向通道基准.
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    // 左杆前推: axis[3] 是左摇杆 Y (前后).
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    const auto adopted = rig.app.adopted_twist();
    CHECK(adopted.has_value());
    if (!adopted) { return; }
    // 摇杆前推量 = 0.5 * chassis_vx_max (1.0 m/s).
    CHECK(std::fabs(adopted->vx_mps - expect_vx) < 1e-3f);
    CHECK(std::fabs(adopted->vy_mps - expect_vy) < 1e-3f);
}

void chassis_follows_gimbal() {
    // psi = 0: 前推即机体系 +X.
    forward_matches_gimbal(0.0f, 0.5f, 0.0f);
    // psi = +90° (枪管指向机体左方): 机体系应为 (0, +0.5).
    forward_matches_gimbal(pi * 0.5f, 0.0f, 0.5f);
    // psi = 180° (枪管朝后): 机体系 (-0.5, 0).
    forward_matches_gimbal(pi, -0.5f, 0.0f);
    // psi = -90°: 机体系 (0, -0.5).
    forward_matches_gimbal(-pi * 0.5f, 0.0f, -0.5f);
    // 同一物理方向相隔整圈时, 平移方向相同.
    forward_matches_gimbal(2.0f * pi + pi * 0.5f, 0.0f, 0.5f);
}

// 云台角无效时禁止平移: 方向契约不成立就宁可不动, 不按错的参考走.
void no_translation_without_gimbal_angle() {
    Rig rig;
    rig.plant.dm.emitting = false; // DM 掉线 → 关节角无效
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    CHECK(!rig.app.adopted_twist().has_value());
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    CHECK(!rig.app.target(Domain::chassis_rf).has_value());
    CHECK(!rig.app.target(Domain::chassis_lb).has_value());
    CHECK(!rig.app.target(Domain::chassis_rb).has_value());
}

// 运行中掉线也不得继续使用最后一次有效的云台角.
void no_translation_after_gimbal_timeout() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    CHECK(rig.app.adopted_twist().has_value());
    rig.plant.dm.emitting = false;
    rig.now.ms += rig.config.dm[0].feedback_ttl.ms + 1;
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    CHECK(!rig.cycle.outcome().chassis_active);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
}

// 云台目标是 IMU 世界航向, 不应以 0.3 rad 的相对关节角建立世界目标.
void gimbal_is_drivable() {
    Rig rig;
    rig.plant.dm.position_rad = 0.3f;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    // 右摇杆 X 在本地方向层 → main_yaw 角度目标.
    rig.feed(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    const auto target = rig.app.target(Domain::main_yaw);
    CHECK(target.has_value());
    if (!target) { return; }
    CHECK(target->frame == YawFrame::local_inertial);
    // 首次接管必须取自当前有效世界航向, 而不是关节角 0.3 rad 之类的相对量.
    const auto direction = rig.perception.local_direction_rad();
    CHECK(direction.has_value());
    if (direction) { CHECK(std::fabs(target->value - *direction) < 0.05f); }
    // 机械驱动的 main yaw 不应被记为惯性降级.
    CHECK(!rig.app.main_yaw_degraded());
    // 继续推 → 目标角继续增长 (证明没有被冻在降级保持里).
    const float first = target->value;
    rig.feed(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    const auto later = rig.app.target(Domain::main_yaw);
    CHECK(later.has_value());
    if (later) { CHECK(later->value > first); }
    // 执行层接受该目标 (execute 为真), 不是每周期被撤输出.
    const auto& execution = rig.app.snapshot().executions[index(Domain::main_yaw)];
    CHECK(execution.execute);
    CHECK(execution.frame == YawFrame::local_inertial);
}
// 拨杆映射必须贯穿操纵映射和权限仲裁, 不能只在遥控模块里修改.
void infantry_switches_reach_authority() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    CHECK(rig.app.target(Domain::chassis_lf));
    const auto reference_epoch = rig.app.reference_epoch();
    // 中挡拨动原来用于校准/发射的通道, 不得更改参考或产生发射目标.
    rig.feed(FrameSpec{{0, stick(0.5f), stick(0.5f), stick(0.5f)}, -400,
        Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(rig.app.reference_epoch() == reference_epoch);
    CHECK(rig.app.snapshot().permission.manual_stop);
    CHECK(!rig.app.target(Domain::chassis_lf));
    CHECK(!rig.app.target(Domain::main_yaw));
    for (const auto domain : {Domain::friction_left, Domain::friction_right, Domain::trigger}) {
        CHECK(!rig.app.target(domain));
    }
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::down});
    rig.cycle_once();
    rig.feed(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::down});
    rig.cycle_once();
    CHECK(!rig.app.snapshot().permission.manual_stop);
    CHECK(rig.app.adopted_twist() && rig.app.adopted_twist()->wz_rad_s > 0.0f);
    CHECK(rig.app.target(Domain::main_yaw)); // 小陀螺时右杆依旧控制云台.
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::up});
    rig.cycle_once();
    rig.cycle_once();
    CHECK(rig.app.snapshot().permission.authority); // 左中键鼠不能被仲裁当成待机.
    CHECK(!rig.app.snapshot().permission.manual_stop);
}

// 滑环连续性: 达妙反馈是 25 rad 有符号窗口, 越过 ±12.5 rad 时原始值回绕.
// 展开后关节角必须单调连续, 否则底盘跟随方向会在过窗口时突然翻转.
void dm_angle_unwraps_across_window() {
    Rig rig;
    const float edge = rig.config.dm[0].limits->position_max_rad;
    auto read = [&](float raw) -> float {
        rig.plant.dm.position_rad = raw;
        rig.cycle_once();
        return rig.executor.joint_feedback(Domain::main_yaw, Tick{rig.now.ms - 1}).position_rad.value;
    };
    // 从窗口内接近上限, 再跨到窗口下限 (raw 回绕).
    const float before = read(edge - 0.2f);
    const float after = read(-edge + 0.2f);
    // 真实位移是 +0.4 rad, 不是 -(window - 0.4).
    CHECK(std::fabs((after - before) - 0.4f) < 0.02f);
    // 反向跨绕同样连续: 从 -edge+0.2 倒转回 +edge-0.2, 真实位移是 -0.4 rad.
    const float back = read(edge - 0.2f);
    CHECK(std::fabs((back - after) + 0.4f) < 0.02f);
}

// 缺标定必须不驱动: 显式移除 IMU 标定, 验证 yaw 域被判 incomplete 且不发 MIT 命令.
void missing_calibration_does_not_drive_yaw() {
    auto config = ec::infantry::app::infantry_deployment_config();
    config.single_gimbal_imu.reset();
    Rig rig{config};
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
        rig.cycle_once();
    }
    CHECK(rig.app.report().domains[index(Domain::main_yaw)] == ec::infantry::app::ConfigState::incomplete);
    CHECK(!rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    CHECK(rig.executor.stats().dm_command_sent == 0);
}

// 枪管沿机体 −Y, 绕机体 Z 轴转动时水平航向变化率应等于该角速度;
// 绕机体 X 轴 (俯仰轴) 转动时水平航向不变 —— 这两条是安装几何的直接推论.
void tilted_heading_rate() {
    auto config = test_infantry_config();
    config.single_gimbal_imu->min_projection_squared = 0.02f;
    constexpr float g = 9.80665f;
    constexpr float tilt = 0.35f; // 轻微俯仰, 枪管仍接近水平
    const std::array<float, 3> accel{0.0f, g*std::sin(tilt), g*std::cos(tilt)};

    // 绕 Z 轴 1 rad/s: 航向变化率应约为 1.
    {
        InfantryPerception perception{config, ConfigAudience::offline_test};
        perception.submit_imu({0.0f, 0.0f, 0.0f}, accel, Tick{1000}, Tick{1000});
        perception.submit_imu({0.0f, 0.0f, 1.0f}, accel, Tick{1001}, Tick{1001});
        perception.update(Tick{1001});
        CHECK(perception.platform().validity == ec::infantry::app::DomainValidity::valid);
        // 仅两帧样本时 EKF 仍在收敛, 实测约 1.06; 断言量级与符号而非精确值.
        CHECK(std::fabs(perception.platform().value.heading_rate - 1.0f) < 0.15f);
    }
    // 绕 X 轴 (俯仰) 1 rad/s: 水平航向不应变化.
    {
        InfantryPerception perception{config, ConfigAudience::offline_test};
        perception.submit_imu({0.0f, 0.0f, 0.0f}, accel, Tick{1000}, Tick{1000});
        perception.submit_imu({1.0f, 0.0f, 0.0f}, accel, Tick{1001}, Tick{1001});
        perception.update(Tick{1001});
        CHECK(perception.platform().validity == ec::infantry::app::DomainValidity::valid);
        CHECK(std::fabs(perception.platform().value.heading_rate) < 0.05f);
    }
}

// 独立简化动力学: 世界角速度 = 底盘角速度 + 云台相对角速度.
// 反馈由模型产生, 不用 PID 的目标值冒充测量; 所有惯量/阻尼/PID 均为合成值.
void world_heading_closed_loop() {
    Rig rig;
    float base_angle = 0.0f, world_angle = 0.0f, relative_rate = 0.0f;
    constexpr float base_rate = 0.5f, inertia = 0.05f, damping = 0.05f;
    for (int i = 0; i < 64000; ++i) {
        // 本用例显式假设反馈以 25 rad 回绕; 不代表设备固件行为已获实测证明.
        rig.plant.dm.position_rad = std::remainder(world_angle - base_angle, 25.0f);
        rig.plant.dm.velocity_rad_s = relative_rate;
        rig.world_gyro_z = base_rate + relative_rate;
        if (i % 10 == 0) {
            rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::down});
        }
        rig.cycle_once();
        float torque = 0.0f;
        if (const auto frame = rig.bus1.last_with_id(0x005)) {
            // 使能帧不包含有效 MIT 控制量.
            bool action_frame = true;
            for (std::size_t byte = 0; byte < 7; ++byte) {
                action_frame = action_frame && frame->data[byte] == 0xff;
            }
            action_frame = action_frame && frame->data[7] >= 0xfc;
            if (!action_frame) {
                const auto command = ec::infantry::host::decode_dm_mit_command(*frame, *rig.config.dm[0].limits);
                CHECK(command);
                if (command) {
                    CHECK(command->kp == 0.0f && command->kd == 0.0f);
                    CHECK(std::fabs(command->torque_nm) <= 4.01f);
                    const auto& telemetry = rig.executor.main_yaw_stats();
                    CHECK(telemetry.torque_submitted);
                    CHECK(telemetry.pid_trace_valid);
                    CHECK(std::isfinite(telemetry.speed_target_rad_s));
                    CHECK(std::isfinite(telemetry.speed_pid_p_nm) && std::isfinite(telemetry.speed_pid_i_nm));
                    CHECK(std::fabs(telemetry.torque_command_nm - command->torque_nm) < 0.006f);
                    torque = command->torque_nm;
                }
            }
        }
        relative_rate += (torque - damping*relative_rate) / inertia * rig.dt_s;
        base_angle += base_rate * rig.dt_s;
        world_angle += (base_rate + relative_rate) * rig.dt_s;
        if (i > 2000) { CHECK(std::fabs(world_angle) < 0.1f); }
    }
    CHECK(base_angle > 31.0f);
    CHECK(rig.executor.stats().dm_command_sent > 60000);
    CHECK(!rig.app.main_yaw_degraded());

    rig.imu_enabled = false;
    for (int i = 0; i < 30; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::down});
        rig.cycle_once();
    }
    CHECK(!rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    CHECK(rig.executor.stats().dm_disable_sent > 0);
    CHECK(!rig.executor.main_yaw_stats().torque_submitted);
    CHECK(rig.executor.main_yaw_stats().torque_command_nm == 0.0f);
    CHECK(!rig.executor.main_yaw_stats().pid_trace_valid);
    CHECK(rig.executor.main_yaw_stats().speed_target_rad_s == 0.0f);
    CHECK(rig.executor.main_yaw_stats().speed_pid_p_nm == 0.0f);
    CHECK(rig.executor.main_yaw_stats().speed_pid_i_nm == 0.0f);
    rig.imu_enabled = true;
    rig.world_gyro_z = 0.0f;
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::down});
        rig.cycle_once();
    }
    CHECK(rig.app.target(Domain::main_yaw));
    CHECK(std::fabs(rig.app.target(Domain::main_yaw)->value -
        rig.perception.platform().continuous_heading) < 0.001f);
}

// 摩擦启动补偿必须走真实参数事务; 缺参数或写入未生效都在此失败.
void friction_parameters_apply() {
    using namespace ec::infantry::app;
    Rig rig;
    ParamStore params{rig.config};
    const auto amplitude = param_id(domain_count, 19);
    const auto band = param_id(domain_count, 20);
    CHECK(params.find(amplitude));
    CHECK(params.find(band));
    CHECK(params.entry(amplitude)->value == 0.0f);
    CHECK(param_has_apply_path(rig.app, amplitude));
    CHECK(param_has_apply_path(rig.app, band));
    const auto write = [&](std::uint16_t id, float value) {
        return apply_param(params, rig.app, &rig.executor, &rig.perception, &rig.cycle,
            id, value, true);
    };
    CHECK(write(amplitude, 0.15f) == ParamWriteResult::accepted);
    CHECK(write(band, 0.1f) == ParamWriteResult::accepted);
    CHECK(params.entry(amplitude)->value == 0.15f);
    CHECK(params.entry(band)->value == 0.1f);
    CHECK(write(amplitude, -0.1f) == ParamWriteResult::invalid_value);
    CHECK(write(band, 0.0f) == ParamWriteResult::invalid_value);
    CHECK(params.entry(amplitude)->value == 0.15f);
    CHECK(params.entry(band)->value == 0.1f);
    CHECK(rig.executor.gimbal_friction_torque_nm() == 0.15f);
    CHECK(rig.executor.gimbal_friction_band_nm() == 0.1f);
    CHECK(write(amplitude, std::numeric_limits<float>::quiet_NaN()) == ParamWriteResult::invalid_value);
    CHECK(write(amplitude, params.find(amplitude)->max_value + 1.0f) == ParamWriteResult::invalid_value);
    CHECK(!rig.executor.set_gimbal_friction(0.2f, 0.0f));
    CHECK(rig.executor.gimbal_friction_torque_nm() == 0.15f);
    CHECK(rig.executor.gimbal_friction_band_nm() == 0.1f);
    CHECK(apply_param(params, rig.app, nullptr, &rig.perception, &rig.cycle,
        amplitude, 0.2f, true) == ParamWriteResult::unsupported);
    CHECK(params.entry(amplitude)->value == 0.15f);
    CHECK(write(amplitude, 0.0f) == ParamWriteResult::accepted);
    CHECK(rig.executor.gimbal_friction_torque_nm() == 0.0f);
}

void invalid_friction_config_is_rejected() {
    using namespace ec::infantry::app;
    const auto config = test_infantry_config();
    auto gimbal = *config.single_gimbal_imu;
    const auto limits = *config.dm[0].limits;
    CHECK(valid_single_gimbal_imu(gimbal, limits));
    for (float invalid : {-0.1f, limits.torque_max_nm + 1.0f,
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        auto bad = gimbal;
        bad.friction_torque_nm = invalid;
        CHECK(!valid_single_gimbal_imu(bad, limits));
        bad = gimbal;
        bad.friction_band_nm = invalid;
        CHECK(!valid_single_gimbal_imu(bad, limits));
    }
    gimbal.friction_band_nm = 0.0f;
    CHECK(!valid_single_gimbal_imu(gimbal, limits));
}

// 两个不推进负载的真实执行链接受相同输入, 唯一差别为在线补偿参数.
// 逐周期核对最终 MIT 帧、正负号、过零平滑、原限幅和关闭补偿后的输出一致性.
void friction_assist_preserves_torque_contract() {
    using namespace ec::infantry::app;
    for (int direction : {-1, 1}) {
        for (bool limited : {false, true}) {
            auto config = test_infantry_config();
            auto& gimbal = *config.single_gimbal_imu;
            gimbal.torque_direction = direction;
            if (limited) {
                gimbal.speed_pid.output_min = -0.25f;
                gimbal.speed_pid.output_max = 0.3f;
            }
            Rig baseline{config}, assisted{config};
            ParamStore params{config};
            const auto write = [&](float value) {
                CHECK(apply_param(params, assisted.app, &assisted.executor, &assisted.perception,
                    &assisted.cycle, param_id(domain_count, 19), value, false) == ParamWriteResult::accepted);
            };
            write(0.15f);
            bool saw_positive = false, saw_negative = false, saw_zero = false;
            bool saw_smooth = false, saw_saturation = false, saw_disabled = false;
            for (int i = 0; i < 2200; ++i) {
                if (i == 10) {
                    baseline.cycle.set_sweep({true, 0.2f, 1.0f});
                    assisted.cycle.set_sweep({true, 0.2f, 1.0f});
                }
                if (i == 2000) { write(0.0f); }
                for (auto* rig : {&baseline, &assisted}) {
                    rig->feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
                    rig->cycle_once();
                }
                const auto& plain = baseline.executor.main_yaw_stats();
                const auto& actual = assisted.executor.main_yaw_stats();
                CHECK(plain.torque_submitted == actual.torque_submitted);
                if (!actual.torque_submitted) { continue; }
                const float demand = plain.torque_command_nm * static_cast<float>(direction);
                const float magnitude = i < 2000 ? 0.15f : 0.0f;
                const float addition = magnitude * std::clamp(demand / 0.05f, -1.0f, 1.0f);
                const float expected = std::clamp(demand + addition,
                    gimbal.speed_pid.output_min, gimbal.speed_pid.output_max) * static_cast<float>(direction);
                CHECK(std::fabs(actual.torque_command_nm - expected) < 1e-6f);
                CHECK(std::fabs(actual.torque_command_nm) <=
                    std::max(-gimbal.speed_pid.output_min, gimbal.speed_pid.output_max));
                const auto frame = assisted.bus1.last_with_id(0x005);
                CHECK(frame);
                const auto mit = ec::infantry::host::decode_dm_mit_command(*frame, *config.dm[0].limits);
                CHECK(mit);
                CHECK(mit->kp == 0.0f && mit->kd == 0.0f);
                CHECK(std::fabs(mit->torque_nm - expected) < 0.006f);
                saw_positive |= demand > 0.05f;
                saw_negative |= demand < -0.05f;
                saw_zero |= demand == 0.0f && actual.torque_command_nm == 0.0f;
                saw_smooth |= magnitude > 0.0f && std::fabs(demand) > 0.001f && std::fabs(demand) < 0.04f;
                saw_saturation |= demand + addition > gimbal.speed_pid.output_max ||
                    demand + addition < gimbal.speed_pid.output_min;
                if (i >= 2000) {
                    CHECK(actual.torque_command_nm == plain.torque_command_nm);
                    saw_disabled = true;
                }
            }
            CHECK(saw_positive && saw_negative && saw_zero && saw_smooth && saw_disabled);
            CHECK(saw_saturation == limited);
        }
    }
}

// 单变量复现环: 所有真实输入连续有效时, 热写参数不得撤销 yaw 目标/执行资格.
// 走协议处理器所调用的同一 apply_param 事务, 不在真机上重复激励.
void parameter_writes_do_not_interrupt_sweep() {
    using namespace ec::infantry::app;
    Rig rig;
    ParamStore params{rig.config};
    rig.cycle.set_sweep({true, 0.2f, 0.5f});
    for (int i = 0; i < 10; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
        rig.cycle_once();
    }
    const auto epoch = rig.app.reference_epoch();
    const auto generation = rig.app.snapshot().permission.generation;
    const std::array<std::pair<std::uint16_t, float>, 7> writes{{
        {18, 0.0f}, {13, 1.2f}, {16, 0.2f}, {17, 0.5f}, {19, 0.15f}, {20, 0.1f}, {18, 1.0f}}};
    for (const auto& [local, value] : writes) {
        CHECK(apply_param(params, rig.app, &rig.executor, &rig.perception, &rig.cycle,
            param_id(domain_count, local), value, false) == ParamWriteResult::accepted);
        for (int i = 0; i < 100; ++i) {
            rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
            rig.cycle_once();
            const auto& execution = rig.app.snapshot().executions[index(Domain::main_yaw)];
            CHECK(execution.execute);
            CHECK(execution.reason == Reason::none);
            CHECK(execution.adopted_target.has_value());
            CHECK(rig.app.reference_epoch() == epoch);
            CHECK(rig.app.snapshot().permission.generation == generation);
        }
    }
    std::puts("PASS: hot writes preserve yaw execution with continuous valid inputs");
}

// 无任何参数写入, 仅断遥控, 就能复现执行中断后正弦基准被重新捕获.
// 已经开始的试验一旦被停止, 不允许恢复运行挡就自动重新激励.
void interrupted_sweep_requires_explicit_restart() {
    using namespace ec::infantry::app;
    Rig rig;
    rig.cycle.set_sweep({true, 0.2f, 0.5f});
    for (int i = 0; i < 10; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
        rig.cycle_once();
    }
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    for (int i = 0; i < 101; ++i) { rig.cycle_once(); }
    CHECK(!rig.cycle.outcome().remote_online);
    CHECK(!rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    // 失联时试验要退出, 而不是悄悄重新抓一个基准并在重连后自动继续.
    CHECK(!rig.cycle.sweep().enabled);
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
        rig.cycle_once();
    }
    CHECK(!rig.cycle.sweep().enabled); // 运行挡恢复不自动重启正弦.
    rig.cycle.set_sweep({true, 0.2f, 0.5f});
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
    rig.cycle_once();
    CHECK(rig.cycle.sweep().enabled); // 只有明确重新配置才重启.
}

void invalid_reference_cancels_sweep() {
    using namespace ec::infantry::app;
    for (int cause = 0; cause < 4; ++cause) {
        Rig rig;
        CHECK(rig.executor.set_gimbal_friction(0.15f, 0.05f));
        rig.cycle.set_sweep({true, 0.2f, 0.5f});
        for (int i = 0; i < 10; ++i) {
            rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::up});
            rig.cycle_once();
        }
        if (cause == 1) { rig.imu_enabled = false; }
        if (cause == 2) { rig.app.invalidate_domain_reference(Domain::main_yaw); }
        if (cause == 3) { rig.plant.dm.emitting = false; }
        for (int i = 0; i < 30; ++i) {
            rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up,
                cause == 0 ? Dr16Switch::mid : Dr16Switch::up});
            rig.cycle_once();
        }
        CHECK(!rig.cycle.sweep().enabled);
        if (cause != 2) { // 参考重建后允许普通控制重新捕获, 其它三种故障仍在持续.
            CHECK(!rig.executor.main_yaw_stats().torque_submitted);
            CHECK(rig.executor.main_yaw_stats().torque_command_nm == 0.0f);
        }
    }
}

// SYNTHETIC 实际 Cycle/App/Executor/CAN 路径, 不以孤立 Engine 通过冒充接入完成.
void m2_requires_physical_maintenance() {
    using namespace ec::infantry::app;
    Rig rig;
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::down, Dr16Switch::up}); rig.cycle_once();
    }
    rig.app.set_maintenance_session(true, true, 1);
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::down, Dr16Switch::up}); rig.cycle_once();
    }
    CHECK(rig.app.snapshot().permission.source != Source::maintenance);
    for (int i = 0; i < 5; ++i) {
        rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::down, Dr16Switch::down}); rig.cycle_once();
    }
    CHECK(rig.app.snapshot().permission.source == Source::maintenance);
    CHECK(!rig.app.adopted_twist());
    // 遥控掉线不能只靠先前锁存的维护挡继续持权.
    for (int i = 0; i < 110; ++i) { rig.cycle_once(); }
    CHECK(rig.app.snapshot().permission.source != Source::maintenance);
}

void m2_real_execution_stops_and_rearms() {
    using namespace ec::infantry::app;
    for (int direction : {-1, 1}) {
        auto config = test_infantry_config(); config.single_gimbal_imu->torque_direction = direction;
        Rig rig{config}; rig.plant.dm_reports_enabled = false;
        rig.app.set_maintenance_session(true, true, 1);
        const auto step = [&] {
            rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::down, Dr16Switch::down});
            if (rig.app.authority().source() == Source::maintenance && rig.perception.local_direction_rad()) {
                TargetSnapshot target{};
                target.domains[index(Domain::main_yaw)] = DomainSetpoint{ec::control::ControlMode::position,
                    *rig.perception.local_direction_rad(), 0, YawFrame::local_inertial};
                CHECK(rig.app.accept_targets(target, rig.now, Source::maintenance) == AcceptResult::accepted);
            }
            rig.cycle_once();
            for (const auto& frame : rig.bus1.frames) {
                if (frame.id != 5 || frame.size != 8) { continue; }
                bool action = true;
                for (unsigned b = 0; b < 7; ++b) { action = action && frame.data[b] == 0xff; }
                if (action && frame.data[7] == 0xfc) { rig.plant.dm_reports_enabled = true; }
                if (action && frame.data[7] == 0xfd) { rig.plant.dm_reports_enabled = false; }
            }
        };
        for (int i = 0; i < 10; ++i) { step(); }
        sweep::Plan plan{}; plan.count = 1; plan.hz[0] = 5.0f; plan.cycles = 3; plan.amplitude_nm = 0.01f;
        CHECK(rig.executor.start_sweep(plan, rig.now, rig.app));
        bool injected = false;
        for (int i = 0; i < 650; ++i) {
            step();
            const auto& state = rig.executor.sweep_status();
            if (std::fabs(state.requested_nm) > 0.001f && rig.executor.main_yaw_stats().torque_submitted) {
                injected = true;
                CHECK(std::fabs(state.requested_nm - state.applied_nm) < 1e-6f);
                CHECK(std::fabs(rig.executor.main_yaw_stats().torque_command_nm - state.applied_nm) < 1e-5f);
            }
        }
        CHECK(injected);
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::complete);
        CHECK(!rig.plant.dm_reports_enabled);
        CHECK(!rig.executor.main_yaw_stats().torque_submitted);
        // 新显式 START 可从新鲜 Disabled 反馈进入武装, 不能依赖禁用确认前的竞态.
        CHECK(rig.executor.start_sweep(plan, rig.now, rig.app));
        for (int i = 0; i < 100; ++i) { step(); }
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::running);
        rig.executor.cancel_sweep(); step();
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::aborted);
        CHECK(!rig.plant.dm_reports_enabled);
        CHECK(!rig.executor.main_yaw_stats().torque_submitted);
        step(); // 新鲜反馈确认上一周期 disable 已落实
        const float old_target = rig.app.target(Domain::main_yaw)->value;
        const float old_speed_kp = rig.executor.gimbal_speed_pid_config().kp;
        rig.app.set_maintenance_session(false, false, 1);
        // 直接切普通运行挡不能绕过锁存; 必须先经历物理停止挡.
        for (int i = 0; i < 5; ++i) {
            rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up, Dr16Switch::up}); rig.cycle_once();
        }
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::aborted);
        CHECK(!rig.executor.main_yaw_stats().torque_submitted);
        // 没有新鲜电机反馈时, 即使收到停止挡也不能解锁.
        rig.plant.dm.emitting = false;
        for (int i = 0; i < 30; ++i) {
            rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up,
                i < 25 ? Dr16Switch::up : Dr16Switch::mid}); rig.cycle_once();
        }
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::aborted);
        // 发出disable请求不等于已停机; Enabled反馈也不能清锁存.
        rig.plant.dm.emitting = true; rig.plant.dm_reports_enabled = true;
        rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up, Dr16Switch::mid}); rig.cycle_once();
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::aborted);
        // 停止挡锁存不等于新鲜遥控: 掉线后才收到Disabled仍不能解锁.
        for (int i = 0; i < 110; ++i) { rig.cycle_once(); }
        rig.plant.dm_reports_enabled = false; rig.cycle_once();
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::aborted);
        rig.cycle.set_sweep({true, 0.2f, 0.5f}); // 旧试验即使尚未起振也必须撤销
        rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up, Dr16Switch::mid}); rig.cycle_once();
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::idle);
        CHECK(!rig.cycle.sweep().enabled);
        CHECK(!rig.cycle.mapper().held_angle(Domain::main_yaw));
        CHECK(!rig.app.target(Domain::main_yaw));
        CHECK(!rig.executor.main_yaw_stats().torque_submitted);
        // 停止时手动改变姿态, 再进入普通遥控必须抓当前朝向而非复活旧目标.
        rig.world_gyro_z = 0.5f;
        for (int i = 0; i < 100; ++i) {
            rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up, Dr16Switch::mid}); rig.cycle_once();
        }
        rig.world_gyro_z = 0;
        for (int i = 0; i < 5; ++i) {
            rig.feed(FrameSpec{{0,0,0,0}, 0, Dr16Switch::up, Dr16Switch::up}); rig.cycle_once();
            for (const auto& frame : rig.bus1.frames) {
                if (frame.id == 5 && frame.size == 8 && frame.data[0] == 0xff && frame.data[7] == 0xfc) {
                    rig.plant.dm_reports_enabled = true;
                }
            }
        }
        const auto target = rig.app.target(Domain::main_yaw);
        CHECK(target && target->frame == YawFrame::local_inertial);
        CHECK(std::fabs(target->value - *rig.perception.local_direction_rad()) < 0.001f);
        CHECK(std::fabs(target->value - old_target) > 0.02f);
        CHECK(rig.executor.main_yaw_stats().torque_submitted);
        CHECK(rig.executor.gimbal_speed_pid_config().kp == old_speed_kp);
        CHECK(rig.executor.sweep_status().phase == sweep::Phase::idle);
    }
}

} // namespace

int main() {
    m2_requires_physical_maintenance();
    m2_real_execution_stops_and_rearms();
    friction_parameters_apply();
    invalid_friction_config_is_rejected();
    friction_assist_preserves_torque_contract();
    parameter_writes_do_not_interrupt_sweep();
    interrupted_sweep_requires_explicit_restart();
    invalid_reference_cancels_sweep();
    missing_calibration_does_not_drive_yaw();
    tilted_heading_rate();
    world_heading_closed_loop();
    chassis_follows_gimbal();
    no_translation_without_gimbal_angle();
    no_translation_after_gimbal_timeout();
    gimbal_is_drivable();
    infantry_switches_reach_authority();
    dm_angle_unwraps_across_window();
    std::puts("PASS: 前进方向跟随云台, 云台角无效时禁止平移, 云台可由遥控驱动, 25 rad 回绕连续");
}
