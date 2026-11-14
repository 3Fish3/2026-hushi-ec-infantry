// 步兵部署配置契约: 域装配、可启动性、方向参考单一性.
// 主机测试不驱动电机; 轮向/零位/量程仍为待标定项.
#include "infantry_config.hpp"
#include "infantry_app.hpp"
#include "infantry_kinematics.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <limits>

namespace {
using namespace ec::infantry::app;
using ec::drivers::DjiMode;
constexpr float pi = std::numbers::pi_v<float>;

void deployment_is_startable() {
    const auto config = infantry_deployment_config();
    const auto report = validate(config, ConfigAudience::deployment);
    assert(report.startable);
    assert(report.kind == ConfigKind::real_deployment);
    // 未装配域必须保持 unavailable, 不能被当作"已配置但缺项".
    assert(report.domains[domain_index(Domain::fire_yaw)] == ConfigState::unavailable);
    assert(report.domains[domain_index(Domain::friction_left)] == ConfigState::unavailable);
    assert(report.domains[domain_index(Domain::friction_right)] == ConfigState::unavailable);
    assert(report.domains[domain_index(Domain::trigger)] == ConfigState::unavailable);
    // 底盘/Yaw齐备; Pitch 走达妙反馈接入阶段, 明确 incomplete 且不取得执行资格.
    for (auto domain : {Domain::chassis_lf, Domain::chassis_rf, Domain::chassis_lb,
                        Domain::chassis_rb, Domain::main_yaw}) {
        assert(report.domains[domain_index(domain)] == ConfigState::complete);
    }
    assert(report.domains[domain_index(Domain::pitch)] == ConfigState::complete);
    assert(!config.dji[4].installed);
    assert(config.dm[1].installed && config.dm[1].domain == Domain::pitch);
    assert(!config.dm[1].feedback_only);
    // 位置模式: 电机内部 MIT 位置闭环, kp/kd 都必须有值.
    assert(*config.dm[1].mit_kp > 0.0f && *config.dm[1].mit_kd > 0.0f);
    assert(config.dm[1].bus == 1 && config.dm[1].can_id == 0x001);
    assert(config.dm[1].master_id == 0x011);
    assert(config.single_gimbal_imu->angle_pid.kp == 3.0f);
    assert(config.single_gimbal_imu->speed_pid.kp == 0.6f);
    // 单云台 IMU 标定必须存在且来源可追溯, 否则 yaw 不能取得执行资格.
    assert(config.single_gimbal_imu.has_value());
    assert(config.single_gimbal_imu->origin == Origin::measured);
    assert(config.single_gimbal_imu->barrel_axis[1] == -1.0f);
    // 启动补偿默认关闭; 平滑带只是待整定设置, 不预填经验摩擦幅值.
    assert(config.single_gimbal_imu->friction_torque_nm == 0.0f);
    assert(config.single_gimbal_imu->friction_speed_rad_s == 0.05f);
    // report.complete 要求全部 10 个域齐备; 步兵只装 6 个域, 故 complete 必为 false.
    // 可运行判据是 startable, 而不是"哨兵全装"意义上的 complete.
    assert(!report.complete);
    // 已装配域本身不能有缺项 issue 之外的配置错误.
    assert(!report.has(ConfigIssue::domain_duplicated));
    assert(!report.has(ConfigIssue::invalid_bus));
    assert(!report.has(ConfigIssue::invalid_device_id));
}

void hardware_mapping() {
    const auto config = infantry_deployment_config();
    // yaw 走 DM 槽, 收发同 ID; 达妙协议允许.
    assert(config.dm[0].installed);
    assert(config.dm[0].domain == Domain::main_yaw);
    assert(config.dm[0].bus == 1);
    assert(config.dm[0].can_id == 0x005 && config.dm[0].master_id == 0x005);
    // 四轮CAN1 ID1~4; 丝杆Pitch是达妙, 不占DJI槽.
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        assert(config.dji[i].installed);
        assert(config.dji[i].bus == 1);
        assert(config.dji[i].device_id == i + 1);
        assert(config.dji[i].mode == DjiMode::c620_current);
    }
    for (std::size_t i = chassis_wheel_count; i < dji_slot_count; ++i) {
        assert(!config.dji[i].installed);
    }
    // pitch 达妙: 实物在 CAN1, 命令 0x001 / 反馈 0x011, 与同总线 yaw 的 0x005 不冲突.
    const auto pitch_device = device_config(config.dm[1]);
    assert(pitch_device && pitch_device->bus == 1);
    assert(pitch_device->can_id == 0x001 && pitch_device->master_id == 0x011);
}

// 方向契约: 底盘 twist 由 yaw 关节角唯一决定, 不依赖 IMU 或单圈相位.
void single_reference_heading() {
    const auto config = infantry_deployment_config();
    // 云台正前方 (相对角 0): 前进 = 底盘 +X.
    for (float yaw : {0.0f, pi / 2, pi, -pi / 2, 10 * 2 * pi + pi / 6}) {
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        // 与 infantry_direction.hpp 同一约定: body = R(-yaw) * gimbal.
        const float vx = c * 1.0f + s * 0.0f;
        const float vy = s * 1.0f - c * 0.0f;
        // 前进方向应与 (sin(yaw) 前向投影) 一致, 而非独立于 yaw.
        assert(std::fabs(std::hypot(vx, vy) - 1.0f) < 1e-5f);
    }
    // 关节几何必须为 yaw 提供有界窗口, 否则单圈参考无法唯一恢复.
    const auto& joint = config.joints[0];
    assert(joint.domain == Domain::main_yaw);
    assert(joint.min_rad && joint.max_rad && *joint.min_rad < *joint.max_rad);
    // 量程必须绑定到达妙编码器可表示的窗口: 云台有滑环可连续旋转, 若沿用旧工程
    // GM6020 单圈 ±2.6 rad, 大角度时会被判成 device_fault 并撤输出.
    const auto yaw_window = config.dm[0].limits->position_max_rad;
    assert(*joint.max_rad == yaw_window);
    assert(*joint.min_rad == -yaw_window);
}
// Pitch 只由电机位置范围限位: 实测量程钳住两端, 越界整包拒绝, 且不再依赖 IMU.
void pitch_position_limits() {
    const auto config = infantry_deployment_config();
    InfantryApp app{config, ConfigAudience::deployment};
    AppInputs in{};
    ec::Tick now{1000};
    in.link.remote_online = true;
    in.link.switch_left = ec::drivers::Dr16Switch::up;
    in.link.switch_right = ec::drivers::Dr16Switch::up;
    in.link.remote_sampled_at = now;
    in.link.locally_ready = true;
    const auto i = domain_index(Domain::pitch);
    const auto& joint = config.joints[joint_index(Domain::pitch)];
    assert(joint.domain == Domain::pitch);
    assert(joint.min_rad && joint.max_rad && *joint.min_rad < *joint.max_rad);
    // 实测两端 1.805333 / 2.875753 rad, 按用户要求各向外扩跨度 10%.
    assert(std::fabs(*joint.min_rad - 1.698291f) < 1e-5f);
    assert(std::fabs(*joint.max_rad - 2.982795f) < 1e-5f);
    assert(*joint.max_rad < config.dm[1].limits->position_max_rad);
    in.feedback[i].position_rad = {2.3f, now, ec::Validity::valid};
    in.feedback[i].velocity_rad_s = {0.0f, now, ec::Validity::valid};
    in.inertial.validity = DomainValidity::valid;
    in.inertial.roll_pitch_yaw_rad = {{0.0f, 1.0f, 0.0f}, now, ec::Validity::valid};
    app.update(now, in);
    const auto request = [&](float position) {
        TargetSnapshot targets{};
        targets.domains[i] = DomainSetpoint{ec::control::ControlMode::position, position, 0};
        assert(app.accept_targets(targets, now, Source::remote) == AcceptResult::accepted);
        app.update(now, in);
        return app.snapshot().executions[i];
    };
    auto execution = request(2.4f);
    assert(execution.execute && *execution.adopted_target == 2.4f);
    // 行程两端内侧仍然可执行: 限位不是硬停, 不做方向偏置.
    execution = request(*joint.min_rad);
    assert(execution.execute && *execution.adopted_target == *joint.min_rad);
    execution = request(*joint.max_rad);
    assert(execution.execute && *execution.adopted_target == *joint.max_rad);
    // Pitch 不再消费 IMU: 倾角越界、过期、NaN、整体失效都不应阻止电机位置目标.
    for (float tilt : {-1.0f, 0.0f, 1.0f}) {
        in.inertial.roll_pitch_yaw_rad.value[0] = tilt;
        assert(request(2.4f).execute);
    }
    in.inertial.roll_pitch_yaw_rad.sampled_at = ec::Tick{900};
    assert(request(2.4f).execute);
    in.inertial.roll_pitch_yaw_rad.value[0] = std::numeric_limits<float>::quiet_NaN();
    assert(request(2.4f).execute);
    in.inertial.validity = DomainValidity::invalid;
    assert(request(2.4f).execute);
    in.inertial.validity = DomainValidity::valid;
    in.inertial.roll_pitch_yaw_rad = {{0.0f, 0.0f, 0.0f}, now, ec::Validity::valid};
    TargetSnapshot bad{};
    bad.domains[i] = DomainSetpoint{ec::control::ControlMode::velocity, 0.5f, 0};
    assert(app.accept_targets(bad, now, Source::remote) == AcceptResult::invalid_target);
    for (float position : {13.0f, *joint.max_rad + 0.05f, *joint.min_rad - 0.05f}) {
        bad.domains[i] = DomainSetpoint{ec::control::ControlMode::position, position, 0};
        assert(app.accept_targets(bad, now, Source::remote) == AcceptResult::limit_violation);
    }
    in.link.switch_right = ec::drivers::Dr16Switch::down;
    ++now.ms;
    in.link.remote_sampled_at = now;
    app.update(now, in);
    assert(app.snapshot().permission.manual_stop && !app.snapshot().executions[i].execute);
    assert(mode_allowed_for(Domain::pitch, ec::control::ControlMode::position, true));
    assert(!mode_allowed_for(Domain::pitch, ec::control::ControlMode::velocity, true));
}
} // namespace

int main() {
    pitch_position_limits();
    deployment_is_startable();
    hardware_mapping();
    single_reference_heading();
    std::puts("PASS: infantry deployment startable, hardware mapping, single-reference heading");
}
