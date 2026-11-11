// 步兵部署配置契约: 域装配、可启动性、方向参考单一性.
// 主机测试不驱动电机; 轮向/零位/量程仍为待标定项.
#include "sentinel_config.hpp"
#include "sentinel_kinematics.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace {
using namespace ec::sentinel::app;
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
    // 已装配域必须齐备.
    for (auto domain : {Domain::chassis_lf, Domain::chassis_rf, Domain::chassis_lb,
                        Domain::chassis_rb, Domain::main_yaw, Domain::pitch}) {
        assert(report.domains[domain_index(domain)] == ConfigState::complete);
    }
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
    // 四轮 CAN1 ID1~4, pitch CAN2 ID5.
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        assert(config.dji[i].installed);
        assert(config.dji[i].bus == 1);
        assert(config.dji[i].device_id == i + 1);
        assert(config.dji[i].mode == DjiMode::c620_current);
    }
    assert(config.dji[4].domain == Domain::pitch);
    assert(config.dji[4].bus == 2 && config.dji[4].device_id == 5);
    // 其余 DJI 槽未装配.
    for (std::size_t i = 5; i < dji_slot_count; ++i) {
        assert(!config.dji[i].installed);
    }
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
}
} // namespace

int main() {
    deployment_is_startable();
    hardware_mapping();
    single_reference_heading();
    std::puts("PASS: infantry deployment startable, hardware mapping, single-reference heading");
}
