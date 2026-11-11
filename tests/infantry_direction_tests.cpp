// 主机几何和反馈约定测试，不驱动电机，不代表现场轮向已标定。
#include "infantry_config.hpp"
#include "infantry_direction.hpp"
#include "infantry_yaw_angle.hpp"
#include "ec/drivers/dm_device.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

namespace {
using namespace ec::infantry::app;
constexpr float pi = std::numbers::pi_v<float>;
constexpr float tau = 2.0f * pi;
void near(float a, float b, float eps = 1e-4f) { assert(std::fabs(a - b) < eps); }

void yaw_feedback() {
    ContinuousYawAngle yaw;
    assert(!yaw.valid());
    assert(!yaw.update(15.0f));
    assert(!yaw.update(std::nanf("")));
    assert(yaw.update(2.0f));
    near(*yaw.continuous_rad(), 2.0f);
    near(*yaw.relative_rad(1.0f), 1.0f);
    yaw.reset();
    assert(!yaw.relative_rad(1.0f));
    assert(yaw.update(12.48f));
    const auto before = *yaw.continuous_rad();
    assert(yaw.update(-12.48f));
    near(*yaw.continuous_rad() - before, 0.04f, 1e-5f);
    assert(yaw.wrap_events() == 1);
    for (int direction : {1, -1}) {
        yaw.reset();
        constexpr int steps = 200000;
        constexpr float step = 0.01f;
        for (int i = 0; i <= steps; ++i) {
            const float angle = direction * i * step;
            float packet = std::fmod(angle + 12.5f, 25.0f);
            if (packet < 0.0f) { packet += 25.0f; }
            assert(yaw.update(packet - 12.5f));
            if (i % 1000 == 0) { near(*yaw.continuous_rad(), angle, 0.004f); }
        }
    }
}

void direction_geometry() {
    // 独立物理预期：底盘 +X 为前、+Y 为左，云台逆时针 90° 后枪口朝左。
    const auto f0 = to_chassis_frame({1, 0, 0}, 0);
    const auto f90 = to_chassis_frame({1, 0, 0}, pi / 2);
    const auto fm90 = to_chassis_frame({1, 0, 0}, -pi / 2);
    const auto right = to_chassis_frame({0, 1, 0}, 0);
    assert(f0 && f90 && fm90 && right);
    near(f0->vx_mps, 1); near(f0->vy_mps, 0);
    near(f90->vx_mps, 0); near(f90->vy_mps, 1);
    near(fm90->vx_mps, 0); near(fm90->vy_mps, -1);
    near(right->vx_mps, 0); near(right->vy_mps, -1);
    const auto spin = to_chassis_frame({0, 0, 2}, 10 * tau + pi / 2);
    assert(spin); near(spin->wz_rad_s, 2);
    const auto multi = to_chassis_frame({1, 0, 0}, 10 * tau + pi / 2);
    assert(multi); near(multi->vx_mps, 0, 2e-5f); near(multi->vy_mps, 1);
    assert(!to_chassis_frame({1, 0, 0}, std::nanf("")));
    assert(!to_chassis_frame({1, 0, 0}, std::numeric_limits<float>::infinity()));
}

void synthetic_wheels() {
    // 合成切向轮布局仅用于数学测试，不是实车轮向参数。
    ec::motion::OmniDrive d{{{{0.2f, 0.2f, 3*pi/4, 0.06f, 1},
                               {0.2f,-0.2f, pi/4, 0.06f, 1},
                               {-0.2f, 0.2f,-3*pi/4, 0.06f, 1},
                               {-0.2f,-0.2f,-pi/4, 0.06f, 1}}}};
    assert(ec::motion::omni_well_posed(d));
    const auto f = wheel_targets(d, {1, 0, 0}, 0);
    const auto turn = wheel_targets(d, {1, 0, 0}, pi/2);
    assert(f && turn);
    const auto fb = ec::motion::omni_twist(d, *f);
    const auto tb = ec::motion::omni_twist(d, *turn);
    assert(fb && tb); near(fb->vx_mps, 1); near(fb->vy_mps, 0);
    near(tb->vx_mps, 0); near(tb->vy_mps, 1);
}

void references() {
    const auto& devices = reference_motors();
    assert(devices.size() == 10);
    assert(devices[0].can_bus == 1 && devices[0].command_id == 1);
    assert(devices[4].mechanism == Mechanism::yaw && devices[4].protocol == Protocol::dm_mit);
    assert(devices[4].feedback_id == 0x005 && devices[4].command_id == 0x005);
    assert(devices[9].protocol == Protocol::dm_mit && devices[9].feedback_id == 0x003);
    // 旧工作树 yaw 收发 ID 均为 0x005；达妙协议允许该组合 (收发方向分离)，
    // 框架已按此修正校验。此处固定该装配事实，防止退回的错误假设。
    ec::drivers::DmDevice yaw({1, 0x005, 0x005, {12.5f, 30.0f, 10.0f}, {20}});
    assert(yaw.configured());
    assert(yaw.encode_enable().has_value());
    const auto g = reference_geometry();
    near(g.wheel_base_m, 0.4f); near(g.wheel_lateral_m, 0.4f); near(g.wheel_radius_m, 0.06f);
}
} // 命名空间
int main() {
    yaw_feedback(); direction_geometry(); synthetic_wheels(); references();
    std::puts("PASS: yaw math, frame geometry, synthetic wheel model, reference inventory");
}
