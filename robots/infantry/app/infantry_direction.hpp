#pragma once
// 几何契约: 底盘 +X 向前、+Y 向左、+Z 向上; yaw 逆时针为正。
// 遥控前/右平移定义在云台坐标系, 变换后的平移在底盘坐标系。
// 云台惯性稳向与云台相对底盘的几何角是不同量；本函数只消费后者。
#include "ec/motion/omni_kinematics.hpp"
#include <cmath>
#include <optional>

namespace ec::infantry::app {
struct GimbalTwist {
    float forward_mps{}; // 沿枪口前向
    float right_mps{};   // 沿枪口右侧
    float spin_rad_s{};  // 底盘绕 +Z 逆时针为正
};

// yaw_ccw_rad 必须由经过标定且新鲜有效的相对角获得；无效反馈由调用方阻止执行。
// 纯数学计算不携带 CAN 时效、机械参考和执行授权，不能直接用于实车输出。
[[nodiscard]] inline std::optional<motion::PlanarTwist> to_chassis_frame(
    const GimbalTwist& command, float yaw_ccw_rad) noexcept {
    if (!std::isfinite(command.forward_mps) || !std::isfinite(command.right_mps) ||
        !std::isfinite(command.spin_rad_s) || !std::isfinite(yaw_ccw_rad)) {
        return std::nullopt;
    }
    const float c = std::cos(yaw_ccw_rad);
    const float s = std::sin(yaw_ccw_rad);
    return motion::PlanarTwist{
        c * command.forward_mps + s * command.right_mps,
        s * command.forward_mps - c * command.right_mps,
        command.spin_rad_s};
}

// 轮位、滚动方向和增益必须由调用者提供；函数不虚构实车装配。
[[nodiscard]] inline std::optional<std::array<float, 4>> wheel_targets(
    const motion::OmniDrive& drive, const GimbalTwist& command,
    float yaw_ccw_rad) noexcept {
    const auto body = to_chassis_frame(command, yaw_ccw_rad);
    if (!body) { return std::nullopt; }
    return motion::omni_wheel_speeds(drive, *body);
}
} // 命名空间 ec::infantry::app
