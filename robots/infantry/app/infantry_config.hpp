#pragma once
// 步兵装配资料。下面是旧工程当前工作树中的软件配置，不代表已实测的实车接线。
#include "ec/drivers/dji.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace ec::infantry::app {

enum class Mechanism : std::uint8_t {
    chassis_lf, chassis_rf, chassis_lb, chassis_rb,
    yaw, pitch, friction_left, friction_right, friction_upper, trigger,
};

enum class Protocol : std::uint8_t { dji, dm_mit };

// 显式记录旧代码的协议选择、总线及 ID。型号注释冲突处不得据此推断真实电机型号。
struct MotorReference {
    Mechanism mechanism;
    Protocol protocol;
    std::uint8_t can_bus;
    std::uint16_t command_id;
    std::optional<std::uint16_t> feedback_id;
    std::optional<drivers::DjiMode> dji_mode;
};

// DJI command_id 是电调 ID，不是编组 CAN 帧 ID；DM 是 MIT 命令帧 ID。
[[nodiscard]] const std::array<MotorReference, 10>& reference_motors() noexcept;

// 设备参数来源于旧工程；机械方向、滚动方向、零位和限幅仍需现场独立验证。
struct GeometryReference {
    float wheel_base_m;
    float wheel_lateral_m;
    float wheel_radius_m;
};
[[nodiscard]] constexpr GeometryReference reference_geometry() noexcept {
    return {0.4f, 0.4f, 0.06f};
}

} // 命名空间 ec::infantry::app
