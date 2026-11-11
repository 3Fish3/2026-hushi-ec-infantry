#include "infantry_config.hpp"

namespace ec::infantry::app {
const std::array<MotorReference, 10>& reference_motors() noexcept {
    static const std::array<MotorReference, 10> motors{{
        {Mechanism::chassis_lf, Protocol::dji, 1, 1, {}, drivers::DjiMode::c620_current},
        {Mechanism::chassis_rf, Protocol::dji, 1, 2, {}, drivers::DjiMode::c620_current},
        {Mechanism::chassis_lb, Protocol::dji, 1, 3, {}, drivers::DjiMode::c620_current},
        {Mechanism::chassis_rb, Protocol::dji, 1, 4, {}, drivers::DjiMode::c620_current},
        {Mechanism::yaw, Protocol::dm_mit, 1, 0x005, 0x005, {}},
        {Mechanism::pitch, Protocol::dji, 2, 5, {}, drivers::DjiMode::c620_current},
        {Mechanism::friction_left, Protocol::dji, 2, 1, {}, drivers::DjiMode::c610_current},
        {Mechanism::friction_right, Protocol::dji, 2, 2, {}, drivers::DjiMode::c610_current},
        {Mechanism::friction_upper, Protocol::dji, 2, 3, {}, drivers::DjiMode::c610_current},
        {Mechanism::trigger, Protocol::dm_mit, 1, 0x002, 0x003, {}},
    }};
    return motors;
}
} // 命名空间 ec::infantry::app
