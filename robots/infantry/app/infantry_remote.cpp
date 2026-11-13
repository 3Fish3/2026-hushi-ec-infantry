#include "infantry_remote.hpp"
#include <algorithm>
#include <cmath>

namespace ec::infantry::app {
namespace {
// 单帧鼠标计数的合理性上限 (旧步兵工程同值). 只用于丢弃明显异常的整帧跳变,
// 不是控制增益: 实际角增量由配置的 rad/count 决定.
constexpr int mouse_count_limit = 1200;

[[nodiscard]] float clampf(float value, float limit) noexcept {
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}
[[nodiscard]] std::uint32_t index(Domain domain) noexcept { return domain_index(domain); }
// 用途索引: 只用于取该用途的目标角与回中状态.
constexpr std::size_t purpose_index(AxisPurpose purpose) noexcept {
    return static_cast<std::size_t>(purpose);
}
constexpr AxisPurpose angle_purpose(Domain domain) noexcept {
    switch (domain) {
    case Domain::main_yaw: return AxisPurpose::main_yaw_direction;
    case Domain::fire_yaw: return AxisPurpose::fire_yaw_direction;
    case Domain::pitch: return AxisPurpose::pitch_angle;
    default: return AxisPurpose::none;
    }
}
} // 内部辅助

RemoteMapper::RemoteMapper(RemoteMappingConfig config, Lifetime remote_ttl) noexcept
    : config_(config), ttl_(remote_ttl) {}

// 参考语义改变: 该关节的已保持角属于旧坐标, 不能作为新参考下的目标继续执行.
// 只丢弃保持角, 不清来源意图/拨杆层/摩擦开关状态.
void RemoteMapper::drop_held_angle(Domain domain) noexcept {
    const auto purpose = angle_purpose(domain);
    if (purpose == AxisPurpose::none) { return; }
    auto& state = purposes_[purpose_index(purpose)];
    state.target_valid = false;
    state.target_rad = 0.0f;
}

std::optional<float> RemoteMapper::held_angle(Domain domain) const noexcept {
    const auto purpose = angle_purpose(domain);
    const auto& state = purposes_[purpose_index(purpose)];
    return purpose != AxisPurpose::none && state.target_valid ? std::optional{state.target_rad} : std::nullopt;
}
void RemoteMapper::limit_held_angle(Domain domain, float value) noexcept {
    const auto purpose = angle_purpose(domain);
    auto& state = purposes_[purpose_index(purpose)];
    if (purpose == AxisPurpose::none || !state.target_valid) { return; }
    if (state.target_rad != value) { ++diagnostics_.angle_limits; }
    state.target_rad = value;
}
void RemoteMapper::reset() noexcept {
    layer_ = Ch4Layer::unknown;
    intent_ = SourceIntent::none;
    purposes_ = {};
    x_purpose_ = AxisPurpose::none;
    y_purpose_ = AxisPurpose::none;
    mouse_sample_seen_ = false;
    last_mouse_sample_ = {};
    keyboard_mode_ = false;
    friction_on_ = false;
    manual_stop_ = false;
    started_ = false;
}

// 用途索引: 只用于取该用途的目标角与回中状态.
Ch4Layer RemoteMapper::classify(std::int16_t ch4) const noexcept {
    const auto& c = config_.ch4;
    // 滞回: 只有越过带外阈值才换层, 避免中位附近抖动导致反复切层.
    const auto high = [&](std::int16_t value) { return value > c.upper; };
    const auto low = [&](std::int16_t value) { return value < c.lower; };
    Ch4Layer side = Ch4Layer::unknown;
    if (high(ch4)) { side = c.low_is_launcher ? Ch4Layer::fire_yaw : Ch4Layer::launcher; }
    else if (low(ch4)) { side = c.low_is_launcher ? Ch4Layer::launcher : Ch4Layer::fire_yaw; }
    else { side = Ch4Layer::main_gimbal; }
    if (layer_ == Ch4Layer::unknown || side == layer_) { return side; }
    // 换层方向与当前层相反时, 需要越过带外阈值 (滞回带内保持当前层).
    if (side == Ch4Layer::main_gimbal) { return Ch4Layer::main_gimbal; }
    const bool moving_up = ch4 > c.mid;
    const bool inside_band = (moving_up && ch4 < c.upper + c.hysteresis) ||
        (!moving_up && ch4 > c.lower - c.hysteresis);
    return inside_band ? layer_ : side;
}

bool RemoteMapper::stick_active(float axis) const noexcept {
    return std::fabs(axis) > config_.stick_deadzone;
}

RemoteCommand RemoteMapper::step(const std::optional<input::Dr16Snapshot>& sample, Tick now,
    float dt_s, const std::array<std::optional<float>, domain_count>& actual_angle_rad) noexcept {
    RemoteCommand out{};
    static_cast<void>(now); // 时效由 Dr16Input 判定, 这里只处理已经一致的快照
    if (!sample) {
        // 失联/从未收到: 清除本地摩擦与拨弹意图, 不自动恢复; 已生效的人工停止不被解除.
        diagnostics_.link_lost = true;
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
        out.manual_stop = manual_stop_;
        return out;
    }
    diagnostics_.link_lost = false;
    ++diagnostics_.frames;
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) { dt_s = 0.0f; }

    // 左拨杆: 上=本地, 中=待机/释放正常来源, 下=Jetson. 遥控自身不产生 Jetson 目标.
    const auto previous_intent = intent_;
    // 步兵左拨杆中挡是键鼠操纵 (旧工程 switch1==3), 同属本地来源; 哨兵中挡仍是待机/释放.
    const auto mid_intent = config_.infantry_controls ? SourceIntent::local : SourceIntent::standby;
    switch (sample->switch_left) {
    case drivers::Dr16Switch::up: intent_ = SourceIntent::local; break;
    case drivers::Dr16Switch::mid: intent_ = mid_intent; break;
    case drivers::Dr16Switch::down: intent_ = SourceIntent::host; break;
    case drivers::Dr16Switch::unknown: intent_ = SourceIntent::none; break;
    }
    if (intent_ != previous_intent) {
        ++diagnostics_.source_changes;
        // 换源清掉人工摩擦/拨弹意图与角度通道, 不自动恢复.
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
    }
    // 步兵按旧遥控约定: 右上普通、右中停止、右下小陀螺; 原框架映射保持不变.
    manual_stop_ = sample->switch_right == (config_.infantry_controls
        ? drivers::Dr16Switch::mid : drivers::Dr16Switch::down);
    out.manual_stop = manual_stop_;
    out.intent = manual_stop_ ? SourceIntent::standby : intent_;
    out.rotation_gear = sample->switch_right == (config_.infantry_controls
        ? drivers::Dr16Switch::down : drivers::Dr16Switch::up);

    const auto previous_layer = layer_;
    // 单云台步兵不启用哨兵 fire_yaw / 发射机构的 ch4 层.
    layer_ = config_.infantry_controls ? Ch4Layer::main_gimbal : classify(sample->ch4);
    if (layer_ != previous_layer) { ++diagnostics_.layer_changes; }
    out.layer = layer_;

    const bool local_active = out.intent == SourceIntent::local && !manual_stop_;

    // 本周期两个输入通道的用途. 用途改变 → 该用途必须先看到一次回中才接管.
    AxisPurpose x_purpose = AxisPurpose::none;
    AxisPurpose y_purpose = AxisPurpose::none;
    if (local_active) {
        if (out.rotation_gear && !config_.infantry_controls) { x_purpose = AxisPurpose::rotation_wz; }
        else if (layer_ == Ch4Layer::main_gimbal) { x_purpose = AxisPurpose::main_yaw_direction; }
        else if (layer_ == Ch4Layer::fire_yaw) { x_purpose = AxisPurpose::fire_yaw_direction; }
        else if (layer_ == Ch4Layer::launcher) { x_purpose = AxisPurpose::friction_switch; }
        if (layer_ == Ch4Layer::main_gimbal || layer_ == Ch4Layer::fire_yaw) {
            y_purpose = AxisPurpose::pitch_angle;
        } else if (layer_ == Ch4Layer::launcher) {
            y_purpose = AxisPurpose::trigger;
        }
    }
    const auto purpose_changed = [&](AxisPurpose previous, AxisPurpose current) {
        if (previous == current || current == AxisPurpose::none) { return false; }
        auto& state = purposes_[purpose_index(current)];
        // 任何"改用途"都要求先看到一次回中: 角度用途、开关类用途与旋转档覆盖用同一条规则.
        // 不因为"首次进入该用途"就接受旧偏转 (C06).
        state.awaiting_recentre = true;
        ++diagnostics_.recentre_required;
        return true;
    };
    static_cast<void>(purpose_changed(x_purpose_, x_purpose));
    static_cast<void>(purpose_changed(y_purpose_, y_purpose));
    x_purpose_ = x_purpose;
    y_purpose_ = y_purpose;

    // 回中一次后才允许该用途接管; 未操纵的角度用途保持目标角.
    const auto may_act = [&](AxisPurpose purpose, bool active) {
        auto& state = purposes_[purpose_index(purpose)];
        if (state.awaiting_recentre) {
            if (active) { return false; }
            state.awaiting_recentre = false;
        }
        return true;
    };

    // 左杆前后/左右始终给底盘 vx/vy; 旋转档的 wz 由右 X 经同一回中接管后覆盖 (见下).
    // 未标定的映射配置 (量程为 0) 不得生成"有效零目标": 零值不等于已标定的零运动意图.
    const bool chassis_configured = config_.chassis_vx_max_mps > 0.0f ||
        config_.chassis_vy_max_mps > 0.0f || config_.chassis_wz_max_rad_s > 0.0f;
    // 步兵键鼠: 左拨杆中挡即本地键盘/鼠标操纵 (旧工程 switch1==3); 上挡仍是摇杆.
    const bool keyboard_mode = config_.infantry_controls && local_active &&
        sample->switch_left == drivers::Dr16Switch::mid;
    // 键鼠切回摇杆 (仍在本地来源): 摇杆此刻可能仍偏转, 必须回到中位一次才允许接管角度用途.
    // 离开本地来源时 purposes_ 会被整体清空, 不需要这一步.
    if (keyboard_mode_ && !keyboard_mode && local_active) {
        purposes_[purpose_index(AxisPurpose::main_yaw_direction)].awaiting_recentre = true;
        purposes_[purpose_index(AxisPurpose::pitch_angle)].awaiting_recentre = true;
        ++diagnostics_.recentre_required;
    }
    keyboard_mode_ = keyboard_mode;
    // 鼠标字段是"自上一帧以来的计数增量": 槽位是高频保持快照, 同一份样本可能被控制循环
    // 连续读到多次; 只在新样本上消费一次, 避免同一段位移被逐周期重复积分.
    // Snapshot 没有独立帧序号, 以采样毫秒时间戳区分; InfantryCycle::accept_remote 已按到达
    // 毫秒严格去重 (相同时间戳的重复读取不再入库), 因此时间戳相同即同一帧.
    const bool mouse_frame_new = keyboard_mode &&
        (!mouse_sample_seen_ || sample->sampled_at.ms != last_mouse_sample_.ms);
    if (keyboard_mode) {
        mouse_sample_seen_ = true;
        last_mouse_sample_ = sample->sampled_at;
        // 键鼠底盘: WASD 在机体坐标 (+X 前 / +Y 左) 给出平移; 小陀螺档给固定 wz.
        // 未标定的部分不参与: 平移未标定时不发零平移, 只有小陀螺档且 wz 已配置才发旋转.
        const bool translation_ready = config_.keyboard_chassis_speed_mps > 0.0f;
        const bool rotation_ready = out.rotation_gear && config_.chassis_wz_max_rad_s > 0.0f;
        if (translation_ready || rotation_ready) {
            const float forward = (key_held(sample->keys, KeyboardKey::w) ? 1.0f : 0.0f) -
                (key_held(sample->keys, KeyboardKey::s) ? 1.0f : 0.0f);
            const float left = (key_held(sample->keys, KeyboardKey::a) ? 1.0f : 0.0f) -
                (key_held(sample->keys, KeyboardKey::d) ? 1.0f : 0.0f);
            out.targets.chassis = motion::PlanarTwist{
                translation_ready ? forward * config_.keyboard_chassis_speed_mps : 0.0f,
                translation_ready ? left * config_.keyboard_chassis_speed_mps : 0.0f,
                rotation_ready ? config_.chassis_wz_max_rad_s : 0.0f};
            out.chassis_active = true;
        }
    } else {
        mouse_sample_seen_ = false;
        if (local_active && chassis_configured) {
            out.targets.chassis = motion::PlanarTwist{
                clampf(sample->axis[3], 1.0f) * config_.chassis_vx_max_mps,
                clampf(sample->axis[2], 1.0f) * config_.chassis_vy_max_mps,
                config_.infantry_controls && out.rotation_gear ? config_.chassis_wz_max_rad_s : 0.0f};
            out.chassis_active = true;
        }
    }

    const float right_x = sample->axis[0];
    const float right_y = sample->axis[1];

    // 已保持的角度目标每周期都要出现在完整快照里: 切层只改变"当前操纵哪个通道",
    // 不能让另一个轴的保持角从快照里消失 —— 完整快照会把缺省域当作撤销, 于是"没操纵的轴
    // 保持"会变成"被撤输出".
    const auto frame_of = [&](Domain domain) noexcept {
        return domain == Domain::main_yaw && !config_.main_yaw_joint_frame
            ? YawFrame::local_inertial : YawFrame::joint;
    };
    const auto emit_held_angle = [&](AxisPurpose purpose, Domain domain) {
        const auto& state = purposes_[purpose_index(purpose)];
        if (!state.target_valid) { return; }
        out.targets.domains[index(domain)] =
            DomainSetpoint{control::ControlMode::position, state.target_rad, 0, frame_of(domain)};
    };

    // 角度用途: 目标角速度 × dt 积分成角度目标, 回中保持目标角.
    const auto apply_angle = [&](AxisPurpose purpose, float axis, Domain domain, float rate_max) {
        const bool active = stick_active(axis);
        if (!may_act(purpose, active)) { return; }
        auto& state = purposes_[purpose_index(purpose)];
        if (!state.target_valid) {
            // 初次接管取有效实际姿态建立新目标; 没有有效实际姿态就不给目标.
            const auto actual = actual_angle_rad[index(domain)];
            if (!actual) { return; }
            state.target_rad = *actual;
            state.target_valid = true;
        }
        if (active && dt_s > 0.0f) { state.target_rad += clampf(axis, 1.0f) * rate_max * dt_s; }
        out.targets.domains[index(domain)] =
            DomainSetpoint{control::ControlMode::position, state.target_rad, 0, frame_of(domain)};
    };

    // 步兵键鼠角度: 鼠标计数是帧增量, 直接叠加到同一用途的保持角上, 与摇杆共用
    // "未操纵保持目标角"的语义; 每帧只消费一次 (见 mouse_frame_new).
    if (keyboard_mode && mouse_frame_new) {
        const auto integrate_mouse = [&](AxisPurpose purpose, Domain domain, std::int16_t counts,
                                         float rad_per_count) {
            if (rad_per_count == 0.0f) { return; }
            auto& state = purposes_[purpose_index(purpose)];
            if (!state.target_valid) {
                // 首次接管取有效实际姿态; 没有有效实际姿态就不给目标.
                const auto actual = actual_angle_rad[index(domain)];
                if (!actual) { return; }
                state.target_rad = *actual;
                state.target_valid = true;
            }
            // 鼠标没有"回中"概念: 该用途由帧增量直接建立, 清掉摇杆用途的等待回中.
            state.awaiting_recentre = false;
            const int clamped = std::clamp(static_cast<int>(counts), -mouse_count_limit, mouse_count_limit);
            state.target_rad += static_cast<float>(clamped) * rad_per_count;
        };
        // 轴向与符号由配置的 rad/count 决定; 代码不猜鼠标 X/Y 的方向与比例.
        integrate_mouse(AxisPurpose::main_yaw_direction, Domain::main_yaw, sample->raw.mouse_x,
            config_.mouse_yaw_rad_per_count);
        integrate_mouse(AxisPurpose::pitch_angle, Domain::pitch, sample->raw.mouse_y,
            config_.mouse_pitch_rad_per_count);
    }

    switch (x_purpose) {
    case AxisPurpose::main_yaw_direction:
        // 键鼠档的 yaw 由鼠标增量给出, 不再叠加右摇杆.
        if (!keyboard_mode && config_.yaw_rate_max_rad_s > 0.0f) {
            apply_angle(x_purpose, right_x, Domain::main_yaw, config_.yaw_rate_max_rad_s);
        }
        break;
    case AxisPurpose::fire_yaw_direction:
        if (config_.yaw_rate_max_rad_s > 0.0f) {
            apply_angle(x_purpose, right_x, Domain::fire_yaw, config_.yaw_rate_max_rad_s);
        }
        break;
    case AxisPurpose::friction_switch: {
        // 右 X 正向开/负向关/中位保持; 不锁存停止, 跨层保留.
        const bool active = stick_active(right_x);
        if (may_act(x_purpose, active)) {
            if (right_x > config_.stick_deadzone) { friction_on_ = true; }
            else if (right_x < -config_.stick_deadzone) { friction_on_ = false; }
        }
        break;
    }
    case AxisPurpose::rotation_wz: {
        // 旋转档覆盖同一轴的用途: 与角度/开关用途走同一条回中接管, 不直接使用轴值.
        const bool active = stick_active(right_x);
        if (may_act(x_purpose, active) && active && out.chassis_active) {
            out.targets.chassis = motion::PlanarTwist{out.targets.chassis->vx_mps,
                out.targets.chassis->vy_mps,
                clampf(right_x, 1.0f) * config_.chassis_wz_max_rad_s};
        }
        break;
    }
    case AxisPurpose::pitch_angle:
    case AxisPurpose::trigger:
    case AxisPurpose::none:
        break;
    }

    if (y_purpose == AxisPurpose::pitch_angle) {
        // 未标定的 pitch 速率 (0) 不生成位置保持目标 (那会是"有效零目标").
        // 键鼠档的 pitch 由鼠标增量给出, 不再叠加右摇杆.
        if (!keyboard_mode && config_.pitch_rate_max_rad_s > 0.0f) {
            apply_angle(y_purpose, right_y, Domain::pitch, config_.pitch_rate_max_rad_s);
        }
    } else if (y_purpose == AxisPurpose::trigger) {
        // 拨弹不锁存: 中位立即取消, 负向人工反转.
        const bool active = stick_active(right_y);
        if (config_.trigger_rad_s > 0.0f && may_act(y_purpose, active) && active) {
            out.targets.domains[index(Domain::trigger)] = DomainSetpoint{
                control::ControlMode::velocity,
                right_y > 0.0f ? config_.trigger_rad_s : -config_.trigger_rad_s};
        }
    }
    // 已保持的角度目标每周期都要出现在完整快照里: 切层与"等待回中"只限制新的操纵
    // 增量, 不取消已有保持; 切回已保持的用途时 apply_angle 会提前返回, 因此本轴的保持
    // 也必须在这里补上, 否则完整快照的缺省语义会把它当成撤销.
    if (local_active) {
        emit_held_angle(AxisPurpose::main_yaw_direction, Domain::main_yaw);
        emit_held_angle(AxisPurpose::fire_yaw_direction, Domain::fire_yaw);
        emit_held_angle(AxisPurpose::pitch_angle, Domain::pitch);
    }

    // 摩擦轮: 跨层保留人工运行意图, 由新鲜遥控帧持续生成本地目标.
    // 未标定的目标转速 (0) 不生成"有效零目标".
    out.friction_on = friction_on_;
    if (local_active && friction_on_ && config_.friction_rad_s > 0.0f) {
        out.targets.domains[index(Domain::friction_left)] =
            DomainSetpoint{control::ControlMode::velocity, config_.friction_rad_s};
        out.targets.domains[index(Domain::friction_right)] =
            DomainSetpoint{control::ControlMode::velocity, -config_.friction_rad_s};
    }
    if (!local_active) {
        // 非本地来源或人工停止: 不给本地目标, 并清掉人工摩擦/拨弹意图.
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
    }
    started_ = true;
    return out;
}
} // 命名空间 ec::infantry::app
