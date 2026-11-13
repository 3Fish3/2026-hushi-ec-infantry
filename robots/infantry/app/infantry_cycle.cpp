#include "infantry_cycle.hpp"
#include <algorithm>
#include <cmath>

namespace ec::infantry::app {
namespace {
// 主 yaw 的参考系由配置唯一决定: 惯性驱动 (哨兵) → local_inertial;
// 机械驱动 (步兵) → joint. 收在这里, 不让调用方各自抄一遍判据.
RemoteMappingConfig derive_mapping(const InfantryApp& app, RemoteMappingConfig mapping) noexcept {
    mapping.main_yaw_joint_frame = !main_yaw_inertial_driven(app.config());
    mapping.infantry_controls = app.config().infantry_controls;
    return mapping;
}
} // 匿名命名空间
InfantryCycle::InfantryCycle(InfantryApp& app, InfantryExecutor& executor,
    InfantryPerception& perception, RemoteMappingConfig mapping, Lifetime remote_ttl) noexcept
    : app_(app), executor_(executor), perception_(perception), remote_(remote_ttl),
      mapper_(derive_mapping(app, mapping), remote_ttl) {
    reference_epochs_ = {app_.domain_reference_epoch(Domain::main_yaw),
        app_.domain_reference_epoch(Domain::fire_yaw),
        app_.domain_reference_epoch(Domain::pitch)};
}

bool InfantryCycle::accept_remote(const CycleInputs& inputs) noexcept {
    if (!inputs.remote_sample) { return false; }
    // 槽位语义是"最近一份样本", 控制任务可能连续多个周期读到同一份:
    // 按到达时刻去重, 否则会重复计数并反复重建连续基准 (边沿判定因此失效).
    // 毫秒时间戳按 2^32 回绕: 相同时间戳是重复, 半个回绕窗口内的倒退才是真回退,
    // 不能用普通 `<` 把回绕后的前进误判成"不更新".
    if (has_remote_) {
        const std::uint32_t gap = inputs.remote_received_at.ms - last_remote_at_.ms;
        if (gap == 0u || gap >= 0x80000000u) { return false; }
    }
    if (!remote_.accept(*inputs.remote_sample, inputs.remote_received_at)) { return false; }
    has_remote_ = true;
    last_remote_at_ = inputs.remote_received_at;
    return true;
}

void InfantryCycle::refresh_feedback(Tick now) noexcept {
    for (auto& entry : inputs_.feedback) { entry = DomainFeedback{}; }
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        if (!app_.config().dji[i].installed) { continue; }
        auto* device = executor_.dji_device(i);
        if (device == nullptr) { continue; }
        const auto state = device->state(now);
        auto& entry = inputs_.feedback[domain_index(app_.config().dji[i].domain)];
        entry.position_rad = state.position_rad;
        entry.velocity_rad_s = state.velocity_rad_s;
    }
    if (app_.config().dm[0].installed) {
        // DM 域: 与健康判定/Mapper 共用同一个关节视图 (已减零位偏移),
        // 不能向应用回传设备原生角 —— 非零零位时两者不是同一个坐标.
        const auto domain = app_.config().dm[0].domain;
        inputs_.feedback[domain_index(domain)] = executor_.joint_feedback(domain, now);
    }
}

CycleOutcome InfantryCycle::control(const CycleInputs& inputs) noexcept {
    outcome_ = CycleOutcome{};
    outcome_.remote_frame_accepted = accept_remote(inputs);

    // 感知先消费真实关节视图; health 建立唯一可恢复的机械参考, 不提供假零位.
    inputs_.faults = executor_.health(inputs.now);
    for (const auto domain : {Domain::fire_yaw, Domain::pitch}) {
        const auto time = executor_.feedback_time(domain);
        if (time) {
            perception_.submit_joint(domain, executor_.joint_feedback(domain, inputs.now), *time,
                app_.domain_reference_epoch(domain));
        }
    }
    if (inputs.imu_gyro_rad_s && inputs.imu_accel_mps2) {
        if (inputs.imu_times) {
            perception_.submit_timed_imu(*inputs.imu_gyro_rad_s, *inputs.imu_accel_mps2, *inputs.imu_times,
                inputs.now, inputs.now_us.value_or(MicroTick{inputs.now.ms*1000u}));
        } else {
            perception_.submit_imu(*inputs.imu_gyro_rad_s, *inputs.imu_accel_mps2, inputs.imu_sampled_at, inputs.now);
        }
        outcome_.imu_submitted = true;
    }
    perception_.update(inputs.now, inputs.now_us);
    if (local_epoch_ != perception_.local_epoch()) {
        // 惯性参考失效只在惯性驱动时才有意义: 机械驱动下 IMU 重新初始化与 main yaw
        // 的坐标基准无关, 不应推高参考代次 (那会让外部客户端看到无意义的 epoch 抖动).
        if (local_epoch_ != 0 && main_yaw_inertial_driven(app_.config())) {
            app_.invalidate_inertial_reference();
        }
        local_epoch_ = perception_.local_epoch();
    }

    // 1. 操纵映射: 缺省样本 (失联/从未收到) 会让映射器清掉旧摩擦与拨弹意图.
    //    实际关节角来自执行层的关节视图 (减零位后的同一坐标), 不用设备原生位置.
    std::array<std::optional<float>, domain_count> joint_rad{};
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        if (!is_joint(domain)) { continue; }
        const auto feedback = executor_.joint_feedback(domain, inputs.now);
        if (feedback.position_rad.validity == Validity::valid) {
            joint_rad[i] = feedback.position_rad.value;
        }
    }
    const auto main_index = domain_index(Domain::main_yaw);
    const auto main_joint = joint_rad[main_index];
    // 惯性驱动 (哨兵): main yaw 没有绝对编码器, 本地操纵捕获的是局部惯性方向,
    // 其它两个域仍是机械关节角.
    // 机械驱动 (步兵): main_joint 已经是绝对关节角, 必须原样保留 —— 覆盖成惯性方向
    // 会把它变成 nullopt, 使 mapper 永远建不起 main yaw 目标.
    if (main_yaw_inertial_driven(app_.config())) {
        joint_rad[main_index] = main_joint ? perception_.local_direction_rad() : std::nullopt;
        if (!joint_rad[main_index]) { mapper_.drop_held_angle(Domain::main_yaw); }
    }
    const auto snapshot = remote_.snapshot(inputs.now);
    outcome_.remote_online = snapshot.has_value();
    // 参考语义改变 (零位/范围/对齐) 后, 该关节已保持的本地目标角属于旧坐标: 丢弃它,
    // 下次接管时按新的实际姿态重新建立, 不用旧数值继续驱动.
    const auto drop_stale_hold = [&](Domain domain) {
        const auto slot = joint_index(domain);
        const auto epoch = app_.domain_reference_epoch(domain);
        if (reference_epochs_[slot] == epoch) { return; }
        reference_epochs_[slot] = epoch;
        mapper_.drop_held_angle(domain);
    };
    drop_stale_hold(Domain::main_yaw);
    drop_stale_hold(Domain::fire_yaw);
    drop_stale_hold(Domain::pitch);
    const auto previous_heading = mapper_.held_angle(Domain::main_yaw);
    auto command = mapper_.step(snapshot, inputs.now, inputs.dt_s, joint_rad);
    // 试验中途失联/停止/反馈或参考失效, 结束本次激励; 不自动换基准重新起振.
    const auto sweep_heading = perception_.local_direction_rad();
    const bool sweep_ready = command.intent == SourceIntent::local && !command.manual_stop &&
        main_joint && sweep_heading && !inputs_.faults.device_fault[main_index] &&
        !inputs_.faults.feedback_lost[main_index] && !inputs_.faults.reference_lost[main_index] &&
        !inputs_.faults.execution_rejected[main_index];
    if (sweep_base_valid_ && (!sweep_ready ||
        sweep_reference_epoch_ != app_.domain_reference_epoch(Domain::main_yaw))) {
        sweep_.enabled = false;
        sweep_base_valid_ = false;
        mapper_.drop_held_angle(Domain::main_yaw);
        command.targets.domains[main_index].reset();
    }
    // 频率响应测试: 目标 = 首次进入时的基准航向 + A·sin(2πft).
    // 未开始的测试允许在停止挡配置, 待明确进入运行态后首次捕获基准.
    if (sweep_.enabled && sweep_ready) {
        const auto heading = sweep_heading;
        if (heading) {
            if (!sweep_base_valid_) {
                sweep_base_rad_ = *heading;
                sweep_base_valid_ = true;
                sweep_start_ = inputs.now;
                sweep_reference_epoch_ = app_.domain_reference_epoch(Domain::main_yaw);
                mapper_.drop_held_angle(Domain::main_yaw);
            }
            const float elapsed_s = static_cast<float>(inputs.now.ms - sweep_start_.ms) * 0.001f;
            constexpr float tau = 6.28318530717958647692f;
            const float phase = tau * sweep_.frequency_hz * elapsed_s;
            command.targets.domains[main_index] = DomainSetpoint{
                control::ControlMode::position,
                sweep_base_rad_ + sweep_.amplitude_rad * std::sin(phase),
                0, YawFrame::local_inertial};
        }
    } else if (sweep_base_valid_) {
        // 关闭测试: 清基准, 下次进入重新抓取当前航向.
        sweep_base_valid_ = false;
    }
    // 世界稳向: 目标航向**只由摇杆决定** (mapper 在首次接管时取当前实测航向,
    // 之后按摇杆增量积分并回中保持). 这里不再把目标重读成当前实测值 ——
    // 那会让目标跟着实测跑, 稳向与摇杆控制同时失效.
    // 云台因此既能稳向 (摇杆不动时底盘转动不带走枪管), 又能被飞手自由转动.
    // 步兵方向契约: 本地上位意图里的平移量在**云台系**表达, 进应用层前旋到机体系.
    //   v_body = R(+psi) * v_gimbal,  psi = gain * 云台关节角 (逆时针为正)
    // psi = 0 时恒等; psi = +90° 且推前进 -> 机体系 (0, +v), 即底盘向自己左侧平移,
    // 使云台枪管始终指向此刻的"前进". wz 是绕同一根 Z 轴的角速度, 与参考系无关, 不旋转.
    // 云台角无效时不产生平移意图: 方向契约无法成立, 宁可不动也不按错的参考走.
    const float follow_gain = app_.config().chassis_follow_gain;
    if (follow_gain != 0.0f && command.chassis_active && command.targets.chassis) {
        if (!main_joint) {
            command.targets.chassis.reset();
            command.chassis_active = false;
        } else {
            const float psi = follow_gain * *main_joint;
            const float c = std::cos(psi), s = std::sin(psi);
            const auto& t = *command.targets.chassis;
            command.targets.chassis = motion::PlanarTwist{
                t.vx_mps * c - t.vy_mps * s,
                t.vx_mps * s + t.vy_mps * c,
                t.wz_rad_s};
        }
    }
    for (const auto domain : {Domain::fire_yaw, Domain::pitch}) {
        auto& target = command.targets.domains[domain_index(domain)];
        const auto& range = app_.joint_geometry(domain);
        if (target && range.min_rad && range.max_rad) {
            target->value = std::clamp(target->value, *range.min_rad, *range.max_rad);
            mapper_.limit_held_angle(domain, target->value);
        }
    }
    auto& main_target = command.targets.domains[main_index];
    const auto& platform = perception_.platform();
    if (main_target && main_joint && app_.config().serial) {
        const auto& range = app_.joint_geometry(Domain::main_yaw);
        const auto reachable = [&](float heading) {
            return range.min_rad && range.max_rad && reachable_heading(*app_.config().serial,
                platform.value.orientation, *main_joint, heading, *range.min_rad, *range.max_rad).has_value();
        };
        if (!reachable(main_target->value)) {
            main_target->value = previous_heading && reachable(*previous_heading) ? *previous_heading : platform.continuous_heading;
            mapper_.limit_held_angle(Domain::main_yaw, main_target->value);
        }
    }
    outcome_.intent = command.intent;
    outcome_.manual_stop = command.manual_stop;
    outcome_.layer = command.layer;
    outcome_.chassis_active = command.chassis_active;
    outcome_.friction_on = command.friction_on;

    // 3. 输入视图: 反馈/惯性/链路/外部约束都来自真实来源; 缺失保持 unknown/unavailable.
    inputs_.inertial = perception_.inertial();
    inputs_.main_yaw = perception_.platform();
    // 只有惯性驱动的 main yaw 才存在"惯性失效但仍可用机械角保持"的降级语义.
    // 机械驱动时 platform 恒为 unavailable, 若照搬就会每周期误报降级并撤输出.
    inputs_.faults.main_yaw_inertial_only = main_yaw_inertial_driven(app_.config()) &&
        inputs_.main_yaw.validity != DomainValidity::valid;
    inputs_.aux = inputs.aux;
    inputs_.epochs = Epochs{app_.config_epoch(), app_.reference_epoch()};
    refresh_feedback(inputs.now);
    std::array<DomainFeedback, 4> wheels{};
    std::array<std::optional<MicroTick>, 4> wheel_times{};
    for (std::size_t i=0; i<4; ++i) {
        wheels[i] = inputs_.feedback[i];
        wheel_times[i] = executor_.feedback_time(static_cast<Domain>(i));
    }
    perception_.update_wheel_odom(wheels, wheel_times, inputs.now,
        inputs.now_us.value_or(MicroTick{inputs.now.ms*1000u}));
    inputs_.link = LinkInputs{};
    inputs_.link.remote_online = outcome_.remote_online;
    if (snapshot) {
        inputs_.link.switch_left = snapshot->switch_left;
        inputs_.link.switch_right = snapshot->switch_right;
        // 仲裁器的停止挡约定为 down; 步兵物理中挡是停止, 下挡是小陀螺.
        // 仅转换给仲裁器的视图, 原始遥控样本仍保持真实挡位.
        if (app_.config().infantry_controls) {
            // 左中键鼠和左上摇杆均属于本地来源, 不将键鼠误判为待机.
            if (snapshot->switch_left == drivers::Dr16Switch::mid) {
                inputs_.link.switch_left = drivers::Dr16Switch::up;
            }
            if (snapshot->switch_right == drivers::Dr16Switch::mid) {
                inputs_.link.switch_right = drivers::Dr16Switch::down;
            } else if (snapshot->switch_right == drivers::Dr16Switch::down) {
                inputs_.link.switch_right = drivers::Dr16Switch::mid;
            }
        }
        inputs_.link.remote_sampled_at = snapshot->sampled_at;
    }
    inputs_.link.host_link_alive = inputs.host_link_alive;
    inputs_.link.host_request = inputs.host_request;
    inputs_.link.host_session_epoch = inputs.host_session_epoch;
    inputs_.link.maintenance_link_alive = inputs.maintenance_link_alive;
    inputs_.link.maintenance_request = inputs.maintenance_request;
    inputs_.link.maintenance_session_epoch = inputs.maintenance_session_epoch;
    inputs_.link.locally_ready = inputs.locally_ready;

    // 4. 本地来源的目标经同一入口交给应用. 必须在 update 之前: 执行层读的是 update
    //    产出的执行判定, 目标晚一个周期才生效会让"已接受"与"本周期实际下发"错位.
    //    停止态/非本地来源不下发本地目标 (来源资格仍由应用仲裁判定).
    if (command.intent == SourceIntent::local && !command.manual_stop) {
        outcome_.accept_attempted = true;
        // 该参数问的是"主 yaw 是否有可用的惯性方向"; 机械驱动的 main yaw 不需要它.
        const bool inertial_available = !main_yaw_inertial_driven(app_.config()) ||
            inputs_.main_yaw.validity == DomainValidity::valid;
        outcome_.accept = app_.accept_remote_targets(command.targets, inputs.now, inertial_available);
    }

    // 5. 应用仲裁: 来源选择/人工停止/故障影响/目标生命周期都在应用层完成.
    app_.update(inputs.now, inputs_);
    return outcome_;
}

void InfantryCycle::commit(const CycleInputs& inputs) noexcept {
    // 执行提交: 真实设备编解码与 CAN 组帧都在执行层, 编排只负责调用顺序.
    executor_.step(inputs.now, inputs.dt_s, app_, inputs.now_us);
    // 命令预测 (不是轮式反馈里程计): 用应用实际采用的 twist 积分.
    const auto& execution = app_.snapshot().executions;
    const bool executing_chassis = std::all_of(execution.begin(), execution.begin()+4,
        [](const auto& e) { return e.execute && e.mode == control::ControlMode::velocity; });
    if (const auto twist = app_.adopted_twist(); twist && executing_chassis) {
        perception_.integrate_commanded_odom(*twist, inputs.now);
    } else { perception_.suspend_commanded_odom(); }
}
} // 命名空间 ec::infantry::app
