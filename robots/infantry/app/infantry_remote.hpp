#pragma once
// DT7 本地操纵映射 (O4, C06). 把一致 DR16 快照翻译成本地来源意图与目标, 交给应用层
// 正式接收路径 (InfantryApp::accept_targets); 本模块不直接写设备、不发 CAN、不做仲裁.
//  - 左拨杆选择来源意图, 右拨杆选择人工停止/普通运行/底盘旋转操作;
//  - 步兵 (infantry_controls) 左拨杆中挡为键鼠操纵: 鼠标 X/Y 给主 yaw/pitch, WASD 给底盘;
//    不含射击/摩擦轮/拨弹与零位校准, 左拨杆上挡仍走摇杆映射;
//  - ch4 三层决定右摇杆用途; 改用途的通道必须先回中一次才接管新用途;
//  - 角度通道按目标角速度与 dt 积分成角度目标, 回中保持目标角, 初次接管取有效实际姿态;
//  - 拨弹不锁存 (回中或离开发射机构层立即取消), 摩擦轮跨层保持但停止/失联/换源清除.
#include "ec/input/dr16_input.hpp"
#include "infantry_app.hpp"
#include "infantry_types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::infantry::app {
// 来源意图: 左拨杆上=本地, 中=待机/释放 (步兵: 键鼠本地), 下=Jetson.
enum class SourceIntent : std::uint8_t { none = 0, local, standby, host };

// ch4 三层. unknown 表示尚未建立判读基准 (首个样本) 或取值落在滞回带内.
enum class Ch4Layer : std::uint8_t { unknown = 0, main_gimbal, fire_yaw, launcher };

// ch4 三层判读配置. 中位/上下阈值是原始值, 实测前只是显式夹具, 不代表实机量程.
struct Ch4LayerConfig {
    std::int16_t mid{};
    std::int16_t upper{};
    std::int16_t lower{};
    std::int16_t hysteresis{};
    // true 表示 low 侧是发射机构层, high 侧是 fire yaw 层.
    bool low_is_launcher{true};
};

// 本地操纵配置 (SI). 操纵比例/死区/阈值/速度边界都是显式配置, 不照搬旧内部量.
struct RemoteMappingConfig {
    float stick_deadzone{};
    float chassis_vx_max_mps{};
    float chassis_vy_max_mps{};
    float chassis_wz_max_rad_s{};
    float yaw_rate_max_rad_s{};   // 右 X 在角度层给出的目标角速度上限
    float pitch_rate_max_rad_s{}; // 右 Y 给出的 pitch 目标角速度上限
    float friction_rad_s{};       // 摩擦轮开关采用的目标转速
    float trigger_rad_s{};        // 拨弹目标转速 (反转取反)
    Ch4LayerConfig ch4{};
    // 主 yaw 角度目标用哪个参考系表达.
    // false (默认): local_inertial —— 哨兵, main yaw 没有绝对编码器, 操纵的是惯性方向;
    // true: joint —— 步兵, main yaw 本身是绝对编码器, 操纵的是机械关节角.
    // 由 InfantryCycle 从 InfantryConfig::main_yaw_inertial_driven() 派生, 不手工填.
    bool main_yaw_joint_frame{};
    // 步兵遥控: 右上普通、右中停止、右下小陀螺, 右杆始终操作单云台.
    bool infantry_controls{};
    // 步兵键鼠 (左拨杆中挡): 鼠标每计数对应的关节角增量, 单位 rad/count.
    // 0 = 未标定, 该轴不产生鼠标目标; 方向符号随安装方向在标定时确定, 不在代码里假设.
    float mouse_yaw_rad_per_count{};
    float mouse_pitch_rad_per_count{};
    // 步兵键鼠 WASD 平移速度 (m/s). 0 = 未标定, 不产生键鼠底盘目标.
    float keyboard_chassis_speed_mps{};
};

// DR16 键盘位图 (16 位掩码) 中本模块用到的键位, 位序与旧步兵工程 2026 赛季一致.
enum class KeyboardKey : std::uint16_t {
    w = 0x0001,
    s = 0x0002,
    a = 0x0004,
    d = 0x0008,
};
[[nodiscard]] constexpr bool key_held(std::uint16_t keys, KeyboardKey key) noexcept {
    return (keys & static_cast<std::uint16_t>(key)) != 0;
}

struct RemoteDiagnostics {
    std::uint32_t frames{};
    std::uint32_t recentre_required{}; // 通道改用途后等待回中的次数
    std::uint32_t layer_changes{};
    std::uint32_t source_changes{};
    bool link_lost{};
    std::uint32_t angle_limits{};
};

// 输入通道的"用途": 用途改变后必须先回中一次才接管新用途.
// 主 yaw 方向与 fire yaw 相对角是两个不同用途, 不共用一个目标角.
enum class AxisPurpose : std::uint8_t {
    none = 0,
    rotation_wz,
    main_yaw_direction,
    fire_yaw_direction,
    friction_switch,
    pitch_angle,
    trigger,
};
inline constexpr std::size_t axis_purpose_count = 7;

// 一次映射结果. 目标通过 TargetSnapshot 交给应用层, 来源意图/停止另走权限输入.
struct RemoteCommand {
    SourceIntent intent{SourceIntent::none};
    bool manual_stop{};
    bool rotation_gear{};
    Ch4Layer layer{Ch4Layer::unknown};
    TargetSnapshot targets{};
    bool chassis_active{};
    bool friction_on{};
};

// 本地操纵映射器. 由单一任务拥有, 不分配内存, 不读设备.
class RemoteMapper {
public:
    RemoteMapper(RemoteMappingConfig config, Lifetime remote_ttl) noexcept;

    // 每个控制周期调用一次: sample 为空表示本周期没有新鲜遥控帧 (失联/从未收到).
    // 返回本周期要交给应用的本地目标与意图; 失联/换源会清掉旧摩擦与拨弹意图.
    [[nodiscard]] RemoteCommand step(const std::optional<input::Dr16Snapshot>& sample, Tick now,
        float dt_s, const std::array<std::optional<float>, domain_count>& actual_angle_rad) noexcept;
    [[nodiscard]] const RemoteDiagnostics& diagnostics() const noexcept { return diagnostics_; }
    [[nodiscard]] const RemoteMappingConfig& config() const noexcept { return config_; }
    // 参考语义改变 (关节零位/机械范围/对齐) 后, 该关节的已保持角不再指向同一物理角:
    // 只丢弃这一个域的保持角, 不触碰来源意图、拨杆层与摩擦开关状态.
    void drop_held_angle(Domain domain) noexcept;
    [[nodiscard]] std::optional<float> held_angle(Domain domain) const noexcept;
    void limit_held_angle(Domain domain, float value) noexcept;
    // 有效实际姿态建立新目标 (初次接管 / 降级恢复) 时用; 缺失时该角度通道本周期不给目标.
    void reset() noexcept;

private:
    struct PurposeState {
        bool awaiting_recentre{}; // 用途改变后尚未回中
        float target_rad{};       // 角度用途当前目标角
        bool target_valid{};
    };
    [[nodiscard]] Ch4Layer classify(std::int16_t ch4) const noexcept;
    [[nodiscard]] bool stick_active(float axis) const noexcept;

    RemoteMappingConfig config_{};
    Lifetime ttl_{};
    Ch4Layer layer_{Ch4Layer::unknown};
    SourceIntent intent_{SourceIntent::none};
    std::array<PurposeState, axis_purpose_count> purposes_{};
    AxisPurpose x_purpose_{AxisPurpose::none};
    AxisPurpose y_purpose_{AxisPurpose::none};
    // 鼠标增量按帧消费: 同一份快照被控制循环重复读取时只应用一次.
    bool mouse_sample_seen_{};
    Tick last_mouse_sample_{};
    // 上一周期是否处于键鼠档, 用于"切回摇杆要先回中"的边沿.
    bool keyboard_mode_{};
    bool friction_on_{};
    bool manual_stop_{};
    RemoteDiagnostics diagnostics_{};
    bool started_{};
};
} // 命名空间 ec::infantry::app
