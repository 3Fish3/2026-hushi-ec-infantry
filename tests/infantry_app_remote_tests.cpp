// O4 DT7/ch4 全链路契约: 从原始 18 B 帧经 Dr16Stream/Dr16Input/RemoteMapper 到应用采用目标.
// 覆盖串流分片/边界、ch4 中位与两侧、切层抖动、右旋转档覆盖、重复帧、掉线后首帧、
// 时间回绕, 以及改用途通道的回中接管、拨弹不锁存、摩擦跨层保持与换源/停止/失联清除.
// ch4 中位/方向/量程是显式夹具值, 不代表实机取值.
#include "infantry_execution.hpp"
#include "infantry_remote.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

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
using ec::input::Dr16Input;
using ec::input::Dr16Snapshot;
using ec::infantry::app::AcceptResult;
using ec::infantry::app::AxisPurpose;
using ec::infantry::app::Ch4Layer;
using ec::infantry::app::Ch4LayerConfig;
using ec::infantry::app::ConfigAudience;
using ec::infantry::app::Domain;
using ec::infantry::app::DomainSetpoint;
using ec::infantry::app::RemoteCommand;
using ec::infantry::app::RemoteMapper;
using ec::infantry::app::RemoteMappingConfig;
using ec::infantry::app::InfantryApp;
using ec::infantry::app::SourceIntent;
using ec::control::ControlMode;
using ec::transport::CanFrame;

constexpr std::uint32_t index(Domain domain) { return ec::infantry::app::domain_index(domain); }

struct FrameSpec {
    std::array<int, 4> ch{{0, 0, 0, 0}};
    int ch4{};
    Dr16Switch switch_left{Dr16Switch::mid};
    Dr16Switch switch_right{Dr16Switch::mid};
    int mouse_x{};
    int mouse_y{};
    std::uint16_t keys{};
};

// 按官方 C 板示例的位域构造 18 B 帧: ch0–ch3 为 11 bit 位域减 1024, byte5 高位是拨杆,
// byte16..17 是第 5 通道 (小端, 减 1024).
std::array<std::uint8_t, 18> encode_frame(const FrameSpec& spec) {
    std::array<std::uint8_t, 18> buf{};
    std::uint64_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        const auto raw = static_cast<std::uint64_t>(spec.ch[static_cast<std::size_t>(i)] + 1024) &
            0x7ffu;
        bits |= raw << (11 * i);
    }
    for (int i = 0; i < 6; ++i) {
        buf[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xffu);
    }
    buf[5] = static_cast<std::uint8_t>((buf[5] & 0x0fu) |
        ((static_cast<unsigned>(spec.switch_left) & 0x03u) << 6) |
        ((static_cast<unsigned>(spec.switch_right) & 0x03u) << 4));
    const auto write_i16 = [&](std::size_t offset, int value) {
        const auto raw = static_cast<std::uint16_t>(value);
        buf[offset] = static_cast<std::uint8_t>(raw & 0xffu);
        buf[offset + 1] = static_cast<std::uint8_t>(raw >> 8);
    };
    write_i16(6, spec.mouse_x);
    write_i16(8, spec.mouse_y);
    buf[14] = static_cast<std::uint8_t>(spec.keys & 0xffu);
    buf[15] = static_cast<std::uint8_t>(spec.keys >> 8);
    const auto raw4 = static_cast<std::uint16_t>(spec.ch4 + 1024);
    buf[16] = static_cast<std::uint8_t>(raw4 & 0xffu);
    buf[17] = static_cast<std::uint8_t>(raw4 >> 8);
    return buf;
}

RemoteMappingConfig mapping() {
    RemoteMappingConfig config{};
    config.stick_deadzone = 0.05f;
    config.chassis_vx_max_mps = 1.0f;
    config.chassis_vy_max_mps = 1.0f;
    config.chassis_wz_max_rad_s = 2.0f;
    config.yaw_rate_max_rad_s = 1.0f;
    config.pitch_rate_max_rad_s = 1.0f;
    config.friction_rad_s = 25.0f;
    config.trigger_rad_s = 10.0f;
    // 夹具: 中位 0, 高侧 >300, 低侧 <-300, 滞回 50, 低侧为发射机构层.
    config.ch4 = Ch4LayerConfig{0, 300, -300, 50, true};
    return config;
}

// 摇杆归一化: dr16_axis 把 ±660 映到 ±1, 这里给百分比值.
int stick(float normalized) { return static_cast<int>(normalized * 660.0f); }

struct Rig {
    Dr16Stream stream{};
    Dr16Input input{ec::Lifetime{100}};
    RemoteMapper mapper{mapping(), ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    Tick now{1000};
    std::uint32_t sequence{};

    Rig() {
        actual[index(Domain::main_yaw)] = 0.0f;
        actual[index(Domain::fire_yaw)] = 0.0f;
        actual[index(Domain::pitch)] = 0.0f;
        // 上电后首个 IDLE 只建立帧边界, 不交付样本.
        const std::uint8_t priming = 0;
        stream.push({&priming, 1});
        static_cast<void>(stream.on_idle());
    }
    // 分片投递: 一帧被拆成两段 DMA 块, 仅真实 IDLE 结束一帧.
    bool push(const FrameSpec& spec, Tick at, bool fragmented = true) {
        const auto frame = encode_frame(spec);
        if (fragmented) {
            stream.push({frame.data(), 9});
            stream.push({frame.data() + 9, frame.size() - 9});
        } else {
            stream.push(frame);
        }
        const auto sample = stream.on_idle();
        return sample ? input.accept(*sample, at) : false;
    }
    RemoteCommand step(std::uint32_t ms, float dt_s = 0.001f) {
        now = Tick{ms};
        const auto snapshot = input.snapshot(now);
        ++sequence;
        return mapper.step(snapshot, now, dt_s, actual);
    }
};

void raw_frame_contracts() {
    Dr16Stream stream;
    const auto frame = encode_frame(FrameSpec{{100, -200, 300, -400}, 500, Dr16Switch::up,
        Dr16Switch::down});
    // 启动后的首个 IDLE 只建立边界, 不交付样本.
    stream.push({frame.data(), 3});
    CHECK(!stream.on_idle().has_value());
    // 一帧被拆成两段 DMA 块: 分段之间没有 IDLE, 只有帧尾 IDLE 才交付.
    stream.push({frame.data(), 9});
    stream.push({frame.data() + 9, frame.size() - 9});
    const auto sample = stream.on_idle();
    CHECK(sample.has_value());
    CHECK(sample->ch0 == 100);
    CHECK(sample->ch1 == -200);
    CHECK(sample->ch2 == 300);
    CHECK(sample->ch3 == -400);
    CHECK(sample->ch4 == 500);
    CHECK(sample->switch_left == Dr16Switch::up);
    CHECK(sample->switch_right == Dr16Switch::down);

    // 半帧 + IDLE 被丢弃, 不当作有效样本.
    const auto rejected_before = stream.rejected;
    stream.push({frame.data(), 5});
    CHECK(!stream.on_idle().has_value());
    CHECK(stream.rejected == rejected_before + 1);

    // ch4 小端字节序: 0x0123 原始值 → 0x0123 - 1024.
    auto little = frame;
    little[16] = 0x23;
    little[17] = 0x01;
    stream.push(little);
    const auto endian_sample = stream.on_idle();
    CHECK(endian_sample.has_value());
    CHECK(endian_sample->ch4 == 0x0123 - 1024);

    // 通道超范围仍被拒绝 (ch0–ch3 的 ±660 判定不变), 但 ch4 不受该判定限制.
    auto wide = frame;
    wide[16] = 0xff;
    wide[17] = 0xff;
    stream.push(wide);
    const auto wide_sample = stream.on_idle();
    CHECK(wide_sample.has_value());
    // 与官方示例的 int16 算术一致: 原始 0xFFFF 减去 1024 后按 16 位有符号解释.
    CHECK(wide_sample->ch4 == -1025);

    // 掉线后首帧只建立基准, 不产生边沿; 时间回绕不误判为倒退.
    Dr16Input input{ec::Lifetime{100}};
    CHECK(input.accept(*sample, Tick{0xFFFFFFF0u}));
    CHECK(input.online(Tick{0xFFFFFFF5u}));
    const auto wrapped = input.snapshot(Tick{0x00000010u}); // 回绕后仍在有效期内
    CHECK(wrapped.has_value());
    // 回绕后的新帧被接受 (相对上一帧前进), 且不因为时间轴而拒绝.
    CHECK(input.accept(*sample, Tick{0x00000011u}));
    CHECK(input.snapshot(Tick{0x00000011u}).has_value());
}

void layer_contracts() {
    Rig rig;
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 0}, Tick{1000}));
    CHECK(rig.step(1000).layer == Ch4Layer::main_gimbal);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 400}, Tick{1001}));
    CHECK(rig.step(1001).layer == Ch4Layer::fire_yaw);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, -400}, Tick{1002}));
    CHECK(rig.step(1002).layer == Ch4Layer::launcher);
    // 中位附近抖动: 进入高侧需要越过滞回带, 回到中位立即生效.
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 310}, Tick{1003}));
    CHECK(rig.step(1003).layer == Ch4Layer::launcher); // 仍在滞回带内
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 360}, Tick{1004}));
    CHECK(rig.step(1004).layer == Ch4Layer::fire_yaw);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 320}, Tick{1005}));
    CHECK(rig.step(1005).layer == Ch4Layer::fire_yaw); // 滞回带内保持
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 290}, Tick{1006}));
    CHECK(rig.step(1006).layer == Ch4Layer::main_gimbal);
    CHECK(rig.mapper.diagnostics().layer_changes >= 5);
}

void purpose_contracts() {
    Rig rig;
    // 本地来源, 普通运行档, 主云台层.
    // 首次进入角度用途也要先回中: 上电时摇杆仍在偏转不得直接接管 (C06).
    // 回中周期本身只把目标钉在当前姿态 (不回中周期不给目标).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    auto command = rig.step(998, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    command = rig.step(999, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value) < 1e-6f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    // 右 X 在主云台层给主 yaw 方向目标 (从有效实际姿态 0 起按 1 rad/s × 0.1 s 积分).
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value - 0.05f) < 1e-4f);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());

    // 原 yaw 偏转直接切到发射机构层: 不得立即开摩擦轮.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.layer == Ch4Layer::launcher);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    CHECK(rig.mapper.diagnostics().recentre_required >= 1);
    // 回中一次后接管: 该周期仍不开轮.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(!command.friction_on);
    // 再正向偏转才开轮.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.friction_on);
    CHECK(command.targets.domains[index(Domain::friction_left)].has_value());
    CHECK(command.targets.domains[index(Domain::friction_right)].has_value());
    // 中位保持 (不锁存停止): 摩擦意图保留.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.friction_on);
    // 负向关轮.
    rig.push(FrameSpec{{stick(-0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());

    // 原 pitch 偏转切成拨弹: 切层那一刻右 Y 仍在偏转, 不得立即送弹.
    // 先回到主云台层, 回中一次接管 pitch 用途, 再偏转建立 pitch 角度目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1006});
    command = rig.step(1006, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1007});
    command = rig.step(1007, 0.1f);
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->value > 0.0f);
    // 直接切到发射机构层且右 Y 不松: 不得送弹.
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1008});
    command = rig.step(1008, 0.1f);
    CHECK(command.layer == Ch4Layer::launcher);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 切层只改变"当前操纵哪个通道": 已保持的 pitch 角度目标必须仍在快照里,
    // 否则完整快照会把缺省域当作撤销, "没操纵的轴保持"会变成"被撤输出".
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->value > 0.0f);
    // 回中一次后才接管: 该周期仍不送弹.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1009});
    command = rig.step(1009, 0.1f);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 再正向偏转才送弹.
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1010});
    command = rig.step(1010, 0.1f);
    CHECK(command.targets.domains[index(Domain::trigger)].has_value());
    CHECK(command.targets.domains[index(Domain::trigger)]->value > 0.0f);
    // 回中立即取消拨弹 (不锁存).
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1011});
    command = rig.step(1011, 0.1f);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 负向人工反转.
    rig.push(FrameSpec{{0, stick(-0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1012});
    command = rig.step(1012, 0.1f);
    CHECK(command.targets.domains[index(Domain::trigger)].has_value());
    CHECK(command.targets.domains[index(Domain::trigger)]->value < 0.0f);
}

void gear_and_hold_contracts() {
    Rig rig;
    // 主云台层: 先回中一次接管角度用途 (首次进入同样要求回中), 再偏转建立主 yaw 目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    rig.step(999, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    auto command = rig.step(1000, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    const float held = command.targets.domains[index(Domain::main_yaw)]->value;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    // 右拨杆上 = 本地底盘旋转操作: 右 X 改给 wz, 不再给 yaw 方向目标.
    // 该通道换用途同样先回中: 回中周期只解除等待, wz 仍为 0.
    rig.push(FrameSpec{{stick(0.5f), 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::up},
        Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(command.chassis_active);
    CHECK(std::fabs(command.targets.chassis->wz_rad_s) < 1e-6f);
    rig.push(FrameSpec{{0, 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up, Dr16Switch::up},
        Tick{1003});
    command = rig.step(1003, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::up},
        Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(command.chassis_active);
    CHECK(std::fabs(command.targets.chassis->wz_rad_s - 1.0f) < 1e-4f);
    CHECK(std::fabs(command.targets.chassis->vx_mps - (-0.5f)) < 1e-4f);
    CHECK(std::fabs(command.targets.chassis->vy_mps - 0.25f) < 1e-4f);
    // 已保持的主 yaw 角度目标仍在快照里 (旋转档占用右 X 不等于撤销该轴).
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    // 旋转档下切到发射机构层也不开摩擦 (右 X 已被旋转档占用).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::up}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(!command.friction_on);
}

void source_and_loss_contracts() {
    Rig rig;
    // 建立摩擦轮开启意图.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    rig.step(1000, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    auto command = rig.step(1001, 0.1f);
    CHECK(command.friction_on);
    // 跨层保留: 切到主云台层仍然是开轮.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    CHECK(command.friction_on);
    CHECK(command.targets.domains[index(Domain::friction_left)].has_value());
    // 换源 (左拨杆中) 清掉人工摩擦意图, 且不给本地目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.intent == SourceIntent::standby);
    CHECK(!command.friction_on);
    CHECK(!command.chassis_active);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 回到本地来源: 摩擦不自动恢复; 改用途后的摩擦开关通道同样需要先回中一次.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 失联: 清除摩擦/拨弹意图, 目标时效由输入层判定.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(command.friction_on);
    command = rig.step(1200, 0.1f); // 超过 remote_ttl
    CHECK(rig.mapper.diagnostics().link_lost);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 人工停止: 右拨杆下锁存, 不给任何本地目标; 拨回运行档后恢复资格.
    rig.push(FrameSpec{{stick(0.5f), stick(0.5f), stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::down},
        Tick{1300});
    command = rig.step(1300, 0.1f);
    CHECK(command.manual_stop);
    CHECK(!command.chassis_active);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1301});
    command = rig.step(1301, 0.1f);
    CHECK(!command.manual_stop);
    CHECK(command.chassis_active);
}

// 全链路: 原始 18 B 帧 → 映射 → 应用采用目标.
struct Chain {
    Rig remote{};
    InfantryApp app{ec::infantry::app::synthetic_config(), ConfigAudience::offline_test};
    ec::infantry::app::AppInputs inputs{};
    Chain() { inputs.link.locally_ready = true; }
    void step(std::uint32_t ms, const FrameSpec& spec, float dt_s = 0.01f) {
        remote.push(spec, Tick{ms});
        const auto command = remote.step(ms, dt_s);
        inputs.epochs = ec::infantry::app::Epochs{1, 0};
        inputs.link.remote_online = true;
        inputs.link.remote_sampled_at = remote.now;
        inputs.link.switch_left = spec.switch_left;
        inputs.link.switch_right = spec.switch_right;
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = true;
        inputs.aux.launch_sampled_at = Tick{ms};
        inputs.aux.launch_ttl = ec::Lifetime{1000};
        for (std::size_t i = 0; i < ec::infantry::app::domain_count; ++i) {
            inputs.feedback[i].position_rad = remote.actual[i]
                ? ec::Sample<float>{*remote.actual[i], Tick{ms}, ec::Validity::valid}
                : ec::Sample<float>{};
        }
        app.update(Tick{ms}, inputs);
        if (command.intent == SourceIntent::local && !command.manual_stop) {
            CHECK(app.accept_targets(command.targets, Tick{ms}) == AcceptResult::accepted);
        }
        app.update(Tick{ms}, inputs);
    }
};

void chain_contracts() {
    Chain chain;
    // 主云台层: 首次进入角度用途先回中 (上电时摇杆偏转不得直接接管), 之后右 X 才给主 yaw 目标.
    chain.step(999, FrameSpec{{0, 0, stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                   Dr16Switch::mid});
    chain.step(1000, FrameSpec{{stick(0.5f), 0, stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                   Dr16Switch::mid});
    const auto twist = chain.app.requested_twist();
    CHECK(twist.has_value());
    CHECK(std::fabs(twist->vx_mps - 0.5f) < 1e-4f);
    CHECK(twist->wz_rad_s == 0.0f);
    CHECK(chain.app.target(Domain::main_yaw).has_value());
    CHECK(chain.app.snapshot().permission.source == ec::infantry::app::Source::remote);
    CHECK(!chain.app.snapshot().permission.manual_stop);
    // 发射机构层: 摩擦开关与拨弹经应用采用, 目标值是配置转速.
    chain.step(1001, FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1002, FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1003, FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    CHECK(chain.app.target(Domain::friction_left).has_value());
    CHECK(chain.app.target(Domain::friction_right).has_value());
    chain.step(1004, FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1005, FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1006, FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    CHECK(chain.app.target(Domain::trigger).has_value());
    CHECK(chain.app.target(Domain::trigger)->value > 0.0f);
    // 右拨杆下: 人工停止经应用生效, 不再有本地目标.
    chain.step(1007, FrameSpec{{0, stick(0.8f), stick(0.8f), stick(0.8f)}, -400, Dr16Switch::up,
                   Dr16Switch::down});
    CHECK(chain.app.snapshot().permission.manual_stop);
    CHECK(!chain.app.snapshot().permission.authority);
}
} // 测试辅助

// 未标定的映射量程 (全 0) 不得产生"有效零目标": 零值不等于已标定的零运动意图.
void unconfigured_mapping_contracts() {
    RemoteMappingConfig empty{};
    empty.stick_deadzone = 0.05f;
    RemoteMapper mapper{empty, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.0f;
    actual[index(Domain::fire_yaw)] = 0.0f;
    actual[index(Domain::pitch)] = 0.0f;
    Dr16Snapshot neutral{};
    neutral.switch_left = Dr16Switch::up;
    neutral.switch_right = Dr16Switch::mid;
    const auto idle = mapper.step(neutral, Tick{1000}, 0.1f, actual);
    CHECK(idle.intent == SourceIntent::local);
    CHECK(!idle.chassis_active);
    CHECK(!idle.friction_on);
    for (std::size_t i = 0; i < ec::infantry::app::domain_count; ++i) {
        CHECK(!idle.targets.domains[i].has_value());
    }
    // 即使摇杆偏转也不产生目标 (未标定量程).
    Dr16Snapshot moved_sample{};
    moved_sample.switch_left = Dr16Switch::up;
    moved_sample.switch_right = Dr16Switch::mid;
    moved_sample.axis = {0.5f, 0.5f, 0.5f, 0.5f};
    const auto moved = mapper.step(moved_sample, Tick{1001}, 0.1f, actual);
    CHECK(!moved.chassis_active);
    CHECK(!moved.targets.chassis.has_value());
    CHECK(!moved.friction_on);
    for (std::size_t i = 0; i < ec::infantry::app::domain_count; ++i) {
        CHECK(!moved.targets.domains[i].has_value());
    }
}

// 切回一个已保持角度的用途时: "先回中"只拦新操纵增量, 不取消已有保持角.
void held_angle_on_purpose_return_contracts() {
    Rig rig;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    static_cast<void>(rig.step(998, 0.1f));
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    auto command = rig.step(999, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    const float held = command.targets.domains[index(Domain::main_yaw)]->value;
    // 切到 fire_yaw 层: 主云台保持角仍在快照里.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 400, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(command.layer == Ch4Layer::fire_yaw);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    // x 未回中时切回主云台层: 新增量被回中门拦住, 已有保持角不得消失.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    }
    // 回中周期只解除回中门, 保持不变.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    }
    // 回中后再次偏转才接受新增量 (从保持角继续积分).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value > held);
    }
}

// 参考语义改变 → 丢弃该域的保持角; 无有效实际姿态时本周期不重建.
void drop_held_angle_contracts() {
    Rig rig;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    static_cast<void>(rig.step(998, 0.1f));
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    auto command = rig.step(999, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    rig.mapper.drop_held_angle(Domain::main_yaw);
    rig.actual[index(Domain::main_yaw)] = std::nullopt; // 新参考下实际姿态不可用 → 不能重建
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
}

void infantry_switch_contracts() {
    auto config = mapping();
    config.infantry_controls = true;
    config.main_yaw_joint_frame = true;
    RemoteMapper mapper{config, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.0f;
    actual[index(Domain::pitch)] = 0.0f;
    Dr16Snapshot sample{};
    sample.switch_left = Dr16Switch::up;
    sample.switch_right = Dr16Switch::up;
    sample.ch4 = -400; // 不论 ch4 处于哨兵哪个层级, 都不启用发射机构.
    static_cast<void>(mapper.step(sample, Tick{1000}, 0.1f, actual));
    sample.axis = {0.5f, 0.5f, 0.25f, 0.5f};
    auto command = mapper.step(sample, Tick{1001}, 0.1f, actual);
    CHECK(!command.manual_stop && !command.rotation_gear);
    CHECK(command.targets.chassis && command.targets.chassis->wz_rad_s == 0.0f);
    CHECK(command.targets.domains[index(Domain::main_yaw)]);
    CHECK(command.targets.domains[index(Domain::pitch)]);
    CHECK(!command.friction_on && !command.targets.domains[index(Domain::trigger)]);

    sample.switch_right = Dr16Switch::mid;
    command = mapper.step(sample, Tick{1002}, 0.1f, actual);
    CHECK(command.manual_stop && !command.targets.chassis);
    for (const auto& target : command.targets.domains) { CHECK(!target); }
    CHECK(!command.friction_on);

    sample.axis = {};
    sample.switch_right = Dr16Switch::down;
    static_cast<void>(mapper.step(sample, Tick{1003}, 0.1f, actual));
    sample.axis = {0.5f, 0.5f, 0.25f, 0.5f};
    command = mapper.step(sample, Tick{1004}, 0.1f, actual);
    CHECK(!command.manual_stop && command.rotation_gear);
    CHECK(command.targets.chassis && command.targets.chassis->wz_rad_s == -2.0f);
    CHECK(command.targets.domains[index(Domain::main_yaw)]);
    CHECK(command.targets.domains[index(Domain::pitch)]);
    CHECK(!command.friction_on && !command.targets.domains[index(Domain::trigger)]);
}

// 步兵Pitch位置目标: 当前位置接管、单次方向反转、回中保持、STOP/失联重新捕获.
void infantry_pitch_position_contracts() {
    constexpr float rate = 46.2f * std::numbers::pi_v<float> / 180.0f;
    auto config = mapping();
    config.infantry_controls = true;
    config.main_yaw_joint_frame = true;
    config.pitch_rate_max_rad_s = rate;
    config.yaw_stick_sign = -1;
    config.pitch_stick_sign = -1;
    RemoteMapper mapper{config, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.2f;
    actual[index(Domain::pitch)] = 1.2f;
    auto sample = Dr16Snapshot{};
    sample.switch_left = Dr16Switch::up;
    sample.switch_right = Dr16Switch::up;
    sample.raw.ch1 = 660;
    sample.axis[1] = 1.0f;
    sample.sampled_at = Tick{1000};
    auto command = mapper.step(sample, Tick{1000}, 0.01f, actual);
    CHECK(!command.targets.domains[index(Domain::pitch)]); // 未回中不接管
    sample.raw.ch1 = 0;
    sample.axis[1] = 0.0f;
    sample.sampled_at = Tick{1001};
    command = mapper.step(sample, Tick{1001}, 0.01f, actual);
    const auto target = [&](const RemoteCommand& c) {
        const auto t = c.targets.domains[index(Domain::pitch)];
        CHECK(t.has_value());
        CHECK(t && t->mode == ControlMode::position);
        return t ? t->value : -99.0f;
    };
    CHECK(std::fabs(target(command) - 1.2f) < 1e-6f); // 不跳到电机编码零点
    sample.raw.ch1 = 660;
    sample.axis[1] = 1.0f;
    sample.axis[0] = 1.0f;
    sample.sampled_at = Tick{1002};
    command = mapper.step(sample, Tick{1002}, 0.01f, actual);
    const float held = 1.2f - rate * 0.01f;
    CHECK(std::fabs(target(command) - held) < 1e-6f);
    const auto yaw = command.targets.domains[index(Domain::main_yaw)];
    CHECK(yaw && yaw->value < 0.2f); // X/Y均只反向一次
    sample.raw.ch1 = 0;
    sample.axis[1] = 0.0f;
    sample.axis[0] = 0.0f;
    actual[index(Domain::pitch)] = 1.5f;
    sample.sampled_at = Tick{1003};
    command = mapper.step(sample, Tick{1003}, 0.01f, actual);
    CHECK(std::fabs(target(command) - held) < 1e-6f); // 实测偏移不拖着目标走
    sample.switch_right = Dr16Switch::mid;
    command = mapper.step(sample, Tick{1004}, 0.01f, actual);
    CHECK(command.manual_stop && !command.targets.domains[index(Domain::pitch)]);
    sample.switch_right = Dr16Switch::up;
    command = mapper.step(sample, Tick{1005}, 0.01f, actual);
    CHECK(std::fabs(target(command) - 1.5f) < 1e-6f); // STOP后按新实测接管
    command = mapper.step(std::nullopt, Tick{1200}, 0.01f, actual);
    CHECK(!command.targets.domains[index(Domain::pitch)]);
    actual[index(Domain::pitch)] = 0.7f;
    command = mapper.step(sample, Tick{1201}, 0.01f, actual);
    CHECK(std::fabs(target(command) - 0.7f) < 1e-6f);
    auto disabled = config;
    disabled.pitch_rate_max_rad_s = 0.0f;
    RemoteMapper bare{disabled, ec::Lifetime{100}};
    command = bare.step(sample, Tick{1300}, 0.01f, actual);
    CHECK(!command.targets.domains[index(Domain::pitch)]);
    // 缺位置反馈时不能伪造启动位置.
    RemoteMapper missing{config, ec::Lifetime{100}};
    actual[index(Domain::pitch)].reset();
    command = missing.step(sample, Tick{1400}, 0.01f, actual);
    CHECK(!command.targets.domains[index(Domain::pitch)]);
}
// 非步兵构型仍是位置语义: 右 Y 按角速率积分成角度目标, 不接受"rad/count 当速度".
void non_infantry_pitch_stays_position() {
    auto config = mapping();
    config.infantry_controls = false;
    config.pitch_rate_max_rad_s = 1.0f;
    RemoteMapper mapper{config, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.0f;
    actual[index(Domain::pitch)] = 0.25f;
    Dr16Snapshot sample{};
    sample.switch_left = Dr16Switch::up;
    sample.switch_right = Dr16Switch::up; // 非步兵: 上挡才是运行
    sample.ch4 = 0;
    // 先回中, 建立角度用途与保持角.
    auto command = mapper.step(sample, Tick{1000}, 0.1f, actual);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->mode == ControlMode::position);
    CHECK(std::fabs(command.targets.domains[index(Domain::pitch)]->value - 0.25f) < 1e-6f);
    // 右 Y 满杆: 1 rad/s × 0.1 s 积分, 仍是位置目标.
    sample.raw.ch1 = 660;
    sample.axis[1] = 1.0f;
    command = mapper.step(sample, Tick{1001}, 0.1f, actual);
    const auto target = command.targets.domains[index(Domain::pitch)];
    CHECK(target.has_value());
    if (target) {
        CHECK(target->mode == ControlMode::position);
        CHECK(std::fabs(target->value - 0.35f) < 1e-5f);
    }
    // 回中保持目标角, 不产生零速度目标.
    sample.raw.ch1 = 0;
    sample.axis[1] = 0.0f;
    command = mapper.step(sample, Tick{1002}, 0.1f, actual);
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->mode == ControlMode::position);
    CHECK(std::fabs(command.targets.domains[index(Domain::pitch)]->value - 0.35f) < 1e-5f);
}

// 步兵键鼠: 左拨杆中挡=本地键鼠, 鼠标帧增量只消费一次, WASD 给底盘平移, 不产生发射目标.
void infantry_keyboard_mouse_contracts() {
    auto config = mapping();
    config.infantry_controls = true;
    config.main_yaw_joint_frame = true;
    config.mouse_yaw_rad_per_count = 0.001f;
    config.mouse_pitch_rad_per_count = 0.001f;
    config.keyboard_chassis_speed_mps = 0.5f;
    RemoteMapper mapper{config, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::infantry::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.1f;
    actual[index(Domain::pitch)] = 0.2f;

    const auto make = [](Dr16Switch left, Dr16Switch right, Tick at) {
        Dr16Snapshot sample{};
        sample.switch_left = left;
        sample.switch_right = right;
        sample.sampled_at = at;
        return sample;
    };

    // 左拨杆中挡: 步兵是本地来源 (哨兵才是待机).
    auto sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{1000});
    auto command = mapper.step(sample, Tick{1000}, 0.001f, actual);
    CHECK(command.intent == SourceIntent::local);
    CHECK(!command.manual_stop);
    CHECK(!command.rotation_gear);
    // 首帧鼠标计数为 0: 首次接管取有效实际姿态作为目标角.
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value - 0.1f) < 1e-6f);
    // Pitch与Yaw都是位置目标, 首次接管捕获实际电机位置.
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::pitch)]->value - 0.2f) < 1e-6f);

    // 新样本: 鼠标X/Y分别叠加位置增量, 每帧只消费一次.
    sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{1001});
    sample.raw.mouse_x = 100;
    sample.raw.mouse_y = -50;
    command = mapper.step(sample, Tick{1001}, 0.001f, actual);
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value - 0.2f) < 1e-5f);
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::pitch)]->value - 0.15f) < 1e-5f);
    const float yaw_after_one = command.targets.domains[index(Domain::main_yaw)]->value;

    // 同一份快照被控制循环重复读取 (同一采样时间戳): 不得再次积分鼠标增量.
    command = mapper.step(sample, Tick{1001}, 0.001f, actual);
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == yaw_after_one);
    command = mapper.step(sample, Tick{1001}, 0.001f, actual);
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == yaw_after_one);

    // 新样本且鼠标计数为 0: 只保持目标角.
    sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{1002});
    command = mapper.step(sample, Tick{1002}, 0.001f, actual);
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == yaw_after_one);

    // WASD 给机体平移, 与鼠标角度同时存在; 键鼠档不再叠加右摇杆.
    sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{1003});
    sample.raw.mouse_x = 10;
    sample.keys = 0x0009; // W + D
    sample.axis = {0.9f, 0.9f, 0.9f, 0.9f};
    command = mapper.step(sample, Tick{1003}, 0.001f, actual);
    CHECK(command.chassis_active);
    CHECK(command.targets.chassis.has_value());
    if (command.targets.chassis) {
        CHECK(std::fabs(command.targets.chassis->vx_mps - 0.5f) < 1e-6f);
        CHECK(std::fabs(command.targets.chassis->vy_mps + 0.5f) < 1e-6f);
        CHECK(command.targets.chassis->wz_rad_s == 0.0f);
    }
    // 右摇杆轴值 0.9 未参与 yaw/pitch: 只增加鼠标的 10 count.
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value - (yaw_after_one + 0.01f)) < 1e-5f);
    // 键鼠不产生摩擦轮/拨弹 (用户明确关闭发射映射).
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());

    // 右下 = 小陀螺: 键鼠档给固定 wz, 仍保持鼠标角度.
    sample = make(Dr16Switch::mid, Dr16Switch::down, Tick{1004});
    sample.raw.mouse_x = 5;
    command = mapper.step(sample, Tick{1004}, 0.001f, actual);
    CHECK(command.rotation_gear && command.chassis_active);
    CHECK(command.targets.chassis->wz_rad_s == -2.0f);
    CHECK(std::fabs(command.targets.chassis->vx_mps) < 1e-6f);

    // 切回左拨杆上挡 (摇杆): 摇杆此刻已偏转, 仍须先回中一次才接管角度.
    const float yaw_mouse_hold = command.targets.domains[index(Domain::main_yaw)]->value;
    sample = make(Dr16Switch::up, Dr16Switch::up, Tick{1005});
    sample.axis = {0.5f, 0.0f, 0.0f, 0.0f};
    command = mapper.step(sample, Tick{1005}, 0.1f, actual);
    // 保持角仍在快照里 (完整快照缺省域即撤销), 但不接受摇杆增量.
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == yaw_mouse_hold);
    const float yaw_hold = command.targets.domains[index(Domain::main_yaw)]->value;
    // 回中周期只解除等待, 保持角不变.
    sample = make(Dr16Switch::up, Dr16Switch::up, Tick{1006});
    sample.axis = {0.0f, 0.0f, 0.0f, 0.0f};
    command = mapper.step(sample, Tick{1006}, 0.1f, actual);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == yaw_hold);
    // 回中后再次偏转才按摇杆速率积分.
    sample = make(Dr16Switch::up, Dr16Switch::up, Tick{1007});
    sample.axis = {0.5f, 0.0f, 0.0f, 0.0f};
    command = mapper.step(sample, Tick{1007}, 0.1f, actual);
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value > yaw_hold);

    // 回到左中键鼠档后, 右中 = 人工停止: 不给任何本地目标.
    sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{1008});
    static_cast<void>(mapper.step(sample, Tick{1008}, 0.001f, actual));
    sample = make(Dr16Switch::mid, Dr16Switch::mid, Tick{1009});
    command = mapper.step(sample, Tick{1009}, 0.001f, actual);
    CHECK(command.manual_stop);
    CHECK(!command.chassis_active);
    CHECK(!command.friction_on);
    for (const auto& target : command.targets.domains) { CHECK(!target); }

    // 未标定键鼠量程 (rad/count 与速度全 0): 不生成"有效零目标".
    RemoteMappingConfig empty = config;
    empty.mouse_yaw_rad_per_count = 0.0f;
    empty.mouse_pitch_rad_per_count = 0.0f;
    empty.keyboard_chassis_speed_mps = 0.0f;
    empty.chassis_wz_max_rad_s = 0.0f;
    empty.chassis_vx_max_mps = 0.0f;
    empty.chassis_vy_max_mps = 0.0f;
    RemoteMapper bare{empty, ec::Lifetime{100}};
    auto bare_sample = make(Dr16Switch::mid, Dr16Switch::up, Tick{2000});
    bare_sample.raw.mouse_x = 100;
    bare_sample.keys = 0x0005; // W + A
    const auto bare_command = bare.step(bare_sample, Tick{2000}, 0.001f, actual);
    CHECK(!bare_command.chassis_active);
    CHECK(!bare_command.targets.chassis.has_value());
    CHECK(!bare_command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(!bare_command.targets.domains[index(Domain::pitch)].has_value());

    // 原始 18 B 帧链路: 鼠标/按键字段确实来自 DR16 帧.
    Dr16Stream stream;
    const auto frame = encode_frame(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::up,
        120, -60, 0x0001});
    stream.push({frame.data(), 3});
    CHECK(!stream.on_idle().has_value());
    stream.push(frame);
    const auto raw_sample = stream.on_idle();
    CHECK(raw_sample.has_value());
    CHECK(raw_sample->mouse_x == 120 && raw_sample->mouse_y == -60);
    CHECK(raw_sample->keys == 0x0001);
    CHECK(raw_sample->switch_left == Dr16Switch::mid);
}

int main() {
    infantry_switch_contracts();
    infantry_pitch_position_contracts();
    non_infantry_pitch_stays_position();
    infantry_keyboard_mouse_contracts();
    raw_frame_contracts();
    layer_contracts();
    purpose_contracts();
    unconfigured_mapping_contracts();
    held_angle_on_purpose_return_contracts();
    drop_held_angle_contracts();
    gear_and_hold_contracts();
    source_and_loss_contracts();
    chain_contracts();
    std::puts("infantry app remote contracts passed");
    return 0;
}
