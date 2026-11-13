#pragma once
#include "ec/transport/can.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::infantry::capture {
// 本车采集契约: 固定候选接线表, 不包含输出限值、PID 或臆测的机构标定.
inline constexpr std::uint8_t request_type = 0x30, response_type = 0xb0, schema = 3;
inline constexpr std::uint8_t page_count = 10;
inline constexpr std::uint16_t dm_can_id = 1, dm_master_id = 0x011; // 候选反馈 ID，待实车确认
inline constexpr std::uint8_t dm_bus = 2;  // 达妙 pitch 挂在 CAN2
static_assert(dm_bus == 1 || dm_bus == 2, "DM must be on CAN1 or CAN2");
inline constexpr std::uint32_t dm_timeout_ms = 100, dm_period_ms = 50;
enum class Kind : std::uint8_t { absent = 0, c620 = 1, c610, gm6020, dm };
struct Install { std::uint8_t bus; Kind kind; std::uint16_t rx_id; std::uint8_t node; };
// 历史候选接线表: 保留给未显式选择 profile 的既有调用与测试.
inline constexpr std::array<Install, 10> installs{{
    {1, Kind::c620, 0x201, 1}, {1, Kind::c620, 0x202, 2},
    {1, Kind::c620, 0x203, 3}, {1, Kind::c620, 0x204, 4},
    {1, Kind::gm6020, 0x208, 4}, {2, Kind::gm6020, 0x20a, 6},
    {2, Kind::c620, 0x205, 5}, {2, Kind::c620, 0x206, 6},
    {2, Kind::c610, 0x207, 7}, {dm_bus, Kind::dm, dm_master_id, 1}}};
// 采集 profile: 10 个固定槽位的装配表 + 达妙参数读取用的总线/命令 ID/反馈 Master ID.
// 未安装槽用 bus 0 + Kind::absent 占位, 保持页数与协议不变; 不把历史槽当作本车已装.
struct CaptureConfig {
    std::array<Install, 10> installs;
    std::uint8_t dm_bus;
    std::uint16_t dm_can_id;
    std::uint16_t dm_master_id;
};
inline constexpr CaptureConfig legacy_config{installs, dm_bus, dm_can_id, dm_master_id};
// 步兵 yaw 只读 profile: CAN1 达妙 yaw, 命令 ID 与反馈 Master ID 同为 0x005, 反馈 node 低 4 位为 5.
// 本车实际只装四轮 CAN1 0x201..0x204、pitch CAN2 0x205 与 yaw; 其余槽位仅为协议占位.
inline constexpr std::array<Install, 10> infantry_yaw_installs{{
    {1, Kind::c620, 0x201, 1}, {1, Kind::c620, 0x202, 2},
    {1, Kind::c620, 0x203, 3}, {1, Kind::c620, 0x204, 4},
    {0, Kind::absent, 0, 0}, {0, Kind::absent, 0, 0},
    {2, Kind::c620, 0x205, 5},
    {0, Kind::absent, 0, 0}, {0, Kind::absent, 0, 0},
    {1, Kind::dm, 0x005, 5}}};
inline constexpr CaptureConfig infantry_yaw{infantry_yaw_installs, 1, 0x005, 0x005};
struct RegisterSpec { std::uint8_t rid; bool floating; };
// 只允许读取这些寄存器. p_m 是转子推导的输出轴角, xout 是输出轴编码器角, 均为 rad.
inline constexpr std::array<RegisterSpec, 12> registers{{
    {0x07, false}, {0x08, false}, {0x09, false}, {0x0a, false},
    {0x0e, false}, {0x14, true}, {0x15, true}, {0x16, true},
    {0x17, true}, {0x24, false}, {0x50, true}, {0x51, true}}};
struct Motor {
    ec::transport::CanFrame raw{};
    std::uint32_t received{}, invalid{}, last_ms{}, cycles{};
    std::uint32_t min_cycles{}, max_cycles{}, intervals{};
    std::uint64_t sum_cycles{};
    bool seen{}, valid{};
};
struct Register {
    std::uint32_t responses{}, received_ms{}, bits{}, request_ms{};
    bool seen{}, valid{};
};
struct Diagnostics {
    std::uint32_t unknown{}, malformed{}, queries{}, tx_busy{}, tx_error{}, timeouts{}, unsolicited{};
    std::uint32_t unknown_bus{}, unknown_id{};
};
// 只编码第 17 页参数读取: STD 0x7FF, DLC=4, CANID_L/CANID_H/0x33/RID.
// 无任意帧发送、写寄存器、使能、清错或置零接口.
[[nodiscard]] std::optional<ec::transport::CanFrame> read_request(std::uint8_t rid) noexcept;
[[nodiscard]] std::optional<ec::transport::CanFrame> read_request(
    const CaptureConfig& config, std::uint8_t rid) noexcept;
// 厂商例程 read_motor_ctrl_fbdata: STD 0x7FF, DLC=4, CANID_L/CANID_H/0xCC/0.
[[nodiscard]] ec::transport::CanFrame status_request() noexcept;
[[nodiscard]] ec::transport::CanFrame status_request(const CaptureConfig& config) noexcept;
[[nodiscard]] bool allowed_read(const ec::transport::CanFrame& frame) noexcept;
[[nodiscard]] bool allowed_read(const CaptureConfig& config, const ec::transport::CanFrame& frame) noexcept;
class Collector {
public:
    // 默认 legacy 候选接线, 保持既有调用行为; 板级可显式传入采集 profile.
    Collector() noexcept : config_(legacy_config) {}
    explicit Collector(const CaptureConfig& config) noexcept : config_(config) {}
    void receive(std::uint8_t bus, const ec::transport::CanFrame& frame,
        std::uint32_t received_ms, std::uint32_t cycles) noexcept;
    void poll(ec::transport::CanTx& tx, std::uint32_t now_ms, bool fast_status = false) noexcept;
    [[nodiscard]] const auto& config() const noexcept { return config_; }
    [[nodiscard]] const auto& motors() const noexcept { return motors_; }
    [[nodiscard]] const auto& values() const noexcept { return values_; }
    [[nodiscard]] const auto& diagnostics() const noexcept { return diagnostics_; }
private:
    CaptureConfig config_;
    std::array<Motor, 10> motors_{};
    std::array<Register, registers.size()> values_{};
    Diagnostics diagnostics_{};
    std::uint32_t last_attempt_{}, requested_at_{};
    std::uint8_t phase_{}, config_index_{}, pending_index_{};
    bool attempted_{}, pending_{}, fast_status_{};
};
// 显式小端序列化, 不发送 C++ 对象布局. 满足容量才返回有效长度.
class Writer {
public:
    explicit Writer(std::span<std::uint8_t> out) noexcept : out_(out) {}
    void u8(std::uint8_t v) noexcept;
    void u16(std::uint16_t v) noexcept;
    void u32(std::uint32_t v) noexcept;
    void u64(std::uint64_t v) noexcept;
    void bytes(std::span<const std::uint8_t> v) noexcept;
    void header(std::uint8_t page, std::uint8_t count, std::uint32_t now) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return failed_ ? 0 : size_; }
private:
    std::span<std::uint8_t> out_;
    std::size_t size_{};
    bool failed_{};
};
void encode_motor(Writer& w, const Install& install, const Motor& motor) noexcept;
void encode_register(Writer& w, RegisterSpec spec, const Register& value) noexcept;
// 1..5: 每页两个电机; 7..9: 每页四个 DM 读寄存器结果. 0/6 由板级补充.
[[nodiscard]] std::size_t encode_page(const Collector& collector, std::uint8_t page,
    std::uint32_t now, std::span<std::uint8_t> out) noexcept;
} // 命名空间 ec::infantry::capture
