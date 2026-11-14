#pragma once
// APP 控制提交后只读记录, 不设置目标、不使能、不修改 PID. 单控制任务持有.
// 复用现有 UART 分帧: 请求 0x37, 应答 0xB7; 缓存覆盖不等于电机停止.
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include "ec/transport/uart_frame.hpp"

namespace ec::infantry::app::sweep {
inline constexpr std::uint8_t request_type = 0x37, response_type = 0xb7, schema = 2;
// 256 条约 18 KiB, 会使镜像 RAM 只剩 3.6 KiB; 250 Hz 下 128 条仍保留 512 ms,
// 足以覆盖主机排空延迟, 因此按 RAM 预算取 128.
inline constexpr std::uint32_t capacity = 128;
inline constexpr std::size_t record_size = 72, response_header_size = 16, max_page_records = 3;

enum Flag : std::uint16_t {
    target_valid = 1u << 0, heading_valid = 1u << 1,
    dm_feedback_fresh = 1u << 2, dm_enabled = 1u << 3,
    torque_submitted = 1u << 4, pid_trace_valid = 1u << 5, executing = 1u << 6,
    sweep_running = 1u << 7, sweep_inhibited = 1u << 8
};
struct Sample {
    std::uint32_t sequence{}, timestamp_us{}, reference_epoch{};
    float target_heading_rad{}, heading_rad{}, heading_rate_rad_s{};
    float dm_position_rad{}, dm_velocity_rad_s{}; // MIT 解码电机角, 不是 p_m/xout 或已验证的多圈角
    float torque_command_nm{}, torque_feedback_nm{}; // 提交命令与设备报告值, 均非独立测力
    float speed_target_rad_s{}, speed_p_nm{}, speed_i_nm{};
    std::uint16_t flags{};
    std::uint8_t reason{}, feedback_age_ms{255}; // 255 表示不可用或超过可表达年龄
    std::uint32_t sweep_run_id{};
    // requested 是引擎当前需求, applied 仅表示该注入分量被 CAN 队列接纳, 都不是实测力矩.
    float sweep_hz{}, requested_excitation_nm{}, applied_excitation_nm{};
};
static_assert(sizeof(Sample) == record_size);
static_assert(response_header_size + max_page_records * record_size <= 240);

class Recorder {
public:
    void push(Sample sample) noexcept {
        sample.sequence = next_;
        samples_[next_ % capacity] = sample;
        ++next_;
    }
    [[nodiscard]] std::uint32_t next_sequence() const noexcept { return next_; }
    [[nodiscard]] std::uint32_t first_sequence() const noexcept {
        return next_ > capacity ? next_ - capacity : 0;
    }
    // 请求: [schema, op=0状态/1分页, cursor:u32]. 应答头: [schema,type,op,count,first,next,earliest].
    // first != 请求 cursor 表示已覆盖或游标不合法; 主机须报告缺口, 不自动跳过.
    [[nodiscard]] std::size_t reply(std::span<const std::uint8_t> request,
        std::span<std::uint8_t> out) const noexcept {
        if (request.size() != 6 || request[0] != schema || request[1] > 1 ||
            out.size() < response_header_size) { return 0; }
        const std::uint32_t cursor = get32(request, 2);
        const std::uint32_t earliest = first_sequence();
        const std::uint32_t start = request[1] == 0 ? next_ : std::clamp(cursor, earliest, next_);
        const std::uint32_t count = request[1] == 0 ? 0 : std::min<std::uint32_t>({next_ - start,
            max_page_records, static_cast<std::uint32_t>((out.size() - response_header_size) / record_size)});
        std::size_t pos = 0;
        put8(out, pos, schema); put8(out, pos, response_type);
        put8(out, pos, request[1]); put8(out, pos, static_cast<std::uint8_t>(count));
        put32(out, pos, start); put32(out, pos, next_); put32(out, pos, earliest);
        for (std::uint32_t n = 0; n < count; ++n) { encode(samples_[(start + n) % capacity], out, pos); }
        return pos;
    }
private:
    static std::uint32_t get32(std::span<const std::uint8_t> in, std::size_t pos) noexcept {
        return std::uint32_t(in[pos]) | std::uint32_t(in[pos+1]) << 8 |
            std::uint32_t(in[pos+2]) << 16 | std::uint32_t(in[pos+3]) << 24;
    }
    static void put8(std::span<std::uint8_t> out, std::size_t& pos, std::uint8_t value) noexcept {
        out[pos++] = value;
    }
    static void put16(std::span<std::uint8_t> out, std::size_t& pos, std::uint16_t value) noexcept {
        put8(out, pos, static_cast<std::uint8_t>(value));
        put8(out, pos, static_cast<std::uint8_t>(value >> 8));
    }
    static void put32(std::span<std::uint8_t> out, std::size_t& pos, std::uint32_t value) noexcept {
        for (unsigned i = 0; i < 4; ++i) { put8(out, pos, static_cast<std::uint8_t>(value >> (8*i))); }
    }
    static void putf(std::span<std::uint8_t> out, std::size_t& pos, float value) noexcept {
        static_assert(sizeof(float) == 4);
        put32(out, pos, std::bit_cast<std::uint32_t>(value));
    }
    static void encode(const Sample& s, std::span<std::uint8_t> out, std::size_t& pos) noexcept {
        put32(out, pos, s.sequence); put32(out, pos, s.timestamp_us); put32(out, pos, s.reference_epoch);
        putf(out, pos, s.target_heading_rad); putf(out, pos, s.heading_rad); putf(out, pos, s.heading_rate_rad_s);
        putf(out, pos, s.dm_position_rad); putf(out, pos, s.dm_velocity_rad_s);
        putf(out, pos, s.torque_command_nm); putf(out, pos, s.torque_feedback_nm);
        putf(out, pos, s.speed_target_rad_s); putf(out, pos, s.speed_p_nm); putf(out, pos, s.speed_i_nm);
        put16(out, pos, s.flags); put8(out, pos, s.reason); put8(out, pos, s.feedback_age_ms);
        put32(out, pos, s.sweep_run_id); putf(out, pos, s.sweep_hz);
        putf(out, pos, s.requested_excitation_nm); putf(out, pos, s.applied_excitation_nm);
    }
    std::array<Sample, capacity> samples_{};
    std::uint32_t next_{};
};
// 分页应答使用固定四槽 FIFO, 不用“同类仅保留最新值”的状态应答语义.
// 支持主机最多四页在途, 避免串行往返延迟低于 1 kHz 的排空速度.
class ReplyQueue {
public:
    bool enqueue(std::uint16_t sequence, std::span<const std::uint8_t> payload) noexcept {
        if (count_ == slots_.size()) { return false; }
        auto& slot = slots_[(first_ + count_) % slots_.size()];
        slot.size = transport::encode_uart_frame(response_type, sequence, payload, slot.wire);
        if (!slot.size) { return false; }
        ++count_;
        return true;
    }
    template<class Send> bool service(Send&& send) noexcept {
        if (!count_) { return false; }
        const auto& slot = slots_[first_];
        if (!send(std::span<const std::uint8_t>{slot.wire.data(), slot.size})) { return false; }
        first_ = (first_ + 1) % slots_.size();
        --count_;
        return true;
    }
private:
    struct Slot { std::array<std::uint8_t, transport::uart_wire_max> wire{}; std::size_t size{}; };
    std::array<Slot, 4> slots_{};
    std::size_t first_{}, count_{};
};
} // 命名空间 ec::infantry::app::sweep
