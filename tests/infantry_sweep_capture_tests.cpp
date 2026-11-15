// SYNTHETIC 记录器/真实 UART 编码互操作测试; 不访问硬件, 不代表电机采样结果.
#include "infantry_sweep_capture.hpp"
#include "infantry_link.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>
#include <cassert>
#include <bit>
#include <cstdio>
#include <cstring>
#include <utility>

namespace sweep = ec::infantry::app::sweep;
namespace transport = ec::transport;

static std::uint32_t u32(std::span<const std::uint8_t> data, std::size_t offset) {
    return std::uint32_t(data[offset]) | std::uint32_t(data[offset+1]) << 8 |
        std::uint32_t(data[offset+2]) << 16 | std::uint32_t(data[offset+3]) << 24;
}
static auto request(std::uint8_t op, std::uint32_t cursor) {
    return std::array<std::uint8_t, 6>{sweep::schema, op, static_cast<std::uint8_t>(cursor),
        static_cast<std::uint8_t>(cursor >> 8), static_cast<std::uint8_t>(cursor >> 16),
        static_cast<std::uint8_t>(cursor >> 24)};
}
static sweep::Recorder synthetic_recorder() {
    sweep::Recorder recorder;
    // 128 条环形: 推到 134 条才能验证覆盖, 且首尾相距 6 条.
    for (std::uint32_t i = 0; i < 134; ++i) {
        sweep::Sample sample{};
        sample.timestamp_us = i * 1000; sample.reference_epoch = 7;
        sample.target_heading_rad = 3.25f; sample.heading_rad = -0.5f + static_cast<float>(i)*0.001f;
        sample.heading_rate_rad_s = 0.125f;
        sample.dm_position_rad = std::bit_cast<float>(0x3f7e7d00u);
        sample.dm_velocity_rad_s = 2.0f; sample.torque_command_nm = 0.25f;
        sample.torque_feedback_nm = -0.125f; sample.speed_target_rad_s = 1.25f;
        sample.speed_p_nm = 0.2f; sample.speed_i_nm = 0.05f;
        sample.flags = sweep::target_valid | sweep::heading_valid | sweep::dm_feedback_fresh;
        sample.reason = 13; sample.feedback_age_ms = 2;
        recorder.push(sample);
    }
    return recorder;
}
static auto native_golden() {
    const auto recorder = synthetic_recorder();
    std::array<std::uint8_t, 240> body{};
    const auto length = recorder.reply(request(1, 6), body);
    assert(length == 232);
    std::array<std::uint8_t, transport::uart_wire_max> wire{};
    const auto size = transport::encode_uart_frame(sweep::response_type, 0x7e7d,
        {body.data(), length}, wire);
    assert(size > length);
    return std::pair{wire, size};
}
static void recorder_contracts() {
    sweep::Recorder empty;
    std::array<std::uint8_t, 240> buffer{};
    assert(empty.reply(request(0, 0), buffer) == 16);
    assert(buffer[3] == 0 && u32(buffer, 8) == 0);
    const auto recorder = synthetic_recorder();
    assert(recorder.next_sequence() == 134 && recorder.first_sequence() == 6);
    // 满页 3 条: 16 头 + 3*72 = 232 字节.
    assert(recorder.reply(request(1, 0), buffer) == 232);
    assert(buffer[3] == 3 && u32(buffer, 4) == 6 && u32(buffer, 12) == 6);
    assert(u32(buffer, 16) == 6 && u32(buffer, 16+72*2) == 8);
    // 首条样本第 12 字节起是 target_heading_rad; 记录布局 3*u32 之后.
    assert(std::bit_cast<float>(u32(buffer, 16+12)) == 3.25f);
    assert(recorder.reply(request(1, 133), buffer) == 88 && u32(buffer, 16) == 133);
    assert(recorder.reply(request(1, 135), buffer) == 16 && buffer[3] == 0);
    assert(recorder.reply(request(1, 6), std::span{buffer}.first(88)) == 88);
    assert(recorder.reply(request(1, 6), std::span{buffer}.first(16)) == 16);
    auto malformed = request(1, 6); malformed[0] = static_cast<std::uint8_t>(sweep::schema + 1);
    assert(recorder.reply(malformed, buffer) == 0);
    assert(recorder.reply(request(2, 0), buffer) == 0);
    assert(recorder.reply(std::span{malformed}.first(5), buffer) == 0);
    buffer.fill(0xa5);
    assert(recorder.reply(request(0, 0), std::span{buffer}.first(15)) == 0);
    assert(buffer[15] == 0xa5);
    // bit9位于 flags 高字节; 扩展标记不改变 schema2 的72字节布局.
    sweep::Recorder angle;
    sweep::Sample sample{}; sample.flags = sweep::sweep_angle | sweep::sweep_running;
    angle.push(sample);
    assert(angle.reply(request(1, 0), buffer) == 88);
    assert(buffer[16+52] == 0x80 && buffer[16+53] == 0x02);
}
static void wire_and_mux_contracts() {
    const auto [wire, size] = native_golden();
    transport::UartFrameDecoder decoder;
    std::optional<transport::UartFrame> frame;
    for (std::size_t i = 0; i < size; ++i) {
        if (auto decoded = decoder.push(wire[i])) { frame = decoded; }
    }
    assert(frame && frame->type == sweep::response_type && frame->sequence == 0x7e7d);
    assert(frame->size == 232 && frame->payload[3] == 3);
    assert(u32(frame->payload, 4) == 6 && u32(frame->payload, 16) == 6);
    assert(decoder.rejected() == 0);
    auto damaged = wire; damaged[size/2] ^= 1;
    decoder.reset(); bool accepted = false;
    for (std::size_t i = 0; i < size; ++i) { accepted = decoder.push(damaged[i]).has_value() || accepted; }
    assert(!accepted && decoder.rejected() != 0);

    // 实际 ReplyMux 的独立零限频出口可逐周期交付, 不落入默认 50 ms 节流.
    sweep::ReplyQueue data;
    ec::infantry::app::ReplyMux ordinary;
    transport::UartFrame query{}; query.type = sweep::request_type; query.sequence = 10;
    const std::array<std::uint8_t, 2> payload{1, sweep::response_type};
    unsigned sent = 0;
    for (std::uint32_t ms = 100; ms < 104; ++ms) {
        assert(data.enqueue(query.sequence, payload));
        assert(data.service([&](auto) { ++sent; return true; }));
    }
    assert(sent == 4);
    query.type = static_cast<std::uint8_t>(ec::infantry::app::MessageType::hello);
    const std::array<std::uint8_t, 2> ack{3, 0xc0};
    assert(ordinary.enqueue(query, ack));
    query.type = sweep::request_type; assert(data.enqueue(query.sequence, payload));
    bool occupied = false;
    const auto send = [&](auto) { if (occupied) { return false; } occupied = true; return true; };
    assert(ordinary.service(ec::Tick{105}, send));
    assert(!data.service(send));
    occupied = false;
    assert(data.service(send));
    // 同批四个请求不得像状态槽那样互相覆盖, 满队列拒绝第五条.
    for (std::uint16_t n = 0; n < 4; ++n) { assert(data.enqueue(100+n, payload)); }
    assert(!data.enqueue(104, payload));
    for (std::uint16_t n = 0; n < 4; ++n) {
        assert(data.service([&](auto bytes) {
            transport::UartFrameDecoder page_decoder;
            std::optional<transport::UartFrame> decoded;
            for (const auto byte : bytes) { if (auto next = page_decoder.push(byte)) { decoded = next; } }
            assert(decoded && decoded->sequence == 100+n);
            return true;
        }));
    }
    assert(!data.service([](auto) { return true; }));
}
int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--golden") == 0) {
        const auto [wire, size] = native_golden();
        for (std::size_t i = 0; i < size; ++i) { std::printf("%02x", wire[i]); }
        std::puts(""); return 0;
    }
    recorder_contracts(); wire_and_mux_contracts();
    std::puts("PASS: SYNTHETIC high-rate recorder, real CRC framing and reply multiplexing");
}
