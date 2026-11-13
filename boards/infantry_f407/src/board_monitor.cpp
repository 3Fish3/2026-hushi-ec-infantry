#include "board_monitor.h"
#include "board_io.h"
#include "board_capture.hpp"
#include "board_raw.hpp"
#include "board_app.hpp"
#if EC_INFANTRY_CAPTURE
#include "capture.hpp"
#endif
#include "board_bench.hpp"
#include "board_monitor_protocol.hpp"
#include "board_registry.h"
#include "ec/platform/stm32f407.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>
#include <algorithm>

#if EC_S1_UART6_MONITOR
namespace {
// 所有原 USART6 业务共用 USART1 既有队列, 此处不再持有 UART 句柄或启动 DMA.
#if !EC_INFANTRY_APP
ec::transport::UartFrameDecoder decoder;
std::uint32_t generation{};
#endif
std::array<std::uint8_t, ec::transport::uart_wire_max> wire{};
std::array<std::uint8_t, ec::infantry::monitor_payload_size> payload{};
std::uint32_t last_reply{}, unknown{}, dropped{};
#if EC_INFANTRY_APP
std::array<ec::transport::UartFrame, ec::infantry::app_requests_per_cycle> app_requests{};
std::size_t app_request_count{};
#endif
std::array<std::uint32_t, 14> link_stats() {
    std::array<std::uint32_t, 14> result{};
    ec_board_uart1_link_stats(result.data());
#if EC_INFANTRY_APP
    result[11] = ec::infantry::ec_app_observation().frame_rejects;
#else
    result[11] = decoder.rejected();
#endif
    result[12] = unknown; result[13] = dropped;
    return result;
}
}
extern "C" void ec_monitor_init(void) {}
void ec_monitor_request(const ec::transport::UartFrame& frame) noexcept {
#if EC_INFANTRY_APP
    if (app_request_count == app_requests.size()) { ++dropped; return; }
    app_requests[app_request_count++] = frame;
#else
    static_cast<void>(frame);
#endif
}
extern "C" void ec_monitor_poll(void) {
    bool requested = false;
    bool bench_reply = false;
    bool registry_reply = false;
#if EC_S5_GM6020_BENCH
    bool bench_trace_reply = false;
    std::array<std::uint8_t,12> bench_trace_in{};
    std::size_t bench_trace_size{};
#endif
#if EC_INFANTRY_APP
    bool app_fast_reply = false;
#endif
#if EC_INFANTRY_APP || EC_INFANTRY_CAPTURE
    bool raw_reply = false;
    std::array<std::uint8_t, 12> raw_request{};
#endif
#if EC_INFANTRY_CAPTURE
    std::uint8_t capture_page = 255;
#endif
    std::uint16_t registry_first = 0;
    std::uint16_t sequence = 0;
#if !EC_INFANTRY_APP
    std::array<std::uint8_t, 64> chunk{};
#endif
    const auto accept_frame = [&](const ec::transport::UartFrame& value) {
        const auto* frame = &value;
#if EC_S5_GM6020_BENCH
                if (frame->type==0x26) {
                    if (requested) { ++dropped; }
                    requested=true;bench_trace_reply=true;bench_reply=false;registry_reply=false;sequence=frame->sequence;
                    bench_trace_size=frame->size==bench_trace_in.size() ? frame->size : 0;
                    std::copy_n(frame->payload.begin(),bench_trace_size,bench_trace_in.begin());
                    return;
                }
#endif
#if EC_INFANTRY_APP || EC_INFANTRY_CAPTURE
                if (frame->type == ec::infantry::recording::request_type) {
                    if (frame->size != raw_request.size()) { ++unknown; return; }
                    if (requested) { ++dropped; }
                    requested = true; raw_reply = true; sequence = frame->sequence;
#if EC_INFANTRY_APP
                    app_fast_reply = false;
#endif
                    std::copy_n(frame->payload.begin(), raw_request.size(), raw_request.begin());
                    return;
                }
#endif
#if EC_INFANTRY_APP
                if (frame->type == ec::infantry::app_fast_request && frame->size == 0) {
                    if (requested) { ++dropped; }
                    requested = true; app_fast_reply = true; raw_reply = false;
                    sequence = frame->sequence;
                    return;
                }
#endif
                if (frame->type == ec::infantry::monitor_request && frame->size == 0) {
                    // 同周期只保留最后一个只读请求, 不排队积压响应.
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = false; sequence = frame->sequence;
#if EC_INFANTRY_CAPTURE
                    capture_page = 255;
#endif
                } else if (frame->type == ec::infantry::registry_page_request) {
                    // 注册表分页请求: 只读快照, 载荷是起始记录序号.
                    if (frame->size != 2) { ++unknown; return; }
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = true;
                    registry_first = static_cast<std::uint16_t>(frame->payload[0] | (frame->payload[1] << 8));
                    sequence = frame->sequence;
#if EC_INFANTRY_CAPTURE
                    capture_page = 255;
                } else if (frame->type == ec::infantry::capture::request_type) {
                    if (frame->size != 1 || frame->payload[0] >= ec::infantry::capture::page_count) { ++unknown; return; }
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = false;
                    sequence = frame->sequence; capture_page = frame->payload[0];
#endif
                } else if (ec_bench_request(*frame)) {
                    requested = true; bench_reply = true; sequence = frame->sequence;
                } else { ++unknown; return; }
                // 只有已识别的新请求才覆盖此前的响应类别; 无关坏帧不改变已排队的只读请求.
#if EC_S5_GM6020_BENCH
                bench_trace_reply=false;
#endif
#if EC_INFANTRY_APP || EC_INFANTRY_CAPTURE
                raw_reply = false;
#endif
#if EC_INFANTRY_APP
                app_fast_reply = false;
#endif
    };
#if EC_INFANTRY_APP
    // APP 已在控制任务内按类型分流完整帧, 不再用第二个解码器争抢同一串口.
    for (std::size_t i = 0; i < app_request_count; ++i) { accept_frame(app_requests[i]); }
    app_request_count = 0;
#else
    // 非 APP 同样读取唯一 USART1 管线, 无回显, 每周期仍限定 512 字节.
    for (std::size_t budget = 512; budget; ) {
        const auto current = ec_board_uart1_generation();
        if (current != generation) {
            decoder.reset(); requested = false; generation = current;
            ec_bench_transport_lost();
        }
        const auto size = ec_board_uart1_read(chunk.data(), std::min(chunk.size(), budget));
        if (!size) { break; }
        budget -= size;
        for (std::size_t i = 0; i < size; ++i) {
            if (const auto frame = decoder.push(chunk[i])) { accept_frame(*frame); }
        }
    }
#endif
    if (!requested) { return; }
    const auto now = HAL_GetTick();
    // S5 唯一客户端串行请求, 波形下载仍受 DMA 非阻塞出口约束; 不让只读观测的50ms限频吞掉控制ACK.
#if EC_S5_GM6020_BENCH
    if (!bench_reply && !bench_trace_reply)
#endif
    if (now - last_reply < 50) { ++dropped; return; }
    last_reply = now;
#if EC_S5_GM6020_BENCH
    if (bench_trace_reply) {
        const auto size=ec_bench_trace_request({bench_trace_in.data(),bench_trace_size},payload);
        const auto length=size ? ec::transport::encode_uart_frame(0xa1,sequence,{payload.data(),size},wire) : 0;
        if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
        return;
    }
#endif
#if EC_INFANTRY_APP
    if (app_fast_reply) {
        std::size_t size{};
        {
            ec::stm32f407::IrqCriticalSection lock;
            const auto saved = lock.enter();
            const auto& observation = ec::infantry::ec_app_observation();
            static_assert(sizeof(observation.fast_payload) <= payload.size());
            size = observation.fast_size;
            std::copy_n(observation.fast_payload, size, payload.begin());
            lock.exit(saved);
        }
        const auto length = size ? ec::transport::encode_uart_frame(ec::infantry::app_fast_response,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
        return;
    }
#endif
#if EC_INFANTRY_APP || EC_INFANTRY_CAPTURE
    if (raw_reply) {
        const auto size = ec_raw_request(raw_request, payload);
        const auto length = size ? ec::transport::encode_uart_frame(ec::infantry::recording::response_type,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
        return;
    }
#endif
#if EC_INFANTRY_CAPTURE
    if (capture_page != 255) {
        auto size = ec_capture_snapshot(capture_page, payload);
        if (capture_page == 0 && size) {
            ec::infantry::capture::Writer tail({payload.data() + size, payload.size() - size});
            for (const auto value : link_stats()) { tail.u32(value); }
            size = tail.size() ? size + tail.size() : 0;
        }
        const auto length = size ? ec::transport::encode_uart_frame(ec::infantry::capture::response_type,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
        return;
    }
#endif
    if (bench_reply) {
        const auto size=ec_bench_snapshot(payload);
        const auto length=ec::transport::encode_uart_frame(0xa0,sequence,{payload.data(),size},wire);
        if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
        return;
    }
    if (registry_reply) {
        const auto size = ec_registry_encode_page(registry_first, payload.data(), payload.size());
        if (size != 0) {
            const auto length = ec::transport::encode_uart_frame(ec::infantry::registry_page_response,
                sequence, {payload.data(), size}, wire);
            if (!length || !ec_board_uart1_try_send(wire.data(), length)) {
                ++dropped;
            }
        } else {
            // 注册表未启用或序号越界: 不回笼资料, 只计数, 不阻塞控制周期.
            ++dropped;
        }
        return;
    }
    const auto link = link_stats();
    ec::stm32f407::IrqCriticalSection lock;
    const auto saved = lock.enter();
    // 仅固定长度字段复制; 采样时刻在禁止 ISR 更新后读取, 不使用滞后的任务入口时间.
    const auto size = ec::infantry::encode_monitor_snapshot(g_ec_s1, g_ec_io, uwTick, link, payload);
    lock.exit(saved);
    const auto length = ec::transport::encode_uart_frame(ec::infantry::monitor_response,
        sequence, {payload.data(), size}, wire);
    if (!length || !ec_board_uart1_try_send(wire.data(), length)) { ++dropped; }
}
#else
extern "C" void ec_monitor_init(void) {}
extern "C" void ec_monitor_poll(void) {}
void ec_monitor_request(const ec::transport::UartFrame&) noexcept {}
#endif
