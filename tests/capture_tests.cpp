// 首轮采集的正式 C++ 接收/查询链测试. 不连接真实电调或替代固件行为.
#include "capture.hpp"
#include "raw_record.hpp"
#include <fstream>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>
namespace {
namespace cap = ec::infantry::capture;
void check(bool ok, int line) { if (!ok) { std::fprintf(stderr, "capture check failed at %d\n", line); std::exit(1); } }
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)
struct Tx final : ec::transport::CanTx {
    std::vector<ec::transport::CanFrame> frames;
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& f) noexcept override {
        CHECK(cap::allowed_read(f)); frames.push_back(f); return result;
    }
};
// 步兵 yaw profile 下同样只允许参数/状态读, 任何控制帧在测试发送口直接失败.
struct YawTx final : ec::transport::CanTx {
    const cap::CaptureConfig* config{};
    std::vector<ec::transport::CanFrame> frames;
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& f) noexcept override {
        CHECK(cap::allowed_read(*config, f)); frames.push_back(f); return result;
    }
};
ec::transport::CanFrame yaw_response(std::uint16_t master_id, std::uint8_t rid, std::uint32_t bits) {
    return {master_id, 8, {5, 0, 0x33, rid, static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8),
        static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 24)}};
}
ec::transport::CanFrame response(std::uint8_t rid, std::uint32_t bits) {
    return {cap::dm_master_id, 8, {1, 0, 0x33, rid, static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8),
        static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 24)}};
}
void golden() {
    cap::Collector c;
    c.receive(1, {0x201, 8, {0x1f, 0xff, 0xff, 0x9c, 0x12, 0x34, 42, 0}}, 100, 1000);
    c.receive(1, {0x201, 8, {0, 1, 0, 120, 0xff, 0xfe, 43, 0}}, 101, 169000);
    c.receive(2, {0x207, 8, {0, 123, 0, 5, 0, 0, 255, 0}}, 101, 169000);
    Tx tx; c.poll(tx, 100);
    c.receive(cap::dm_bus, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 104, 0);
    c.poll(tx, 150);
    c.receive(cap::dm_bus, response(0x51, std::bit_cast<std::uint32_t>(-0.5f)), 154, 0);
    c.receive(2, {0x20a, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, 159, 100);
    for (std::uint8_t page : {1, 3, 5, 7, 9}) {
        std::array<std::uint8_t, 256> payload{};
        const auto n = cap::encode_page(c, page, 160, payload);
        std::printf("%u ", page);
        for (std::size_t i = 0; i < n; ++i) { std::printf("%02x", payload[i]); }
        std::puts("");
    }
}
void raw_contracts() {
    namespace raw = ec::infantry::recording;
    // SYNTHETIC 步兵 CAN1/0x005 原始短窗, 只用显式装配槽位, 不套历史地址.
    {
        raw::Recorder yaw;
        std::array<std::uint8_t, 12> query{1, 1, 2, 0};
        raw::put32(query, 4, 321); raw::put32(query, 8, 200000);
        std::array<std::uint8_t, 240> output{};
        CHECK(yaw.request(query, output, {1000}) == 32);
        yaw.can_stream(9, 1, {5, 8, {5, 0, 1, 0, 0, 0, 25, 26}}, {1100}, {1100}, 0);
        yaw.can_stream(10, 1, {5, 8, {}}, {1200}, {1200}, 0);
        CHECK(yaw.size() == 1);
        query[1] = 2; CHECK(yaw.request(query, output, {1300}) == 32);
        query[1] = 3; raw::put32(query, 8, 0);
        CHECK(yaw.request(query, output, {1400}) == 96);
        const auto saved = raw::decode(std::span<const std::uint8_t>{output}.subspan(32, 64));
        CHECK(saved && saved->stream == 9 && saved->data[0] == (1u | (8u << 8) | (5u << 16)));
        CHECK(saved->data[1] == 0x00010005u);
    }
    raw::Recorder recorder;
    std::array<std::uint8_t, 12> request{1, 1, 2, 0};
    raw::put32(request, 4, 123); raw::put32(request, 8, 200000);
    std::array<std::uint8_t, 240> reply{};
    CHECK(recorder.request(request, reply, {0xffffff00u}) == 32 && reply[3] == 1);
    CHECK(recorder.fast_dm());
    recorder.can(1, {0x208, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {0xfffffff0u}, {0xfffffff0u}, 0);
    recorder.can(2, {0x201, 8, {}}, {0xfffffff0u}, {0xfffffff0u}, 0);
    CHECK(recorder.size() == 1);
    recorder.can(1, {0x208, 8, {}}, {100}, {100}, 0); CHECK(recorder.size() == 1);
    CHECK(recorder.request(request, reply, {110}) == 32 && recorder.size() == 1); // 同 run 重传不清窗口
    request[1] = 3;
    CHECK(recorder.request(request, reply, {120}) == 32 && reply[2] == 3); // 活跃记录不可读
    request[1] = 2;
    CHECK(recorder.request(request, reply, {130}) == 32 && reply[3] == 2);
    request[1] = 3; raw::put32(request, 8, 0);
    CHECK(recorder.request(request, reply, {140}) == 96 && reply[9] == 1);
    const auto record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
    CHECK(record && record->run == 123 && record->stream == 4 && record->sampled_us == 0xfffffff0u);
    CHECK(record->data[1] == 0x04030201u && record->data[2] == 0x08070605u);
    auto broken = reply; broken[32] = 9;
    CHECK(!raw::decode(std::span<const std::uint8_t>{broken}.subspan(32, 64)));
    request[1] = 1; request[2] = 1; raw::put32(request, 4, 124); raw::put32(request, 8, 2000000);
    CHECK(recorder.request(request, reply, {1000}) == 32);
    ec::drivers::Bmi088Reading sample;
    sample.times.gyro_valid = sample.times.gyro_updated = true;
    for (unsigned i = 0; i < 600; ++i) {
        sample.times.gyro_estimated = {1000 + 1000*i}; sample.times.gyro = sample.times.gyro_estimated;
        sample.times.gyro_sequence = i;
        recorder.imu(sample, sample.times.gyro);
        recorder.imu(sample, sample.times.gyro); // 重复观察不能增样本
    }
    CHECK(recorder.size() == raw::capacity && recorder.state() == raw::State::full);
    request[1] = 0;
    CHECK(recorder.request(request, reply, {601000}) == 32 && raw::get32(reply, 16) == 512000);
    raw::Recorder pitch;
    request = {1, 1, 2, 0}; raw::put32(request, 4, 125); raw::put32(request, 8, 200000);
    CHECK(pitch.request(request, reply, {0}) == 32);
    pitch.can(1, {0x011, 8, {}}, {1000}, {1000}, 0);
    pitch.can(2, {0x000, 8, {}}, {1000}, {1000}, 0);
    CHECK(pitch.size() == 0);
    pitch.can(2, {0x011, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {1000}, {1000}, 0);
    CHECK(pitch.size() == 1);
    request[1] = 2; CHECK(pitch.request(request, reply, {2000}) == 32);
    request[1] = 3; raw::put32(request, 8, 0);
    CHECK(pitch.request(request, reply, {2000}) == 96);
    const auto pitch_record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
    CHECK(pitch_record && pitch_record->stream == 9 && pitch_record->data[0] == (2u | (8u << 8) | (0x011u << 16)));
    CHECK(pitch_record->data[1] == 0x04030201u && pitch_record->data[2] == 0x08070605u);
    cap::Collector collector; Tx tx;
    for (unsigned i = 0; i < 100; ++i) { collector.poll(tx, i, true); }
    CHECK(tx.frames.size() == 50);
    for (const auto& frame : tx.frames) { CHECK(cap::allowed_read(frame) && frame.data[2] == 0xcc); }
}
// 步兵 yaw profile: CAN1 达妙 yaw (命令/反馈 0x005, node 5), 四轮 CAN1 0x201..0x204, pitch CAN2 0x205.
void yaw_profile() {
    const auto& cfg = cap::infantry_yaw;
    CHECK(cfg.dm_bus == 1 && cfg.dm_can_id == 0x005 && cfg.dm_master_id == 0x005);
    CHECK(cfg.installs[0].kind == cap::Kind::c620 && cfg.installs[0].bus == 1 && cfg.installs[0].rx_id == 0x201);
    CHECK(cfg.installs[3].bus == 1 && cfg.installs[3].rx_id == 0x204 && cfg.installs[3].node == 4);
    CHECK(cfg.installs[6].kind == cap::Kind::c620 && cfg.installs[6].bus == 2 &&
        cfg.installs[6].rx_id == 0x205 && cfg.installs[6].node == 5);
    CHECK(cfg.installs[9].kind == cap::Kind::dm && cfg.installs[9].bus == 1 &&
        cfg.installs[9].rx_id == 0x005 && cfg.installs[9].node == 5);
    // 历史槽 (0x208/0x20a/0x206/0x207) 不再作为本车已装设备.
    for (std::size_t i : {4u, 5u, 7u, 8u}) {
        CHECK(cfg.installs[i].kind == cap::Kind::absent && cfg.installs[i].bus == 0);
    }
    // 读请求与状态请求使用 profile 的命令 ID, 且都只落在 0x7FF 读白名单.
    const auto request = cap::read_request(cfg, 0x50);
    CHECK(request && request->id == 0x7ff && request->size == 4);
    CHECK(request->data[0] == 0x05 && request->data[1] == 0x00 && request->data[2] == 0x33 && request->data[3] == 0x50);
    const auto status = cap::status_request(cfg);
    CHECK(status.id == 0x7ff && status.size == 4);
    CHECK(status.data[0] == 0x05 && status.data[1] == 0x00 && status.data[2] == 0xcc && status.data[3] == 0);
    CHECK(cap::allowed_read(cfg, *request) && cap::allowed_read(cfg, status));
    auto not_read = *request; not_read.data[3] = 0xff; CHECK(!cap::allowed_read(cfg, not_read));
    auto write = *request; write.data[2] = 0x34; CHECK(!cap::allowed_read(cfg, write)); // 写寄存器不是读白名单
    // 使能/失能/清错/置零/保存/任意 MIT 命令帧即使使用本 profile 的 ID 也不在白名单.
    ec::transport::CanFrame action{0x005, 8, {}};
    action.data.fill(0xff); action.data[7] = 0xfc; CHECK(!cap::allowed_read(cfg, action)); // 使能
    action.data[7] = 0xfd; CHECK(!cap::allowed_read(cfg, action)); // 失能
    action.data[7] = 0xfb; CHECK(!cap::allowed_read(cfg, action)); // 清错
    action.data[7] = 0xfe; CHECK(!cap::allowed_read(cfg, action)); // 置零
    action.data[7] = 0xff; CHECK(!cap::allowed_read(cfg, action)); // 保存设置
    ec::transport::CanFrame mit{0x005, 8, {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80}};
    CHECK(!cap::allowed_read(cfg, mit));
    auto saved = status; saved.data[3] = 1; CHECK(!cap::allowed_read(cfg, saved));
    // 两 profile 的读白名单互不通用, 不因沿用同一 0x7FF 就放行另一套命令 ID.
    CHECK(!cap::allowed_read(cfg, cap::status_request()));
    CHECK(!cap::allowed_read(cap::legacy_config, status));
    // legacy 候选表不能被这个 profile 顶替, 两套配置各自独立.
    CHECK(cap::status_request().data[0] == 1 && cap::status_request().data[1] == 0);
    CHECK(cap::legacy_config.installs[9].rx_id == 0x011 && cap::legacy_config.dm_bus == 2);
    cap::Collector c{cfg};
    CHECK(c.config().dm_master_id == 0x005);
    YawTx tx; tx.config = &cfg;
    // 四轮与 pitch 在各自总线、ID、node 上有效; 错误总线或 node 不计入有效.
    for (std::size_t i = 0; i < 7; ++i) {
        if (cfg.installs[i].kind == cap::Kind::absent) { continue; }
        const auto& entry = cfg.installs[i];
        ec::transport::CanFrame f{entry.rx_id, 8, {0, 0, 0, 1, 0, 0, 25, 0}};
        c.receive(entry.bus, f, 10, 0);
        CHECK(c.motors()[i].valid && c.motors()[i].received == 1);
    }
    c.receive(2, {0x201, 8, {0, 0, 0, 1, 0, 0, 25, 0}}, 11, 0); // 四轮只在 CAN1
    CHECK(c.diagnostics().unknown == 1 && c.motors()[0].received == 1);
    // 达妙反馈: CAN1 Master 0x005, 低 4 位 node=5, 有效状态半字节.
    c.receive(1, {0x005, 8, {0x15, 0, 0, 0, 0, 0, 25, 26}}, 20, 0);
    CHECK(c.motors()[9].valid && c.motors()[9].received == 1);
    auto wrong_node = ec::transport::CanFrame{0x005, 8, {0x16, 0, 0, 0, 0, 0, 25, 26}};
    c.receive(1, wrong_node, 21, 0);
    CHECK(c.motors()[9].received == 2 && !c.motors()[9].valid && c.motors()[9].invalid == 1);
    c.receive(2, {0x005, 8, {0x1f, 0, 0, 0, 0, 0, 25, 26}}, 22, 0); // yaw 只在 CAN1
    CHECK(c.diagnostics().unknown == 2 && c.motors()[9].received == 2);
    // 参数响应必须匹配 profile 总线/Master ID/payload 电机 ID/RID 与在途请求.
    c.receive(1, yaw_response(0x005, 0x50, 0), 23, 0); CHECK(c.diagnostics().unsolicited == 1);
    c.poll(tx, 0xfffffff0u);
    CHECK(tx.frames.size() == 1 && tx.frames.back().data[3] == 0x50);
    CHECK(tx.frames.back().data[0] == 0x05 && tx.frames.back().data[1] == 0x00);
    c.receive(1, yaw_response(0x004, 0x50, 0), 1, 0); CHECK(!c.values()[10].seen);
    c.receive(2, yaw_response(0x005, 0x50, 0), 1, 0); CHECK(!c.values()[10].seen); // 错误总线
    // 回显字节被破坏的参数帧与普通 MIT 反馈在协议上不可区分: 只要求它不写入寄存器值,
    // 不把这种帧当作参数响应, 也不据此声称参数有效.
    auto wrong_motor = yaw_response(0x005, 0x50, 0); wrong_motor.data[0] = 6;
    c.receive(1, wrong_motor, 1, 0);
    CHECK(!c.values()[10].seen && !c.motors()[9].valid);
    auto wrong_device = yaw_response(0x005, 0x50, 0); wrong_device.data[1] = 1;
    const auto before_wrong_device = c.motors()[9].received;
    c.receive(1, wrong_device, 1, 0);
    CHECK(!c.values()[10].seen && c.motors()[9].received == before_wrong_device + 1);
    c.receive(1, yaw_response(0x005, 0x51, 0), 1, 0); CHECK(!c.values()[10].seen); // RID 必须匹配在途请求
    // 回显命中但 RID 不在白名单的参数响应只计无效, 不能解成 MIT 位置.
    const auto before_unknown_rid = c.motors()[9].received;
    c.receive(1, yaw_response(0x005, 0x02, 0), 1, 0); CHECK(c.diagnostics().unsolicited == 3);
    CHECK(c.motors()[9].received == before_unknown_rid);
    c.receive(1, yaw_response(0x005, 0x50, std::bit_cast<std::uint32_t>(1.5f)), 1, 0);
    CHECK(c.values()[10].seen && c.values()[10].valid && c.values()[10].received_ms == 1);
    CHECK(c.values()[10].request_ms == 0xfffffff0u);
    // 严格 100 ms: 边界值迟到不算有效, 并计一次超时.
    c.poll(tx, 0x22); CHECK(tx.frames.size() == 2 && tx.frames.back().data[3] == 0x51);
    c.receive(1, yaw_response(0x005, 0x51, 0), 0x22 + 100, 0);
    CHECK(!c.values()[11].seen);
    c.poll(tx, 0x22 + 100); CHECK(c.diagnostics().timeouts == 1);
    // NaN 浮点寄存器标为已见但无效; 不伪报有效角度. 走到固定 RID 0x14 的真实请求路径.
    cap::Collector nan{cfg}; YawTx nan_tx; nan_tx.config = &cfg;
    unsigned nan_at{};
    for (unsigned t = 0; t <= 4000; t += 50) {
        nan.poll(nan_tx, t);
        if (nan_tx.frames.back().data[3] == 0x14) { nan_at = t; break; }
    }
    CHECK(nan_at != 0);
    nan.receive(1, yaw_response(0x005, 0x14,
        std::bit_cast<std::uint32_t>(std::numeric_limits<float>::quiet_NaN())), nan_at + 1, 0);
    CHECK(nan.values()[5].seen && !nan.values()[5].valid);
    // 全程只发送参数/状态读; 长时间轮询也不能出现任何控制或写命令.
    for (unsigned t = 0; t < 10000; t += 10) { c.poll(tx, t); }
    CHECK(!tx.frames.empty());
    for (const auto& f : tx.frames) {
        CHECK(cap::allowed_read(cfg, f));
        CHECK(f.id == 0x7ff && f.size == 4 && f.data[0] == 0x05 && f.data[1] == 0x00);
        CHECK(f.data[2] == 0x33 || (f.data[2] == 0xcc && f.data[3] == 0));
    }
    // 未安装槽位不产生采集记录, 但页结构保持 10 槽协议.
    CHECK(!c.motors()[4].seen && !c.motors()[5].seen && !c.motors()[7].seen && !c.motors()[8].seen);
    std::array<std::uint8_t, 256> page{};
    CHECK(cap::encode_page(c, 3, 0, page) > 0);
    CHECK(page[2] == 2);
    CHECK(cap::encode_page(c, 1, 0, page) == 112);
    // 状态快速轮询同样只发 0xCC 状态读.
    cap::Collector fast{cfg}; YawTx fast_tx; fast_tx.config = &cfg;
    for (unsigned t = 0; t < 100; ++t) { fast.poll(fast_tx, t, true); }
    CHECK(fast_tx.frames.size() == 50);
    for (const auto& f : fast_tx.frames) {
        CHECK(cap::allowed_read(cfg, f) && f.data[2] == 0xcc && f.data[0] == 0x05);
    }
}
void raw_golden() {
    namespace raw = ec::infantry::recording;
    raw::Record record{raw::Kind::gyro, 0, 3, 0, 4, 0xfffffff0u, 20, 1, 600, 9};
    record.data[0] = std::bit_cast<std::uint32_t>(1.25f);
    record.data[1] = std::bit_cast<std::uint32_t>(-0.5f);
    std::array<std::uint8_t, 64> out{};
    CHECK(raw::encode(record, out));
    for (auto byte : out) { std::printf("%02x", byte); }
    std::puts("");
}
int raw_replay(const char* path) {
    namespace raw = ec::infantry::recording;
    std::ifstream file(path, std::ios::binary);
    std::array<char, 8> magic{}; file.read(magic.data(), magic.size());
    CHECK(std::memcmp(magic.data(), "ECRAW1\r\n", 8) == 0);
    std::array<std::uint8_t, 4> length{};
    file.read(reinterpret_cast<char*>(length.data()), length.size());
    const auto size = raw::get32(length, 0); CHECK(file && size <= 1048576);
    file.seekg(size, std::ios::cur);
    std::array<std::uint8_t, 64> wire{};
    unsigned count{};
    while (file.read(reinterpret_cast<char*>(wire.data()), wire.size())) {
        const auto record = raw::decode(wire); CHECK(record);
        ++count;
        std::printf("%u %u %u %.6f\n", record->run, record->sequence, record->sampled_us,
            static_cast<double>(std::bit_cast<float>(record->data[0])));
    }
    CHECK(file.eof() && file.gcount() == 0 && count > 0);
    return 0;
}
} // 测试内部
int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--raw-golden") == 0) { raw_golden(); return 0; }
    if (argc == 3 && std::strcmp(argv[1], "--raw-replay") == 0) { return raw_replay(argv[2]); }
    raw_contracts();
    yaw_profile();
    if (argc == 2 && std::strcmp(argv[1], "--golden") == 0) { golden(); return 0; }
    for (unsigned rid = 0; rid < 256; ++rid) {
        const auto f = cap::read_request(static_cast<std::uint8_t>(rid));
        bool expected = false;
        for (const auto s : cap::registers) { expected |= s.rid == rid; }
        CHECK(static_cast<bool>(f) == expected);
        if (f) {
            CHECK(f->id == 0x7ff && f->size == 4 && f->data[0] == 1 && f->data[1] == 0 && f->data[2] == 0x33);
            CHECK(cap::allowed_read(*f));
            for (unsigned byte = 0; byte < 8; ++byte) {
                auto bad = *f; bad.data[byte] ^= 0x80; CHECK(!cap::allowed_read(bad));
            }
            auto bad = *f; bad.id = 1; CHECK(!cap::allowed_read(bad));
            bad = *f; bad.size = 8; CHECK(!cap::allowed_read(bad));
        }
    }
    const auto status = cap::status_request();
    CHECK(status.id == 0x7ff && status.size == 4 && status.data[2] == 0xcc && status.data[3] == 0);
    CHECK(cap::allowed_read(status));
    auto not_read = status; not_read.data[2] = 0xfc; CHECK(!cap::allowed_read(not_read));
    CHECK(cap::installs[4].bus == 1 && cap::installs[4].rx_id == 0x208 && cap::installs[4].node == 4);
    CHECK(cap::installs[5].bus == 2 && cap::installs[5].rx_id == 0x20a && cap::installs[5].node == 6);
    cap::Collector c;
    for (std::size_t i = 0; i < cap::installs.size(); ++i) {
        const auto entry = cap::installs[i];
        ec::transport::CanFrame f{entry.rx_id, 8, {0, 0, 0, 1, 0, 0, 25, 0}};
        if (entry.kind == cap::Kind::dm) { f.data[0] = 1; }
        c.receive(entry.bus, f, 0xfffffff0u, 0xffffff00u);
        c.receive(entry.bus, f, 1, 0x100u);
        CHECK(c.motors()[i].valid && c.motors()[i].received == 2);
        CHECK(c.motors()[i].intervals == 1 && c.motors()[i].min_cycles == 512);
        c.receive(entry.bus, f, 1001, 0);
        CHECK(c.motors()[i].intervals == 1); // 长断流不伪造跨多次 CYCCNT 回绕差值.
        f.size = 2; c.receive(entry.bus, f, 1002, 1);
        CHECK(!c.motors()[i].valid && c.motors()[i].invalid == 1);
    }
    c.receive(2, {0x201, 8, {}}, 2, 0); CHECK(c.diagnostics().unknown == 1);
    c.receive(1, {0x800, 8, {}}, 2, 0); CHECK(c.diagnostics().malformed == 1);
    // 参数响应必须属于当前成功提交的请求, 参数响应永不流入 MIT 状态槽.
    cap::Collector dm; Tx tx;
    CHECK(cap::dm_bus == 2 && cap::dm_can_id == 1 && cap::dm_master_id == 0x011);
    dm.receive(cap::dm_bus, {0, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 9, 0);
    dm.receive(1, {0x011, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 9, 0);
    CHECK(!dm.motors()[9].seen && dm.diagnostics().unknown == 2);
    dm.receive(cap::dm_bus, response(0x50, 0), 10, 0); CHECK(dm.diagnostics().unsolicited == 1);
    dm.poll(tx, 0xfffffff0u); CHECK(tx.frames.size() == 1 && tx.frames.back().data[3] == 0x50);
    dm.receive(cap::dm_bus, response(0x51, 0), 1, 0); CHECK(!dm.values()[11].seen);
    dm.receive(cap::dm_bus == 1 ? 2 : 1, response(0x50, 0), 1, 0); CHECK(!dm.values()[10].seen);
    auto bad = response(0x50, 0); bad.data[0] = 2; dm.receive(cap::dm_bus, bad, 1, 0); CHECK(!dm.values()[10].seen);
    dm.receive(cap::dm_bus, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 1, 0);
    CHECK(dm.values()[10].seen && dm.values()[10].valid && dm.values()[10].received_ms == 1);
    dm.poll(tx, 0x22); CHECK(tx.frames.size() == 2 && tx.frames.back().data[3] == 0x51);
    dm.receive(cap::dm_bus, response(0x51, std::bit_cast<std::uint32_t>(std::numeric_limits<float>::quiet_NaN())), 0x23, 0);
    CHECK(dm.values()[11].seen && !dm.values()[11].valid);
    CHECK(dm.motors()[9].received == 1); // 仅上面故意错误 node 的普通帧进入 raw 槽.
    dm.poll(tx, 0x54); CHECK(tx.frames.back().data[2] == 0xcc);
    dm.receive(cap::dm_bus, {cap::dm_master_id, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 0x55, 0);
    CHECK(dm.motors()[9].valid && dm.motors()[9].received == 2);
    dm.poll(tx, 0x86); CHECK(tx.frames.back().data[2] == 0x33 && tx.frames.back().data[3] == 7);
    dm.receive(cap::dm_bus, response(7, 0), 0xea, 0); CHECK(!dm.values()[0].seen); // 严格 100 ms 边界.
    dm.poll(tx, 0xea); CHECK(dm.diagnostics().timeouts == 1);
    // busy/error 不伪造发送成功, 调度不会在一个周期补发积压请求.
    cap::Collector busy; Tx fail; fail.result = ec::transport::SendResult::busy;
    busy.poll(fail, 0); busy.poll(fail, 49); CHECK(fail.frames.size() == 1);
    CHECK(busy.diagnostics().queries == 0 && busy.diagnostics().tx_busy == 1);
    fail.result = ec::transport::SendResult::error; busy.poll(fail, 50); CHECK(busy.diagnostics().tx_error == 1);
    busy.receive(cap::dm_bus, response(0x51, 0), 51, 0); CHECK(!busy.values()[11].seen);
    // 单电机台架不等待其它设备; 缺 DM 时的读超时不阻塞 GM6020 原始采集.
    cap::Collector partial; Tx partial_tx;
    for (unsigned t = 0; t < 1000; ++t) {
        partial.receive(1, {0x208, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, t, t * 168000);
        partial.poll(partial_tx, t);
    }
    CHECK(partial.motors()[4].valid && partial.motors()[4].received == 1000);
    CHECK(partial.diagnostics().timeouts > 0);
    for (unsigned i = 0; i < partial.motors().size(); ++i) {
        CHECK(partial.motors()[i].seen == (i == 4));
    }
    for (unsigned t = 1000; t < 2000; ++t) { partial.poll(partial_tx, t); }
    CHECK(partial.motors()[4].last_ms == 999); // 缺帧只保留原采样时间, 不伪造心跳.
    // 多轮发送只能落入读白名单, 不因长期无响应而尝试控制模式.
    for (unsigned t = 0; t < 10000; t += 10) { c.poll(tx, t); }
    for (const auto& f : tx.frames) { CHECK(cap::allowed_read(f)); }
    std::array<std::uint8_t, 256> out{};
    CHECK(cap::encode_page(c, 1, 0, out) == 112);
    CHECK(cap::encode_page(c, 7, 0, out) == 88);
    CHECK(cap::encode_page(c, 1, 0, {out.data(), 111}) == 0);
    CHECK(cap::encode_page(c, 0, 0, out) == 0);
    CHECK(cap::encode_page(c, 10, 0, out) == 0);
    std::puts("capture contracts passed");
}
