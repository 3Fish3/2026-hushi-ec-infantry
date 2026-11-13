// RTT 描述符、空满/回绕、读指针消费与 CAL 格式契约.
#include "board_rtt.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
namespace rtt = ec::infantry::board::rtt;
void check(bool ok, int line) {
    if (!ok) { std::fprintf(stderr, "RTT line %d\n", line); std::exit(1); }
}
#define CHECK(x) check(static_cast<bool>(x), __LINE__)
std::string drain() {
    auto& up = rtt::control_block.up[0];
    auto read = up.read_offset;
    const auto write = up.write_offset;
    std::string result;
    while (read != write) {
        result.push_back(up.buffer[read]);
        read = (read + 1u) % up.size;
    }
    up.read_offset = read;
    return result;
}
int main() {
    rtt::init();
    CHECK(std::strcmp(rtt::control_block.id, "SEGGER RTT") == 0);
    CHECK(rtt::control_block.max_up == 1 && rtt::control_block.max_down == 0);
    CHECK(rtt::control_block.up[0].buffer == rtt::up_buffer);
    CHECK(rtt::control_block.up[0].size == 4096);
    CHECK(rtt::write("abc", 3) == 3);
    CHECK(drain() == "abc");
    CHECK(drain().empty()); // 不重复读旧内存.
    const std::string full(4095, 'x');
    CHECK(rtt::write(full.data(), full.size()) == full.size()); // 跨缓冲区尾部.
    const auto write = rtt::control_block.up[0].write_offset;
    CHECK(rtt::write("drop\n", 5) == 0);
    CHECK(rtt::control_block.up[0].write_offset == write);
    CHECK(drain() == full);
    CHECK(rtt::write("fresh\n", 6) == 6);
    CHECK(drain() == "fresh\n");
    std::array<char, 192> line{};
    const auto length = rtt::format_calibration_line(line.data(), line.size(), 1234,
        -1.25f, 0, 0.5f, 1, 2, 9.81f, -0.001f, 0.002f, 0.003f, 0.4f, -0.2f, 1, 7, -0.125f, 1, 3, -1.5f, 1, 0.25f);
    CHECK(length > 0 && length < line.size());
    const std::string text(line.data(), length);
    CHECK(text.starts_with("CAL 1234 -1.25000 0.00000 0.50000 "));
    CHECK(text.ends_with("0.4000 -0.2000 1 7 -0.125 1 3 -1.50000 1 0.2500\n"));
    std::array<char, 16> small{};
    small.fill('#');
    CHECK(rtt::format_calibration_line(small.data(), 8, 1234,
        1,2,3,4,5,6,7,8,9,10,11,1,7,0.0f,0,0,0.0f,0,0.0f) == 0);
    for (std::size_t i = 8; i < small.size(); ++i) { CHECK(small[i] == '#'); }
    std::puts("RTT descriptor/ring/formatter passed");
}
