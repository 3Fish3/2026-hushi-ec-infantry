// RTT 标定行的定点格式化. 不使用 printf/snprintf: newlib-nano 的浮点格式化会引入
// libc 动态分配依赖, 而正式镜像禁止堆. 这里只做整数与千分位定点, 输出格式固定.
#include "board_rtt.hpp"
#include <initializer_list>

namespace ec::infantry::board::rtt {
namespace {
// 把 value 输出成 "<整数>.<digits 位小数>", 四舍五入后按定点打印.
// 返回写入的字符数; 空间不足时返回 0, 调用方整行丢弃.
std::size_t append_fixed(char* out, std::size_t capacity, std::size_t at,
                         float value, int digits) noexcept {
    if (at + 1 >= capacity) { return 0; }
    if (value != value) { // NaN
        if (at + 4 >= capacity) { return 0; }
        out[at++] = 'n'; out[at++] = 'a'; out[at++] = 'n';
        return at;
    }
    if (value < 0.0f) { out[at++] = '-'; value = -value; }
    std::uint32_t scale = 1;
    for (int i = 0; i < digits; ++i) { scale *= 10u; }
    // 上限保护: 超出量程的数值按截断处理, 不产生溢出.
    const float scaled_f = value * static_cast<float>(scale) + 0.5f;
    const auto scaled = scaled_f > 4290000000.0f
        ? 4290000000u : static_cast<std::uint32_t>(scaled_f);
    const std::uint32_t whole = scaled / scale;
    const std::uint32_t frac = scaled % scale;
    char digits_buf[12];
    std::size_t count = 0;
    std::uint32_t v = whole;
    do { digits_buf[count++] = static_cast<char>('0' + (v % 10u)); v /= 10u; } while (v != 0 && count < sizeof(digits_buf));
    if (at + count + 1 + static_cast<std::size_t>(digits) + 1 >= capacity) { return 0; }
    while (count != 0) { out[at++] = digits_buf[--count]; }
    out[at++] = '.';
    for (int i = digits - 1; i >= 0; --i) {
        const std::uint32_t divisor = [&] {
            std::uint32_t d = 1; for (int k = 0; k < i; ++k) { d *= 10u; } return d;
        }();
        out[at++] = static_cast<char>('0' + ((frac / divisor) % 10u));
    }
    return at;
}

std::size_t append_uint(char* out, std::size_t capacity, std::size_t at,
                        std::uint32_t value) noexcept {
    char buf[12];
    std::size_t count = 0;
    do { buf[count++] = static_cast<char>('0' + (value % 10u)); value /= 10u; } while (value != 0 && count < sizeof(buf));
    if (at + count >= capacity) { return 0; }
    while (count != 0) { out[at++] = buf[--count]; }
    return at;
}

std::size_t append_space(char* out, std::size_t capacity, std::size_t at) noexcept {
    if (at + 1 >= capacity) { return 0; }
    out[at++] = ' ';
    return at;
}
} // 内部辅助

std::size_t format_calibration_line(char* out, std::size_t capacity, std::uint32_t now_ms,
    float joint_rad, float measured_heading_rad, float imu_yaw_rad,
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float roll_rad, float pitch_rad, unsigned calib_valid, std::uint32_t reprime_events,
    float torque_nm, unsigned execute, unsigned reason,
    float target_heading_rad, unsigned target_valid, float heading_rate_rad_s) noexcept {
    if (capacity < 8) { return 0; }
    std::size_t at = 0;
    for (const char c : {'C','A','L',' '}) { out[at++] = c; }
    if ((at = append_uint(out, capacity, at, now_ms)) == 0) { return 0; }
    for (const float value : {joint_rad, measured_heading_rad, imu_yaw_rad}) {
        if ((at = append_space(out, capacity, at)) == 0) { return 0; }
        if ((at = append_fixed(out, capacity, at, value, 5)) == 0) { return 0; }
    }
    for (const float value : {accel_x, accel_y, accel_z}) {
        if ((at = append_space(out, capacity, at)) == 0) { return 0; }
        if ((at = append_fixed(out, capacity, at, value, 4)) == 0) { return 0; }
    }
    for (const float value : {gyro_x, gyro_y, gyro_z}) {
        if ((at = append_space(out, capacity, at)) == 0) { return 0; }
        if ((at = append_fixed(out, capacity, at, value, 5)) == 0) { return 0; }
    }
    for (const float value : {roll_rad, pitch_rad}) {
        if ((at = append_space(out, capacity, at)) == 0) { return 0; }
        if ((at = append_fixed(out, capacity, at, value, 4)) == 0) { return 0; }
    }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_uint(out, capacity, at, calib_valid)) == 0) { return 0; }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_uint(out, capacity, at, reprime_events)) == 0) { return 0; }
    // 稳向验证需要看到实际下发的力矩与执行判定, 否则无法区分"没目标"和"力矩为零".
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_fixed(out, capacity, at, torque_nm, 3)) == 0) { return 0; }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_uint(out, capacity, at, execute)) == 0) { return 0; }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_uint(out, capacity, at, reason)) == 0) { return 0; }
    // 稳向诊断: 目标是否冻结、航向变化率与力矩的符号关系.
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_fixed(out, capacity, at, target_heading_rad, 5)) == 0) { return 0; }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_uint(out, capacity, at, target_valid)) == 0) { return 0; }
    if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    if ((at = append_fixed(out, capacity, at, heading_rate_rad_s, 4)) == 0) { return 0; }
    if (at + 1 >= capacity) { return 0; }
    out[at++] = '\n';
    return at;
}
std::size_t format_gap_line(char* out, std::size_t capacity,
    std::uint32_t last_gap_us, std::uint32_t horizon_us, unsigned gap_flag,
    std::uint32_t dup, std::uint32_t regressed, std::uint32_t too_old,
    std::uint32_t long_gap, std::uint32_t accepted) noexcept {
    if (capacity < 8) { return 0; }
    std::size_t at = 0;
    for (const char c : {'G','A','P',' '}) { out[at++] = c; }
    for (const std::uint32_t value : {last_gap_us, horizon_us}) {
        if ((at = append_uint(out, capacity, at, value)) == 0) { return 0; }
        if ((at = append_space(out, capacity, at)) == 0) { return 0; }
    }
    for (const std::uint32_t value : {static_cast<std::uint32_t>(gap_flag), dup, regressed,
             too_old, long_gap, accepted}) {
        if ((at = append_uint(out, capacity, at, value)) == 0) { return 0; }
        if (at + 1 >= capacity) { return 0; }
        out[at++] = ' ';
    }
    out[at - 1] = '\n';
    return at;
}
} // 命名空间 ec::infantry::board::rtt
