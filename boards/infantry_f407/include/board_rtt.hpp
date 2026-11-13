#pragma once
// 单上行、无下行的标准 RTT 控制块. 读端由 OpenOCD 推进 RdOff;
// 写端只有控制任务, 满时整行跳过, 不暂停芯片、不阻塞控制任务.
#include <cstddef>
#include <cstdint>

namespace ec::infantry::board::rtt {
struct UpBuffer {
    const char* name;
    char* buffer;
    std::uint32_t size;
    volatile std::uint32_t write_offset;
    volatile std::uint32_t read_offset;
    std::uint32_t flags;
};
struct ControlBlock {
    char id[16];
    std::int32_t max_up;   // 上行通道数量, 不是缓冲区字节数
    std::int32_t max_down;
    UpBuffer up[1];
};
#if UINTPTR_MAX == UINT32_MAX
static_assert(sizeof(UpBuffer) == 24);
static_assert(offsetof(ControlBlock, up) == 24);
static_assert(offsetof(UpBuffer, write_offset) == 12);
static_assert(offsetof(UpBuffer, read_offset) == 16);
static_assert(sizeof(ControlBlock) == 48);
#endif
inline constexpr std::size_t up_buffer_size = 4096;
extern ControlBlock control_block;
extern char up_buffer[up_buffer_size];
void init() noexcept;
std::size_t write(const char* text, std::size_t size) noexcept;

// 固定格式 CAL 行; 空间不足整行放弃. 不依赖浮点 printf 或堆.
std::size_t format_calibration_line(char* out, std::size_t capacity, std::uint32_t now_ms,
    float joint_rad, float measured_heading_rad, float imu_yaw_rad,
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float roll_rad, float pitch_rad, unsigned calib_valid, std::uint32_t reprime_events,
    float torque_nm, unsigned execute, unsigned reason,
    float target_heading_rad, unsigned target_valid, float heading_rate_rad_s) noexcept;

// 诊断行: 最近 gyro 间隔/阈值/间隙标志与各类拒收计数.
std::size_t format_gap_line(char* out, std::size_t capacity,
    std::uint32_t last_gap_us, std::uint32_t horizon_us, unsigned gap_flag,
    std::uint32_t dup, std::uint32_t regressed, std::uint32_t too_old,
    std::uint32_t long_gap, std::uint32_t accepted) noexcept;
} // namespace ec::infantry::board::rtt
