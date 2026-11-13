#include "board_rtt.hpp"
#include <atomic>

namespace ec::infantry::board::rtt {
ControlBlock control_block{};
char up_buffer[up_buffer_size]{};

void init() noexcept {
    // 描述符就绪后最后发布标识, 避免调试器识别到半初始化控制块.
    control_block.id[0] = '\0';
    control_block.max_up = 1;
    control_block.max_down = 0;
    auto& up = control_block.up[0];
    up.name = "Calibration";
    up.buffer = up_buffer;
    up.size = up_buffer_size;
    up.write_offset = 0;
    up.read_offset = 0;
    up.flags = 0; // RTT_MODE_NO_BLOCK_SKIP
    constexpr char magic[16] = "SEGGER RTT";
    for (std::size_t i = 1; i < sizeof(magic); ++i) { control_block.id[i] = magic[i]; }
    std::atomic_thread_fence(std::memory_order_release);
    control_block.id[0] = magic[0];
}

std::size_t write(const char* text, std::size_t size) noexcept {
    if (size == 0 || size >= up_buffer_size) { return 0; }
    auto& up = control_block.up[0];
    const auto write_at = up.write_offset;
    const auto read_at = up.read_offset;
    const auto available = (read_at + up_buffer_size - write_at - 1) % up_buffer_size;
    if (size > available) { return 0; }
    for (std::size_t i = 0; i < size; ++i) {
        up_buffer[(write_at + i) % up_buffer_size] = text[i];
    }
    // Cortex-M4 上发布游标前保证缓冲区字节已写入 SRAM.
    std::atomic_thread_fence(std::memory_order_release);
    up.write_offset = (write_at + size) % up_buffer_size;
    return size;
}
} // namespace ec::infantry::board::rtt
