#pragma once
// 步兵单云台 yaw 关节角: 把达妙 MIT 反馈的 [-12.5, 12.5] rad 回绕展开为连续角度.
//
// 设计要点:
//  - 反馈窗口是 25 rad, 不是 2*pi; 相邻帧差值超过 12.5 rad 时按 25 rad 修正一次回绕.
//  - 25 rad 回绕为已观察到的反馈行为；丢帧、断线和反馈量程需独立验证。
//  - 本角度用于机械相对角估计，不替代云台世界坐标惯性稳向反馈。
//  - 首帧保留其原始位置作为连续角初值；断线重置后历史圈数不可恢复，须重建参考。
#include <cmath>
#include <cstdint>
#include <optional>

namespace ec::infantry::app {

class ContinuousYawAngle {
public:
    static constexpr float k_position_min_rad = -12.5f;
    static constexpr float k_position_max_rad = 12.5f;
    static constexpr float k_position_span_rad =
        k_position_max_rad - k_position_min_rad;

    // 丢弃历史；重连后旧参考不再可靠，调用者须重新对齐。
    void reset() noexcept {
        initialized_ = false;
        previous_rad_ = 0.0f;
        continuous_rad_ = 0.0f;
        wrap_events_ = 0;
    }

    [[nodiscard]] bool valid() const noexcept { return initialized_; }

    // 返回 false 表示该帧不能用于角度估计: 非有限或超出 MIT 反馈窗口.
    // 拒绝的帧不更新 previous_rad_, 避免把异常值当成一次合法位移.
    [[nodiscard]] bool update(float position_rad) noexcept {
        if (!std::isfinite(position_rad) ||
            position_rad < k_position_min_rad || position_rad > k_position_max_rad) {
            return false;
        }
        if (!initialized_) {
            previous_rad_ = position_rad;
            continuous_rad_ = position_rad;
            initialized_ = true;
            return true;
        }
        float delta = position_rad - previous_rad_;
        if (delta > k_position_span_rad * 0.5f) {
            delta -= k_position_span_rad;
            ++wrap_events_;
        } else if (delta < -k_position_span_rad * 0.5f) {
            delta += k_position_span_rad;
            ++wrap_events_;
        }
        continuous_rad_ += delta;
        previous_rad_ = position_rad;
        return true;
    }

    // 连续角度 (rad); 每帧位移必须小于半个反馈窗口，未初始化返回 nullopt。
    [[nodiscard]] std::optional<float> continuous_rad() const noexcept {
        if (!initialized_) { return std::nullopt; }
        return continuous_rad_;
    }

    // 相对机械零位的角度, 仍保留跨圈连续语义.
    [[nodiscard]] std::optional<float> relative_rad(float zero_offset_rad) const noexcept {
        if (!initialized_) { return std::nullopt; }
        return continuous_rad_ - zero_offset_rad;
    }

    [[nodiscard]] std::uint32_t wrap_events() const noexcept { return wrap_events_; }

private:
    bool initialized_{};
    float previous_rad_{};
    float continuous_rad_{};
    std::uint32_t wrap_events_{};
};

} // namespace ec::infantry::app
