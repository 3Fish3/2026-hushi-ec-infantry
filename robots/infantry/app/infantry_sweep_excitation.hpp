#pragma once
// 显式维护 START 才允许武装/注入; 完成或中止后撤输出并锁存, 不恢复旧目标.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ec::infantry::app::sweep {
enum class Phase : std::uint8_t { idle=0, running=1, complete=2, aborted=3, arming=4 };
enum class Stop : std::uint8_t {
    none=0, cancelled, authority, feedback, reference, speed, deviation, clock, can_error, invalid_plan
};
struct Plan {
    std::array<float,6> hz{};
    std::uint8_t count{}, cycles{5}; // 每频首尾各一个周期淡入/淡出
    float amplitude_nm{};           // 电机侧力矩幅值, 不混入世界系方向
};
struct Guard {
    bool maintenance{}, running{}, imu_valid{}, dm_enabled{}, dm_fresh{};
    bool target_valid{}, can_healthy{};
    std::uint32_t reference_epoch{};
    float heading_rad{}, rate_rad_s{};
};
struct Status {
    Phase phase{Phase::idle};
    Stop stop{Stop::none};
    std::uint8_t frequency_index{}, frequency_count{};
    float frequency_hz{}, requested_nm{}, applied_nm{};
    std::uint32_t run_id{};
};
class Engine {
public:
    // 软件试验边界, 不是已认证的硬件安全能力.
    static constexpr float amplitude_limit_nm=0.03f, rate_limit_rad_s=1.5f,
        heading_deviation_limit_rad=0.35f;
    static constexpr std::uint32_t arming_timeout_us=500000;
    [[nodiscard]] static bool valid_plan(const Plan& p) noexcept {
        if (!p.count || p.count>p.hz.size() || p.cycles<3 || p.cycles>12 ||
            !std::isfinite(p.amplitude_nm) || p.amplitude_nm<=0 ||
            p.amplitude_nm>amplitude_limit_nm) { return false; }
        float duration=0;
        for (unsigned i=0;i<p.count;++i) {
            if (!std::isfinite(p.hz[i]) || p.hz[i]<0.2f || p.hz[i]>5 ||
                (i && p.hz[i]<=p.hz[i-1])) { return false; }
            duration+=static_cast<float>(p.cycles)/p.hz[i];
        }
        return duration<=120;
    }
    [[nodiscard]] bool start(const Plan& plan,std::uint32_t now,const Guard& g) noexcept {
        if (!valid_plan(plan) || active() || !ready(g) ||
            std::fabs(g.rate_rad_s)>rate_limit_rad_s) { return false; }
        plan_=plan; baseline_heading_=g.heading_rad; baseline_epoch_=g.reference_epoch;
        armed_at_=segment_at_=last_at_=now;
        state_={}; state_.phase=g.dm_enabled ? Phase::running : Phase::arming;
        state_.frequency_count=plan.count; state_.frequency_hz=plan.hz[0];
        state_.run_id=++run_id_;
        return true;
    }
    [[nodiscard]] float step(std::uint32_t now,const Guard& g) noexcept {
        state_.requested_nm=state_.applied_nm=0;
        if (!active()) { return 0; }
        if (!g.maintenance) { abort(Stop::authority); return 0; }
        if (!ready(g)) { abort(Stop::feedback); return 0; }
        if (g.reference_epoch!=baseline_epoch_) { abort(Stop::reference); return 0; }
        if (std::fabs(g.heading_rad-baseline_heading_)>heading_deviation_limit_rad) {
            abort(Stop::deviation); return 0;
        }
        if (std::fabs(g.rate_rad_s)>rate_limit_rad_s) { abort(Stop::speed); return 0; }
        const auto delta=now-last_at_;
        if (delta>10000u) { abort(Stop::clock); return 0; }
        last_at_=now;
        // 新 START 显式解除终态锁存, 仅允许既有使能流程重新取得新鲜确认; 尚不注入.
        if (state_.phase==Phase::arming) {
            if (now-armed_at_>=arming_timeout_us) { abort(Stop::feedback); return 0; }
            if (!g.dm_enabled) { return 0; }
            segment_at_=now; state_.phase=Phase::running;
        } else if (!g.dm_enabled) { abort(Stop::feedback); return 0; }
        auto elapsed=now-segment_at_;
        while (state_.frequency_index<plan_.count) {
            const float duration=1e6f*static_cast<float>(plan_.cycles)/plan_.hz[state_.frequency_index];
            if (static_cast<float>(elapsed)<duration) { break; }
            segment_at_+=static_cast<std::uint32_t>(duration);
            elapsed=now-segment_at_;
            if (++state_.frequency_index==plan_.count) {
                state_.phase=Phase::complete; state_.frequency_hz=0; return 0;
            }
        }
        const float hz=plan_.hz[state_.frequency_index], t=static_cast<float>(elapsed)*1e-6f;
        const float period=1/hz, remaining=static_cast<float>(plan_.cycles)*period-t;
        const float envelope=std::clamp(std::min(t,remaining)/period,0.0f,1.0f);
        state_.frequency_hz=hz;
        state_.requested_nm=plan_.amplitude_nm*envelope*std::sin(6.283185307179586f*hz*t);
        return state_.requested_nm;
    }
    void applied(float value) noexcept { if (state_.phase==Phase::running) { state_.applied_nm=value; } }
    void abort(Stop reason) noexcept {
        if (active()) { state_.phase=Phase::aborted; state_.stop=reason; }
        state_.requested_nm=state_.applied_nm=0;
    }
    void cancel() noexcept { abort(Stop::cancelled); }
    [[nodiscard]] bool active() const noexcept { return state_.phase==Phase::running || state_.phase==Phase::arming; }
    [[nodiscard]] bool finished() const noexcept { return state_.phase==Phase::complete || state_.phase==Phase::aborted; }
    // 仅由执行层确认新鲜停止挡与Disabled后调用; 不启动电机, 不重放旧试验.
    bool clear_after_stop() noexcept {
        if (!finished()) { return false; }
        state_={}; plan_={};
        return true;
    }
    [[nodiscard]] const Status& status() const noexcept { return state_; }
private:
    [[nodiscard]] static bool ready(const Guard& g) noexcept {
        return g.maintenance && g.running && g.imu_valid && g.dm_fresh && g.target_valid &&
            g.can_healthy && std::isfinite(g.heading_rad) && std::isfinite(g.rate_rad_s);
    }
    Plan plan_{};
    Status state_{};
    std::uint32_t baseline_epoch_{},armed_at_{},segment_at_{},last_at_{},run_id_{};
    float baseline_heading_{};
};
} // 命名空间 ec::infantry::app::sweep
