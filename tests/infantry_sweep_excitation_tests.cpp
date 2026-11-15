// SYNTHETIC 激励状态机契约; 不接触硬件, 不代表实车安全边界.
#include "infantry_sweep_excitation.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
namespace sweep = ec::infantry::app::sweep;

static sweep::Guard healthy() {
    sweep::Guard g{};
    g.maintenance = g.running = g.imu_valid = g.dm_enabled = true;
    g.dm_fresh = g.target_valid = g.can_healthy = true;
    g.reference_epoch = 42; g.heading_rad = 0.25f; g.rate_rad_s = 0.1f;
    return g;
}
static sweep::Plan plan() {
    sweep::Plan p{};
    p.hz[0] = 0.5f; p.hz[1] = 1.0f; p.hz[2] = 2.0f;
    p.count = 3; p.cycles = 3; p.amplitude_nm = 0.01f;
    return p;
}
static void plan_validation() {
    assert(sweep::Engine::valid_plan(plan()));
    auto bad = plan(); bad.amplitude_nm = 0.2f;
    assert(!sweep::Engine::valid_plan(bad)); // 超软件上限
    bad = plan(); bad.hz[0] = 0.1f; assert(!sweep::Engine::valid_plan(bad));
    bad = plan(); bad.hz[1] = 0.5f; assert(!sweep::Engine::valid_plan(bad)); // 必须严格递增
    bad = plan(); bad.cycles = 2; assert(!sweep::Engine::valid_plan(bad));
    bad = plan(); bad.count = 0; assert(!sweep::Engine::valid_plan(bad));
    bad = plan(); bad.amplitude_nm = -0.01f; assert(!sweep::Engine::valid_plan(bad));
    bad = plan(); bad.count = 6; bad.hz = {0.2f,0.3f,0.4f,0.5f,0.6f,0.7f};
    bad.cycles = 12; bad.amplitude_nm = 0.03f;
    assert(!sweep::Engine::valid_plan(bad)); // 12s/0.2Hz*6 > 120s
}
static void run_to_completion() {
    sweep::Engine engine;
    const auto good = healthy();
    assert(engine.start(plan(), 1000, good));
    assert(!engine.start(plan(), 1000, good)); // 运行中不接受重复启动
    // 3 频点 × 3 周期: 0.5Hz=6s, 1Hz=3s, 2Hz=1.5s, 共 10.5s
    constexpr float amplitude = 0.01f;
    for (unsigned ms = 1; ms < 11'000; ++ms) {
        const auto torque = engine.step(1000 + 1000u*ms, good);
        assert(std::isfinite(torque));
        assert(std::fabs(torque) <= amplitude + 1e-6f);
    }
    assert(engine.status().phase == sweep::Phase::complete);
    assert(engine.finished());
    assert(!engine.active());
    assert(engine.status().requested_nm == 0.0f);
    const auto completed_id = engine.status().run_id;
    assert(engine.clear_after_stop());
    assert(engine.status().phase == sweep::Phase::idle);
    assert(!engine.finished() && !engine.active());
    assert(engine.status().requested_nm == 0.0f && engine.status().applied_nm == 0.0f);
    // 清理不是重新启动; 新试验仍要显式 START, run id不复用.
    const auto rearm = engine.start(plan(), 20'000, good);
    assert(rearm);
    assert(engine.active());
    assert(engine.status().run_id > completed_id);
    assert(!engine.clear_after_stop()); // 不能直接清正在运行的试验

}
static void fault_aborts() {
    const auto good = healthy();
    struct Case { const char* what; void (*mutate)(sweep::Guard&); sweep::Stop stop; };
    const Case cases[] = {
        {"maintenance", [](sweep::Guard& g) { g.maintenance = false; }, sweep::Stop::authority},
        {"dm_disabled", [](sweep::Guard& g) { g.dm_enabled = false; }, sweep::Stop::feedback},
        {"dm_stale", [](sweep::Guard& g) { g.dm_fresh = false; }, sweep::Stop::feedback},
        {"imu", [](sweep::Guard& g) { g.imu_valid = false; }, sweep::Stop::feedback},
        {"target", [](sweep::Guard& g) { g.target_valid = false; }, sweep::Stop::feedback},
        {"running", [](sweep::Guard& g) { g.running = false; }, sweep::Stop::feedback},
        {"epoch", [](sweep::Guard& g) { g.reference_epoch = 43; }, sweep::Stop::reference},
        {"deviation", [](sweep::Guard& g) { g.heading_rad = 1.0f; }, sweep::Stop::deviation},
        {"speed", [](sweep::Guard& g) { g.rate_rad_s = 2.0f; }, sweep::Stop::speed},
    };
    for (auto mode : {sweep::Mode::torque, sweep::Mode::angle}) {
        for (const auto& item : cases) {
            sweep::Engine engine;
            auto trial = plan(); trial.mode = mode;
            if (mode == sweep::Mode::angle) { trial.amplitude_nm = 0; trial.amplitude_rad = 0.05f; }
            assert(engine.start(trial, 1000, good));
            auto faulted = good;
            item.mutate(faulted);
            const auto torque = engine.step(2000, faulted);
            assert(torque == 0.0f);
            assert(engine.status().phase == sweep::Phase::aborted);
            assert(engine.status().stop == item.stop);
            assert(engine.finished());
            assert(!engine.angle_target(2000));
            std::printf("  abort %-12s -> %u\n", item.what, static_cast<unsigned>(item.stop));
        }
    }
    // 时钟跳变超过 10ms 也中止.
    sweep::Engine clock;
    assert(clock.start(plan(), 1000, good));
    (void)clock.step(1000 + 20'000u, good);
    assert(clock.status().stop == sweep::Stop::clock);
    // CAN 队列失败由执行层调用 abort.
    sweep::Engine can;
    assert(can.start(plan(), 1000, good));
    can.abort(sweep::Stop::can_error);
    assert(can.status().stop == sweep::Stop::can_error && can.finished());
}
static void rearm_and_apply() {
    const auto good = healthy();
    sweep::Engine engine;
    assert(engine.start(plan(), 1000, good));
    assert(engine.active());
    engine.applied(-0.004f);
    assert(engine.status().applied_nm == -0.004f); // 只记录队列接纳值
    engine.cancel();
    assert(engine.status().stop == sweep::Stop::cancelled);
    assert(engine.finished());
    assert(engine.clear_after_stop());
    assert(engine.status().phase == sweep::Phase::idle);
    // 完成/中止后必须显式再启动, 不会自动恢复旧目标.
    assert(engine.start(plan(), 3000, good));
    assert(engine.active());
    // 微秒时钟回绕不应误判.
    sweep::Engine wrap;
    assert(wrap.start(plan(), 0xFFFFFC18u, good));
    assert(std::isfinite(wrap.step(0u, good)));
}
static void angle_contracts() {
    auto angle = plan(); angle.mode = sweep::Mode::angle;
    angle.amplitude_nm = 0; angle.amplitude_rad = 0.05f;
    assert(sweep::Engine::valid_plan(angle));
    auto bad = angle; bad.amplitude_nm = 0.01f;
    assert(!sweep::Engine::valid_plan(bad)); // 不允许两种单位混入同一计划
    bad = angle; bad.amplitude_rad = 0.2001f;
    assert(!sweep::Engine::valid_plan(bad));
    bad = plan(); bad.amplitude_rad = 0.05f;
    assert(!sweep::Engine::valid_plan(bad));
    for (float value : {0.0f, -0.01f, NAN, INFINITY}) {
        bad = angle; bad.amplitude_rad = value;
        assert(!sweep::Engine::valid_plan(bad));
    }
    bad = angle; bad.mode = static_cast<sweep::Mode>(2);
    assert(!sweep::Engine::valid_plan(bad));
    const auto good = healthy();
    sweep::Engine engine;
    auto arming = good; arming.dm_enabled = false;
    assert(engine.start(angle, 1000, arming));
    assert(engine.status().mode == sweep::Mode::angle);
    assert(engine.angle_target(2000) == good.heading_rad);
    assert(!engine.start(plan(), 2000, good)); // 角度与力矩试验互斥
    assert(engine.step(2000, arming) == 0);
    assert(engine.step(3000, good) == 0); // 新鲜 Enabled 才开始计时
    constexpr std::uint32_t start = 3000;
    for (std::uint32_t ms = 1; ms <= 10'500; ++ms) {
        const auto now = start + ms*1000;
        const auto target = engine.angle_target(now); // 与 Cycle 一致, 先生成目标再提交
        assert(target);
        const std::uint32_t offset = ms < 6000 ? 0 : ms < 9000 ? 6000 : 9000;
        const float hz = ms < 6000 ? 0.5f : ms < 9000 ? 1.0f : 2.0f;
        const float t = static_cast<float>(ms-offset)*0.001f;
        const float envelope = std::clamp(std::min(t, 3/hz-t)*hz, 0.0f, 1.0f);
        const float expected = good.heading_rad + 0.05f*envelope*std::sin(6.283185307179586f*hz*t);
        assert(std::fabs(*target-expected) < 2e-6f);
        assert(engine.step(now, good) == 0);
        engine.applied(0.02f); // 角度模式即使被误调用也不记录力矩注入
        assert(engine.status().requested_nm == 0 && engine.status().applied_nm == 0);
    }
    assert(engine.status().phase == sweep::Phase::complete);
    assert(!engine.angle_target(start+10'501'000));
    assert(engine.status().mode == sweep::Mode::angle); // 终态采集仍携带模式
    assert(engine.clear_after_stop());
    assert(engine.start(plan(), 20'000'000, good));
    assert(!engine.angle_target(20'001'000));
    assert(engine.status().mode == sweep::Mode::torque);
    sweep::Engine wrap;
    assert(wrap.start(angle, 0xfffffc18u, good));
    assert(wrap.angle_target(0u));
    assert(wrap.step(0u, good) == 0);
}
static void angle_plan_respects_rate_guard() {
    auto angle = plan();
    angle.mode = sweep::Mode::angle; angle.amplitude_nm = 0;
    angle.count = 1; angle.hz[0] = 5.0f; angle.amplitude_rad = 0.05f;
    // 各自合法的幅值/频率组合, 理想跟踪也会超过既有1.5 rad/s保护.
    assert(!sweep::Engine::valid_plan(angle));
    sweep::Engine rejected;
    assert(!rejected.start(angle, 1000, healthy()));
    assert(rejected.status().phase == sweep::Phase::idle && rejected.status().run_id == 0);
    angle.hz[0] = 2.0f; angle.amplitude_rad = 0.2f;
    assert(!sweep::Engine::valid_plan(angle));

    // 线上float32幅值在理论速率边界两侧的相邻可表示值必须得到不同结果.
    constexpr double tau = 6.28318530717958647692;
    angle.hz[0] = 5.0f;
    const double boundary = 1.5 / (tau * 5.0);
    float inside = static_cast<float>(boundary);
    if (static_cast<double>(inside) > boundary) { inside = std::nextafter(inside, 0.0f); }
    angle.amplitude_rad = inside;
    assert(sweep::Engine::valid_plan(angle));
    angle.amplitude_rad = std::nextafter(inside, INFINITY);
    assert(!sweep::Engine::valid_plan(angle));
    angle.count = 2; angle.hz[0] = 1.0f; angle.hz[1] = 5.0f;
    assert(!sweep::Engine::valid_plan(angle)); // 后续频点也必须检查, 不能只看首点.

    // 力矩幅值不与角速度同量纲, 不把角度计划约束误套到力矩试验.
    auto torque = plan(); torque.count = 1; torque.hz[0] = 5.0f;
    torque.amplitude_nm = sweep::Engine::amplitude_limit_nm;
    assert(sweep::Engine::valid_plan(torque));
}
static void angle_envelope_respects_peak_rate() {
    for (std::uint8_t cycles : {3, 4, 12}) {
        auto angle = plan(); angle.mode = sweep::Mode::angle; angle.amplitude_nm = 0;
        angle.amplitude_rad = 0.04f; angle.count = 1; angle.hz[0] = 5.0f; angle.cycles = cycles;
        sweep::Engine engine;
        const auto good = healthy();
        assert(engine.start(angle, 0, good));
        auto previous = engine.angle_target(0);
        float ramp_in_peak = 0, steady_peak = 0, ramp_out_peak = 0;
        const unsigned duration_ms = static_cast<unsigned>(cycles) * 200;
        for (unsigned ms = 1; ms <= duration_ms; ++ms) {
            const auto target = engine.angle_target(ms * 1000u);
            assert(target && previous);
            const float rate = std::fabs((*target - *previous) / 0.001f);
            // 直接检查真实目标波形的差分斜率, 包含首尾包络及其切换点.
            auto& peak = ms <= 200 ? ramp_in_peak :
                ms <= duration_ms - 200 ? steady_peak : ramp_out_peak;
            peak = std::max(peak, rate);
            assert(rate < 1.258f); // 理论2πAf约1.25664, 容差只覆盖float波形求值.
            previous = target;
            (void)engine.step(ms * 1000u, good);
        }
        assert(ramp_in_peak > 1.24f && steady_peak > 1.24f && ramp_out_peak > 1.24f);
        assert(engine.status().phase == sweep::Phase::complete);
    }
}
int main() {
    angle_plan_respects_rate_guard();
    angle_envelope_respects_peak_rate();
    angle_contracts();
    plan_validation();
    run_to_completion();
    fault_aborts();
    rearm_and_apply();
    std::puts("PASS: SYNTHETIC excitation plan/abort/rearm contracts");
}
