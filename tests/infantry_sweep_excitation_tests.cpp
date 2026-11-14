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
    for (const auto& item : cases) {
        sweep::Engine engine;
        assert(engine.start(plan(), 1000, good));
        auto faulted = good;
        item.mutate(faulted);
        const auto torque = engine.step(2000, faulted);
        assert(torque == 0.0f);
        assert(engine.status().phase == sweep::Phase::aborted);
        assert(engine.status().stop == item.stop);
        assert(engine.finished());
        std::printf("  abort %-12s -> %u\n", item.what, static_cast<unsigned>(item.stop));
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
int main() {
    plan_validation();
    run_to_completion();
    fault_aborts();
    rearm_and_apply();
    std::puts("PASS: SYNTHETIC excitation plan/abort/rearm contracts");
}
