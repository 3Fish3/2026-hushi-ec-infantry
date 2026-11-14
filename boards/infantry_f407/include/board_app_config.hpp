#pragma once
// 正式 APP 的部署配置表 (O6).
// 本文件只使用 ec::infantry::app 的配置类型, 不含 HAL/板级类型, 因此同一份表既能编进
// F407 固件, 也能被主机测试与离线检查脚本直接核对, 避免"测试用的表"和"烧录的表"分叉.
//
// 步兵装配来自同一份 constexpr 工厂; 实车参数及遥控映射已按旧工程换算.
// 参数齐备不等于实车验收, 板级仍由遥控权限及停止状态决定是否执行.
#include "infantry_config.hpp"
#include "infantry_remote.hpp"
#include <array>
#include <cstdint>

namespace ec::infantry::board {
// 装配参考旧步兵配置: 四轮C620在CAN1, 云台yaw与丝杆pitch均为达妙J4310.
struct HarnessEntry {
    std::uint8_t bus{};
    drivers::DjiMode mode{drivers::DjiMode::c620_current};
    std::uint8_t device_id{};
};
inline constexpr std::array<HarnessEntry, 4> known_dji_harness{{
    {1, drivers::DjiMode::c620_current, 1},
    {1, drivers::DjiMode::c620_current, 2},
    {1, drivers::DjiMode::c620_current, 3},
    {1, drivers::DjiMode::c620_current, 4},
}};
// 两颗达妙都在 CAN1: yaw(0x005/0x005) 与丝杆 pitch(0x001/0x011).
inline constexpr std::uint32_t known_dm_harness_count = 2;

// 软件时序策略 (不是实测机械/电气参数): 控制周期由板级 1 kHz 任务决定, 遥控/IMU 时效
// 沿用板级既有口径 (board_inputs.hpp), 目标与上位机链路时效是首版策略默认值, 待与
// 上位机回传率核对后确认.
inline constexpr float app_control_dt_s = 0.001f;
// 单次控制步长的上限 (ms): 超过它按超期计数; 真实 dt 仍照传给轴/环, 由它们既有的
// max_dt 拒绝路径处理, 不用标称周期冒充正常采样.
inline constexpr std::uint32_t app_max_dt_ms = 5;

// 单个控制周期的真实步长决策 (纯函数, 便于用受控时钟覆盖边界).
//  - 首次周期: 用明确的初始化步长 (不把合法 Tick=0 当哨兵);
//  - 重复时刻 (elapsed==0) 或时间回退 (半个回绕窗口外的倒退): dt_s = 0,
//    交给轴/环已有的 dt 拒绝/不推进路径, 不伪装成标称周期;
//  - 否则用真实毫秒差, 并单独给出超期标记供统计.
struct AppControlStep {
    float dt_s{app_control_dt_s};
    bool overrun{};
};
[[nodiscard]] constexpr AppControlStep app_control_step(
    std::uint32_t now_ms, std::uint32_t last_ms, bool have_last) noexcept {
    AppControlStep out{};
    if (!have_last) { return out; }
    const std::uint32_t elapsed_ms = now_ms - last_ms;
    if (elapsed_ms == 0u || elapsed_ms >= 0x80000000u) {
        out.dt_s = 0.0f;
        return out;
    }
    out.overrun = elapsed_ms > app_max_dt_ms;
    out.dt_s = static_cast<float>(elapsed_ms) * 0.001f;
    return out;
}

// 正式 APP 控制使用微秒差; 毫秒版本保留给已有观测/离线入口.
[[nodiscard]] constexpr AppControlStep app_control_step_us(
    std::uint32_t now_us, std::uint32_t last_us, bool have_last) noexcept {
    if (!have_last) { return {}; }
    const auto elapsed = now_us - last_us;
    if (elapsed == 0 || elapsed >= 0x80000000u) { return {0.0f, false}; }
    return {static_cast<float>(elapsed) * 1e-6f, elapsed > app_max_dt_ms * 1000u};
}

// 本地操纵映射量程/档位阈值 (C06). 底盘与云台摇杆量程由旧工程有效配置换算,
// 键鼠未配置的比例仍为 0; 转换结果需要在当前车体复核.
[[nodiscard]] constexpr app::RemoteMappingConfig deployment_mapping() noexcept {
    app::RemoteMappingConfig mapping{};
    mapping.infantry_controls = true;
    mapping.stick_deadzone = 0.05f;
    // 云台 yaw 目标角速度上限 (rad/s): 由旧工程摇杆映射换算而来, 仍属待整定项.
    // 摇杆在本地"主云台"层时以此速率积分目标航向, 回中即保持.
    mapping.yaw_rate_max_rad_s = 1.5f;
    // 位置目标变化率上限沿用46.2 deg/s(电机输出轴), 普通/小陀螺共用线性映射.
    // 它不是枪管俯仰角速度, 丝杆比例未标定前不做两者换算.
    mapping.pitch_rate_max_rad_s = 46.2f * 3.14159265358979323846f / 180.0f;
    // 保留用户要求反向后的物理操纵方向; 旧速度路径自带负号, 现已删除,
    // 所以位置路径Pitch取+1才能与旧路径的(-raw)×(-1)同向, Yaw仍取-1.
    mapping.yaw_stick_sign = -1;
    mapping.pitch_stick_sign = 1;
    // 按旧工程满杆 3960 输出轴 deg/s 与 60 mm 轮半径换算, 再按要求将速度减半;
    // 旧全向轮每轴系数为 1, 本框架单位滚动向量含 1/sqrt(2), 故需补偿 sqrt(2).
    constexpr float pi = 3.14159265358979323846f;
    constexpr float sqrt2 = 1.4142135623730950488f;
    mapping.chassis_vx_max_mps = 0.5f * 3960.0f * pi / 180.0f * 0.06f * sqrt2;
    mapping.chassis_vy_max_mps = mapping.chassis_vx_max_mps;
    // 旧自转参考 140 换算后再减半; 这是等效目标, 不是实测车体角速度.
    mapping.chassis_wz_max_rad_s = 0.5f * 140.0f * 400.0f * pi / 180.0f * pi / 180.0f *
        0.06f * sqrt2 / 0.4f;
    return mapping;
}
inline constexpr std::uint32_t app_target_ttl_ms = 100;
inline constexpr std::uint32_t app_host_link_ttl_ms = 200;
inline constexpr float app_max_dt_s = 0.005f;

// 板级与主机共用步兵装配表, 仅覆盖板级软件时序策略.
[[nodiscard]] constexpr app::InfantryConfig deployment_config() noexcept {
    auto config = app::infantry_deployment_config();
    config.timing.target_ttl = Lifetime{app_target_ttl_ms};
    config.timing.host_link_ttl = Lifetime{app_host_link_ttl_ms};
    config.timing.max_dt_s = app_max_dt_s;
    return config;
}

// 部署配置的静态摘要: 固定布局、无指针, 供离线检查脚本从未压缩镜像按符号读取,
// 核对镜像里不是合成配置、也不含"装配了但来源未知"的槽位.
inline constexpr std::uint32_t deployment_summary_dji_slots = 9;
struct DeploymentSummary {
    std::uint32_t magic;
    std::uint32_t layout;
    std::uint32_t kind;              // app::ConfigKind 的数值
    std::uint32_t dji_capacity;      // 配置可声明的 DJI 槽数
    std::uint32_t dm_capacity;
    std::uint32_t installed_dji;     // 已声明装配的 DJI 槽数
    std::uint32_t installed_dm;
    std::uint32_t synthetic_values;  // 部署配置里的合成来源值 (必须为 0)
    std::uint32_t unknown_origin_installed; // 装配但来源未知 (部署用途拒绝)
    std::uint32_t harness_dji;       // 已验证接线条目数
    std::uint32_t harness_dm;
    // 每槽实际总线/设备 ID: bus | device_id<<8, 0 表示该槽未装配.
    // 只报装配数量时, "底盘挂错总线"这类镜像照样能通过审计; 地址进摘要才能离线核对.
    std::uint32_t dji_address[deployment_summary_dji_slots];
    std::uint32_t dm_address;        // bus | can_id<<8 | master_id<<16
    std::uint32_t dm_address_1;      // 第二颗达妙, 布局同上; 0表示未装配
};
inline constexpr std::uint32_t deployment_summary_magic = 0x45434150u; // "ECAP"
inline constexpr std::uint32_t deployment_summary_layout = 3; // 3: 增补第二颗达妙地址

[[nodiscard]] constexpr DeploymentSummary summarize(const app::InfantryConfig& config) noexcept {
    DeploymentSummary summary{deployment_summary_magic, deployment_summary_layout,
        static_cast<std::uint32_t>(config.kind),
        static_cast<std::uint32_t>(config.dji.size()),
        static_cast<std::uint32_t>(config.dm.size()), 0, 0, 0, 0,
        static_cast<std::uint32_t>(known_dji_harness.size()), known_dm_harness_count,
        {}, 0, 0}; // 地址表与 DM 地址先清零, 下面按实际装配填写
    std::uint32_t slot_index = 0;
    for (const auto& slot : config.dji) {
        const std::uint32_t index = slot_index++;
        if (!slot.installed) { continue; }
        ++summary.installed_dji;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
        if (index < deployment_summary_dji_slots) {
            summary.dji_address[index] = static_cast<std::uint32_t>(slot.bus) |
                (static_cast<std::uint32_t>(slot.device_id) << 8);
        }
    }
    for (const auto& slot : config.dm) {
        if (!slot.installed) { continue; }
        ++summary.installed_dm;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
    }
    // 槽0=yaw(0x005/0x005), 槽1=pitch(0x001/0x011); 编码为 bus | can_id<<8 | master_id<<16,
    // 高位 bit30 标记槽1, 保持单字段兼容.
    for (std::size_t i = 0; i < config.dm.size(); ++i) {
        const auto& slot = config.dm[i];
        if (!slot.installed) { continue; }
        const auto address = static_cast<std::uint32_t>(slot.bus) |
            (static_cast<std::uint32_t>(slot.can_id.value_or(0)) << 8) |
            (static_cast<std::uint32_t>(slot.master_id.value_or(0)) << 16);
        if (i == 0) { summary.dm_address = address; }
        else { summary.dm_address_1 = address; }
    }
    return summary;
}

inline constexpr DeploymentSummary deployment_summary = summarize(deployment_config());
static_assert(deployment_summary.magic == deployment_summary_magic);
static_assert(deployment_summary.kind == static_cast<std::uint32_t>(app::ConfigKind::real_deployment),
    "部署配置不得声明为合成夹具或未配置");
static_assert(deployment_summary.synthetic_values == 0, "部署配置不得含合成来源的值");
static_assert(deployment_summary.installed_dji == 4 && deployment_summary.installed_dm == 2,
    "步兵装配必须为四 DJI 加两 DM");
static_assert(deployment_summary.dm_address_1 != 0, "第二颗达妙地址必须写入摘要");
static_assert(deployment_summary.dji_capacity >= known_dji_harness.size(),
    "已验证接线条目数不得超过配置可声明的 DJI 槽数");
} // 命名空间 ec::infantry::board

extern "C" {
// 部署摘要符号: 单一定义在包含本头文件的板级 TU (固件: board_app.cpp; 主机: 部署配置测试).
// 外部链接 + used 保证它被发出; 读取方必须经 volatile 指针读, 否则 Release 下引用会被
// 常量折叠, 链接器就会把符号 GC 掉 (镜像里就没有可读的部署摘要了).
// 因此本头文件不得被同一二进制里的多个 TU 同时包含.
extern const ec::infantry::board::DeploymentSummary ec_app_deployment_summary;
[[gnu::used]] const ec::infantry::board::DeploymentSummary ec_app_deployment_summary =
    ec::infantry::board::deployment_summary;
}
