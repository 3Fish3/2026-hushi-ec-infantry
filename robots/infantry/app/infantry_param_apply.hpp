#pragma once
// 参数生效事务 (APP11). 参数接口的意义在于"读回 == 实际控制配置", 因此只把新数值写进
// ParamStore 不能叫生效: 必须真正落到运行对象上, 再从运行对象读回实际值.
//
// 事务顺序: 值域校验 → 参考类要求受影响机构已退出执行 → 落到运行对象 → 从运行对象读回
//           → 回写参数存储 → 参考语义变化时上推代次并作废受影响目标.
// 尚未实现生效路径的参数返回 unsupported, 既不谎报 effective, 也不留下"已写入"的假象.
// 资格判定 (是否持权、是否只读) 由协议层负责, 本模块只负责"能不能真的生效".
#include "infantry_app.hpp"
#include "infantry_execution.hpp"
#include "infantry_params.hpp"
#include "infantry_perception.hpp"

namespace ec::infantry::app {
// 仅表示该 ID 有生效实现; 权限、机构状态和运行对象是否齐备仍由事务执行检查.
[[nodiscard]] bool param_has_apply_path(const InfantryApp& app, std::uint16_t id) noexcept;

// 一次参数写入的完整事务. idle 表示"受影响机构已退出执行" (参考类参数的必要条件).
// 返回 accepted 时, ParamStore 中该参数的值来自运行对象的实际生效值.
class InfantryCycle;
[[nodiscard]] ParamWriteResult apply_param(ParamStore& params, InfantryApp& app,
    InfantryExecutor* executor, InfantryPerception* perception, InfantryCycle* cycle,
    std::uint16_t id, float value, bool idle) noexcept;
} // 命名空间 ec::infantry::app
