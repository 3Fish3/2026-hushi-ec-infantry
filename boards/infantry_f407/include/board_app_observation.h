#ifndef EC_BOARD_APP_OBSERVATION_H
#define EC_BOARD_APP_OBSERVATION_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// 正式 APP 的应用状态观测视图 (O6). 固定布局、无指针; 由 1 kHz 控制任务写, 注册表发布
// 与串口/SWD 观测读同一份已提交视图, 因此这里只描述"应用观察到了什么", 不代表实机结论.
// 数值字段的语义由 board_app.hpp 的映射注释给出, 不在本文件重复定义枚举.
// 域数与应用层 ec::infantry::app::domain_count 一致: 四轮 + 主/从 yaw + pitch + 两摩擦 + 拨弹.
#define EC_APP_DOMAIN_COUNT 10
#define EC_APP_DJI_SLOT_COUNT 9
#define EC_APP_DM_SLOT_COUNT 2
struct EcAppDomainObservation {
    uint8_t validity;   // app::DomainValidity
    uint8_t mode;       // control::ControlMode (与执行状态分开)
    uint8_t execute;    // 应用层执行结论 (0/1)
    uint8_t reason;     // app::Reason
    uint8_t fault;      // app::FaultState
    uint8_t reserved[3];
    uint32_t sampled_at_ms; // 该域反馈样本时刻; 无样本时为 0
    float target;       // 本周期采用的目标; 无目标时保持 0 并由 reason 说明
    float measurement;  // 本周期测量 (轴侧 SI); 不可用时保持 0 并由 validity 说明
};
struct EcAppDeviceObservation {
    uint8_t installed;
    uint8_t online;      // 反馈在 TTL 内
    uint8_t enabled;     // 使能已确认 (DM); DJI 无使能确认语义时保持 0
    uint8_t faulted;
    uint8_t temperature_valid;
    int8_t temperature_c;
    uint8_t bus;
    uint8_t reserved;
    uint32_t sampled_at_ms;
    float velocity_rad_s; // 轴侧角速度 (SI)
};
struct EcAppObservation {
    uint32_t now_ms;
    uint8_t source;      // app::Source
    uint8_t authority;   // 是否持有执行授权
    uint8_t manual_stop; // 人工整车停止挡
    uint8_t locally_ready;
    uint8_t host_session;
    uint8_t maintenance_session;
    uint8_t config_complete;
    uint8_t config_startable;
    uint32_t config_epoch, reference_epoch, config_missing_items;
    uint32_t accepted_packets, rejected_packets, link_losses, link_dropped;
    uint32_t frame_rejects, replay_rejects;
    uint32_t group_sent, group_refused, group_tx_error, dm_tx_error;
    struct EcAppDomainObservation domains[EC_APP_DOMAIN_COUNT];
    // 设备视图与板级静态装配层使用同一套监控 ID: DJI 0..8, DM 从 9 开始.
    struct EcAppDeviceObservation dji[EC_APP_DJI_SLOT_COUNT];
    struct EcAppDeviceObservation dm[EC_APP_DM_SLOT_COUNT];
    // UART v3 快状态的相同载荷; 已提交的固定存储快照, 不另算一份近似反馈.
    uint16_t fast_size;
    uint8_t fast_payload[240];
};
// 独立 SWD 调试视图, 不改变原 UART/注册表线格式. validity 必须和数值一起读取.
struct EcInfantryYawObservation {
    uint32_t sequence; // 奇数表示写入中; 读取前后需相同偶数
    uint32_t now_ms, imu_age_ms, joint_age_ms, reference_epoch;
    float target_heading_rad, measured_heading_rad, heading_rate_rad_s;
    float joint_angle_rad, torque_command_nm;
    uint8_t target_valid, imu_valid, joint_valid, torque_submitted;
    uint8_t execute, reason, dm_online, dm_enabled;
    // 未完成安装标定也可观察 IMU 原估计, 不把它当作可执行的云台世界参考.
    float imu_estimated_yaw_rad, imu_gyro_z_rad_s;
    uint32_t imu_sample_age_ms;
    uint8_t imu_sample_valid;
    uint8_t reserved[3];
    // 安装标定用: 未扣安装旋转的原始姿态与传感器量, 只用于人工核对, 不参与控制.
    // quat 顺序 w,x,y,z, 表示传感器系 -> 世界系; gyro/accel 为去标定零偏后的传感器系原始量.
    float imu_quat_w, imu_quat_x, imu_quat_y, imu_quat_z;
    float imu_gyro_x_rad_s, imu_gyro_y_rad_s;
    float imu_accel_x_mps2, imu_accel_y_mps2, imu_accel_z_mps2;
    float imu_roll_rad, imu_pitch_rad;
    uint8_t imu_calib_valid;
    uint8_t reserved2[3];
};
extern volatile struct EcInfantryYawObservation g_ec_infantry_yaw;
#ifdef __cplusplus
}
#endif
#endif
