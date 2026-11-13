#ifndef EC_BOARD_MONITOR_H
#define EC_BOARD_MONITOR_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// 监控/采集协议共用 USART1 收发队列; 不启动第二条 UART DMA.
// APP 由控制任务提交完整监控请求并服务, 非 APP 由观测任务解析 USART1 字节.
void ec_monitor_init(void);
void ec_monitor_poll(void);
#ifdef __cplusplus
}
#include "ec/transport/uart_frame.hpp"
// 与 APP 会话协议复用同一个外层分帧器; 只提交已识别的监控消息类型.
void ec_monitor_request(const ec::transport::UartFrame& frame) noexcept;
#endif
#endif
