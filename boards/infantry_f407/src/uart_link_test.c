// 独立串口链路测试: 只初始化时钟、USART1/USART6, 不初始化 CAN、IMU 或电机.
// 115200 8N1, 每 500 ms 发一次端口标识; 任意收到的字节原样回传.
#include "stm32f4xx_hal.h"
#include <stdint.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim14;
const char g_uart_link_test_id[] = "infantry-uart-link-test-115200-no-can";
volatile uint32_t g_uart1_echo_bytes;
volatile uint32_t g_uart6_echo_bytes;
volatile uint32_t g_uart1_tx_errors;
volatile uint32_t g_uart6_tx_errors;
volatile uint32_t g_uart1_framing;
volatile uint32_t g_uart6_framing;

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

static void clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clock = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // 沿用本板已运行的 12 MHz HSE / 168 MHz 系统时钟配置.
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 6;
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV4;
    clock.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart) {
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (uart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // TX 与 RX 分属不同端口, 必须各自按端口配置: 复用号相同但寄存器不同.
        gpio.Alternate = GPIO_AF7_USART1;
        gpio.Pin = GPIO_PIN_9; // PA9 = USART1_TX
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Alternate = GPIO_AF7_USART1;
        gpio.Pin = GPIO_PIN_7; // PB7 = USART1_RX
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (uart->Instance == USART6) {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        gpio.Alternate = GPIO_AF8_USART6;
        gpio.Pin = GPIO_PIN_14 | GPIO_PIN_9; // PG14 TX, PG9 RX
        HAL_GPIO_Init(GPIOG, &gpio);
    }
}

static void uart_init(UART_HandleTypeDef *uart, USART_TypeDef *instance) {
    uart->Instance = instance;
    uart->Init.BaudRate = 115200;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart) != HAL_OK) { Error_Handler(); }
}

static void transmit(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size,
                     volatile uint32_t *errors) {
    if (HAL_UART_Transmit(uart, data, size, 20) != HAL_OK) { *errors += 1; }
}

// 直接轮询 SR/DR, 绕开 HAL 状态机, 便于区分"没有字节"与"HAL 状态机卡住".
static void echo(UART_HandleTypeDef *uart, volatile uint32_t *received,
                 volatile uint32_t *errors, volatile uint32_t *framing) {
    USART_TypeDef *reg = uart->Instance;
    const uint32_t sr = reg->SR;
    if ((sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0u) {
        // 读取 SR 后再读 DR 完成 PE/FE/NE/ORE 清除.
        (void)reg->DR;
        *framing += 1;
    }
    if ((sr & USART_SR_RXNE) != 0u) {
        const uint8_t byte = (uint8_t)(reg->DR & 0xffu);
        *received += 1;
        reg->DR = byte; // 回显
        while ((reg->SR & USART_SR_TXE) == 0u) {}
        (void)errors;
    }
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) { HAL_TIM_IRQHandler(&htim14); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer) {
    if (timer->Instance == TIM14) { HAL_IncTick(); }
}
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }

int main(void) {
    HAL_Init();
    clock_init();
    uart_init(&huart1, USART1);
    uart_init(&huart6, USART6);
    uint32_t last = HAL_GetTick() - 500u;
    static const uint8_t hello1[] = "\r\n[INFANTRY UART1] 115200 8N1 ECHO READY\r\n";
    static const uint8_t hello6[] = "\r\n[INFANTRY UART6] 115200 8N1 ECHO READY\r\n";
    while (1) {
        const uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last) >= 500u) {
            last = now;
            transmit(&huart1, hello1, sizeof(hello1) - 1u, &g_uart1_tx_errors);
            transmit(&huart6, hello6, sizeof(hello6) - 1u, &g_uart6_tx_errors);
        }
        echo(&huart1, &g_uart1_echo_bytes, &g_uart1_tx_errors, &g_uart1_framing);
        echo(&huart6, &g_uart6_echo_bytes, &g_uart6_tx_errors, &g_uart6_framing);
    }
}
