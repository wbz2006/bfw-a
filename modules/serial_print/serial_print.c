/**
 * @file serial_print.c
 * @brief 串口打印模块实现
 *
 * 依赖 HAL USART 句柄 (Core/Src/usart.c)。
 * 使用 UART7 @ 115200，阻塞发送 + FreeRTOS 互斥量防重入。
 */
#include "serial_print.h"

#include "usart.h"       /* extern huart7 */
#include "bsp_log.h"     /* LOGINFO / LOGERROR */
#include "FreeRTOS.h"    /* FreeRTOS 基础类型 */
#include "semphr.h"      /* FreeRTOS 互斥量 */

#include <stdarg.h>      /* va_list, va_start, va_end */
#include <stdio.h>       /* vsnprintf */

/* ---- 常量 ---- */
#define PRINT_BUFFER_SIZE 256  /* 格式化缓冲区大小，可按需调整 */

/* ---- 静态变量 ---- */
static uint8_t           print_tx_buf[PRINT_BUFFER_SIZE];  /* 发送缓冲区 */
static SemaphoreHandle_t print_mutex;                      /* 互斥量，保护缓冲区与发送序列 */

/**
 * @brief 初始化串口打印模块
 *
 * 使用 UART7 (115200-8-N-1) 作为打印端口。
 * 该串口已在 MX_UART7_Init() 中由 HAL 初始化，
 * 本模块只发送，不启动 UART7 已配置的 DMA 接收。
 */
void SerialPrintInit(void)
{
    print_mutex = xSemaphoreCreateMutex();
    if (print_mutex == NULL)
    {
        LOGERROR("[serial_print] mutex create failed!");
        return;
    }

    LOGINFO("[serial_print] init done on UART7 @ 115200");
}

/**
 * @brief 格式化并发送字符串
 *
 * 流程: 取互斥量 -> vsnprintf -> HAL_UART_Transmit (阻塞) -> 放互斥量。
 * 互斥量保护整个 "格式化-发送" 序列，确保多任务调用安全。
 * 115200 波特率下 256 字节约 22.2ms 完成。
 */
void SerialPrintf(const char *fmt, ...)
{
    if (print_mutex == NULL)
        return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)print_tx_buf, PRINT_BUFFER_SIZE, fmt, args);
    va_end(args);

    /* vsnprintf 返回负值表示格式化错误 */
    if (len < 0)
    {
        xSemaphoreGive(print_mutex);
        return;
    }
    /* 如果超出缓冲，截断到缓冲区末尾 */
    if (len >= PRINT_BUFFER_SIZE)
        len = PRINT_BUFFER_SIZE - 1;

    HAL_UART_Transmit(&huart7, print_tx_buf, (uint16_t)len, 100);

    xSemaphoreGive(print_mutex);
}

/**
 * @brief 发送原始字节数据
 *
 * @param data 数据指针
 * @param len  数据长度
 */
void SerialPrint(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || print_mutex == NULL)
        return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    HAL_UART_Transmit(&huart7, (uint8_t *)data, len, 100);

    xSemaphoreGive(print_mutex);
}
