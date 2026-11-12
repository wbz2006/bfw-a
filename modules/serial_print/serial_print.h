/**
 * @file serial_print.h
 * @brief 基于 HAL UART 的串口打印模块
 *
 * 使用 UART7 (115200-8-N-1) 提供类似 printf 的串口输出功能。
 * 在 MX_UART7_Init() 后调用 SerialPrintInit() 创建互斥量，
 * 即可通过 SerialPrintf() / SerialPrint() 在任何 FreeRTOS 任务中打印调试信息。
 * 以 "%.1f,%.1f,...\n" 的 CSV 格式输出可用于 VOFA+ (FireWater) 等上位机绘制波形。
 *
 * @note 发送为阻塞模式 (HAL_UART_Transmit, 超时 100ms)，
 *       内部使用 FreeRTOS 互斥量保护格式化缓冲和发送序列，多任务调用安全。
 *       发送接口仅用于调度器启动后的任务上下文，不可在中断中调用。
 *
 * @example
 *    // ---- 初始化 (在 BSPInit 中调用一次) ----
 *    SerialPrintInit();
 *
 *    // ---- 任意任务中打印 ----
 *    SerialPrintf("Hello! tick = %lu\r\n", xTaskGetTickCount());
 *    SerialPrintf("motor: rpm=%d, temp=%.1f\r\n", rpm, temp);
 *    SerialPrint((uint8_t *)"OK\r\n", 4);
 *
 *    // ---- 波形输出 (VOFA+ FireWater 格式, 每行一组采样) ----
 *    SerialPrintf("%.1f,%.1f,%.1f\n", yaw, yaw_ref, error);
 */
#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H

#include <stdint.h>

/**
 * @brief 初始化串口打印模块的互斥量，不启用 DMA 接收
 */
void SerialPrintInit(void);

/**
 * @brief 通过串口发送格式化字符串（类似 printf）
 *
 * @note 内部使用 vsnprintf 格式化到静态缓冲区 (256 字节)，
 *       然后阻塞发送；互斥量保证多任务调用安全。
 *
 * @param fmt  格式化字符串
 * @param ...  可变参数列表
 */
void SerialPrintf(const char *fmt, ...);

/**
 * @brief 通过串口发送原始字节
 *
 * @param data 要发送的数据指针
 * @param len  数据长度 (字节)
 */
void SerialPrint(const uint8_t *data, uint16_t len);

#endif // SERIAL_PRINT_H
