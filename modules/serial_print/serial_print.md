# UART7 串口打印

模块移植自 `smart-rescue/modules/serial_print`，保留 `SerialPrintInit()`、
`SerialPrintf()` 和 `SerialPrint()` 接口。`BSPInit()` 创建互斥量，发送接口在
FreeRTOS 任务中调用。发送采用 HAL 阻塞方式，不使用 TX DMA，也不启动 RX DMA。

## 接线与设置

- PE8（UART7 TX）接 USB 转串口模块的 RX，双方 GND 共地，使用 3.3 V TTL 电平。
- 仅查看输出时无需连接 PE7（UART7 RX）。
- 串口参数为 **115200、8 数据位、无校验、1 停止位、无流控**。
- VOFA+ 使用 **FireWater** 协议，也可用串口助手查看文本。

## 云台输出

`GimbalTask()` 每隔至少 20 ms 输出一行 CSV，约 50 Hz：

| 列 | 内容 | 来源 | 单位 |
| --- | --- | --- | --- |
| 1 | Yaw 目标角度 | `yaw_motor->motor_controller.angle_PID.Ref` | 度 |
| 2 | Yaw 实际角度 | `yaw_motor->motor_controller.angle_PID.Measure` | 度 |
| 3 | Yaw 角度误差 | `yaw_motor->motor_controller.angle_PID.Err` | 度 |
| 4 | Pitch 目标角度 | `pitch_motor->motor_controller.angle_PID.Ref` | 度 |
| 5 | Pitch 实际角度 | `pitch_motor->motor_controller.angle_PID.Measure` | 度 |
| 6 | Pitch 角度误差 | `pitch_motor->motor_controller.angle_PID.Err` | 度 |

两轴都读取角度环 PID 实际计算的数据。Yaw 反馈为 IMU 累计角度，Pitch 反馈为
IMU 俯仰角；误差为目标减反馈。每个值保留一位小数，以 `\r\n` 结束一行，例如：

```text
120.0,118.5,1.5,10.0,9.0,1.0
```

停机时仍周期输出，PID 目标为最后设置的参考值。Makefile 已启用 newlib-nano 的
`_printf_float`，支持 `%f`。同一串口用于绘图时，应保持每行都是六个数值。

## 其他任务中打印

```c
#include "serial_print.h"

SerialPrintf("value=%.1f\r\n", value);
SerialPrint((const uint8_t *)"OK\r\n", 4);
```

格式化缓冲区为 256 字节，超长输出截断为 255 字节。发送超时为 100 ms，
互斥量保护格式化及发送过程；不要在中断或关闭中断的初始化阶段调用发送接口。
