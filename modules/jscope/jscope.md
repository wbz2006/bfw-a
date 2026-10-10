# J-Scope 波形输出模块

J-Scope 模块通过独立的 SEGGER RTT 上行缓冲区发送浮点采样帧。
缓冲区名称为 `JScope_f4f4f4f4`，表示每个采样帧包含 4 个 32 位浮点通道。
`f4` 表示一个 4 字节浮点数，因此需要重复四次。`t4` 表示 4 字节微秒时间戳，
不是通道数量；本模块不发送时间戳。
模块不会修改 RTT 通道 0，因此可以和 `SEGGER_RTT_printf` 日志同时使用。

在 `RobotInit()` 中，`BSPInit()` 完成 RTT 初始化后调用一次，随后再初始化应用和任务。
`JScopeInit()` 不会重新调用 `SEGGER_RTT_Init()`，避免清除已配置的日志通道；
初始化成功后也不要再次调用 `SEGGER_RTT_Init()`，否则会清除 J-Scope 通道。

```c
#include "jscope/jscope.h"

if (JScopeInit() < 0)
{
    // RTT 上行缓冲区 1 不可用。
}
```

`JScopePrintf` 是采样帧写入函数，不是文本格式化函数。每个支持的格式转换符
按出现顺序对应一个通道，整数会转换为浮点数，未使用的通道会填充为 0：

```c
// 放在已有的周期任务函数中，每次任务调用发送一帧，无需添加循环。
JScopePrintf("%f %f", motor->speed, motor->current);
```

如果要发送完整的波形缓冲区，可以使用 `JScopeWriteSamples`。该函数接收按采样点
排列的数据，支持 1 到 4 个通道，缺少的通道会填充为 0。例如，3 个采样点、2 个
通道的数据排列为 `{ch0_s0, ch1_s0, ch0_s1, ch1_s1, ch0_s2, ch1_s2}`。
批量数据必须一次性放入 J-Scope RTT 缓冲区；默认缓冲区大小为 1024 字节，发送时
需要保持 J-Scope 已连接并正在读取数据。

在 J-Scope 中选择 RTT 采样模式。软件会自动查找以 `JScope_` 开头的上行通道，
本模块使用上行通道 1，名称为 `JScope_f4f4f4f4`。
如果出现 `No RTT Up-Channel named "JScope_%Format%" found`，检查 `JScopeInit()`
是否已执行并返回 0，以及初始化后是否有代码重新调用了 `SEGGER_RTT_Init()`。
也可在调试器中查看 `_SEGGER_RTT.aUp[1].sName`，应为 `JScope_f4f4f4f4`。
模块使用小端序 IEEE-754 `float` 数据表示，每个采样帧包含 4 个数值。

协议依据：[SEGGER J-Scope 官方手册：RTT 通道命名](https://kb.segger.com/UM08028_J-Scope#RTT_channel_naming_convention)。
