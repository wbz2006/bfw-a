# J-Scope 波形输出模块

本模块通过 SEGGER RTT 向 J-Scope 发送波形数据。应用层在已有的周期任务中调用 `JScopePrintf()`，每次调用发送一个采样点，不需要另建任务或添加无限循环。

## 当前工程配置

| 项目 | 当前配置 |
| --- | --- |
| RTT 上行通道 | 1 |
| RTT 通道名称 | `JScope_f4f4f4f4` |
| 波形通道数 | 固定 4 个 |
| 帧数据 | 4 个 32 位 `float`，共 16 字节 |
| 缓冲区大小 | 1024 字节 |
| 缓冲区满时 | 丢弃本次采样帧，返回 0，不等待主机读取 |

每个 `f4` 表示一个 4 字节浮点通道。模块沿用本工程已经验证可工作的通道 1 和浮点协议。RTT 通道 0 继续用于 `SEGGER_RTT_printf()` 日志。

Makefile 已包含 `modules/jscope/jscope.c` 和头文件搜索路径 `-Imodules/jscope`，应用文件直接使用：

```c
#include "jscope.h"
```

## 初始化位置

当前工程在 `application/robot.c` 的 `RobotInit()` 中初始化一次，放在 `BSPInit()` 之后、应用初始化之前：

```c
void RobotInit()
{
    __disable_irq();

    BSPInit();
    JScopeInit();
    RobotCMDInit();
    GimbalInit();
    ShootInit();
    ChassisInit();

    __enable_irq();
}
```

`BSPInit()` 内的日志初始化负责调用 `SEGGER_RTT_Init()`；`JScopeInit()` 仅配置波形通道。返回 0 表示初始化成功，负数表示配置失败。

初始化后不要再次调用 `SEGGER_RTT_Init()`，否则已配置的波形通道会被清除。也不要同时保留原来的 `JScope_Init()` 去配置同一个 RTT 通道。

## 当前使用示例：发射与摩擦轮波形

实际发送位置是 `application/shoot/shoot.c` 的 `ShootTask()`，每个机器人控制周期发送：

```c
JScopePrintf("%f %f %f %f",
             shoot_cmd_recv.shoot_rate,
             shoot_cmd_recv.cooling_rate / HEAT_CONTROL_BULLET_HEAT,
             friction_l->measure.speed_aps,
             friction_r->measure.speed_aps);
```

| 波形通道（从 0 开始） | 数据 | 含义 |
| --- | --- | --- |
| 0 | `shoot_cmd_recv.shoot_rate` | 热量控制目标射频 |
| 1 | `cooling_rate / bullet_heat` | 冷却能力折算射频 |
| 2 | `friction_l->measure.speed_aps` | 左摩擦轮实际速度 |
| 3 | `friction_r->measure.speed_aps` | 右摩擦轮实际速度 |

`RobotTask()` 的调度循环使用 `osDelay(5)`，因此这组波形的采样频率约为 200 Hz。裁判系统热量上限和当前热量仍通过 `heat_control_data` 主题传递，若需要观察原始热量，应临时将通道映射替换为 `shoot_cmd_recv.heat_limit` 和 `shoot_cmd_recv.current_heat`。

修改要观察的信号时，替换这次调用中的参数即可。应保持各通道含义固定；多个任务向同一个 RTT 通道轮流发送不同信号，会让 J-Scope 把它们当成同一组波形的连续采样点。

## 格式和通道限制

调用形式类似 `SEGGER_RTT_printf()`，但这里的格式转换符用来依次选取波形数值，不输出文本。

```c
// 浮点信号：float 或 double 参数均可，最终发送为 float。
JScopePrintf("%f %f", pitch, yaw);

// 整数信号：按整数类型读取，再转换成 float 发送。
JScopePrintf("%d %u", (int)speed, (unsigned int)heat);
```

支持 `%f`、`%d`、`%i`、`%u`、`%x`、`%X`、`%c`。参数类型必须与格式匹配，例如整数使用 `%f` 时需要先转换成 `float`，不能直接传入整数。

格式中的普通文字、空格和换行不会发送；`%.2f` 中的小数位设置也不会对数值进行截断或舍入。`%s`、`%e`、`%g` 和 `*` 动态宽度等格式不受支持，应使用上述简单格式。

一次最多发送 4 个数值，少于 4 个时其余通道自动填 0；超过 4 个时整帧不发送，返回 0。连续调用两次会产生两个采样点，不会增加通道数。

整数也以 `float` 传输，因此较大的整数可能损失精度；例如大于 `2^24` 后不能保证所有整数都精确表示。

增加通道数需要同步修改通道名称、帧长度及写入实现，仅修改 `JSCOPE_CHANNEL_COUNT` 不足以完成扩展。

## 连接和运行

1. 编译并烧录当前工程，让程序持续运行。
2. 在 J-Scope 中选择 RTT Sampling，连接对应的 J-Link 和目标芯片。
3. 确认识别到 `JScope_f4f4f4f4`，然后开始采样。
4. 观察通道 0、1 判断目标射频是否收敛到冷却能力，观察通道 2、3 判断两侧摩擦轮是否掉速。

本工程已验证 J-Scope 可以与 Live Watch 同时工作，无需仅因启用了 Live Watch 就关闭调试。模块没有发送时间戳；若 J-Scope 版本提供采样周期或时间轴设置，应按当前发送节拍约 5 ms 配置。


