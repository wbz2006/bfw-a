#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef struct
{
    uint32_t axes; // 0: 关闭, 1: yaw, 2: pitch, 3: 双轴; 最后写入以启动测试
    float amplitude_deg;
    float frequency_hz;
    float duration_s;
    float elapsed_s;
    float yaw_center_deg;
    float pitch_center_deg;
} GimbalSineTest_s;

extern volatile GimbalSineTest_s gimbal_sine_test;

typedef struct
{
    float time_s;
    float yaw_ref, yaw_measure, yaw_error, yaw_output;
    float pitch_ref, pitch_measure, pitch_error, pitch_output;
} GimbalSineSample_s;

extern GimbalSineSample_s gimbal_sine_samples[1280];
extern volatile uint32_t gimbal_sine_sample_count;
extern volatile uint32_t gimbal_trace_active;
extern volatile uint32_t gimbal_trace_period_ms;
extern volatile uint8_t gimbal_startup_ready;

// 正弦函数只更新角度目标; 速度前馈由通用Yaw目标变化率单独计算。
void GimbalSineTestUpdate(float *yaw_ref, float *pitch_ref);
void GimbalSpeedFeedforwardUpdate(float yaw_ref, uint8_t enabled, float *yaw_speed_feedforward);

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H
