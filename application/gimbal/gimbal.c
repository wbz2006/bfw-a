#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "mpu6500.h"
#include "message_center.h"
#include "general_def.h"
#include "jscope.h"
#include "serial_print.h"
#include "bsp_dwt.h"

#include <math.h>


static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
static uint32_t gimbal_sine_active_axes, gimbal_sine_start_tick;
static float gimbal_sine_amplitude, gimbal_sine_frequency, gimbal_sine_duration;
volatile uint8_t gimbal_startup_ready;
volatile int jscope_result;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

volatile GimbalSineTest_s gimbal_sine_test = {
    .amplitude_deg = 60.0f,
    .frequency_hz = 0.35f,
    .duration_s = 52.0f,
};
GimbalSineSample_s gimbal_sine_samples[1280];
volatile uint32_t gimbal_sine_sample_count;
volatile uint32_t gimbal_trace_active;
volatile uint32_t gimbal_trace_period_ms = 50U;

typedef struct
{
    float ramp;
    float phase;
} GimbalSineWave_s;

static GimbalSineWave_s GimbalSineWaveAt(float elapsed_s)
{
    float wave_time = fminf(elapsed_s, gimbal_sine_duration);
    float ramp = fminf(gimbal_sine_frequency *
                           fminf(wave_time, gimbal_sine_duration - wave_time),
                       1.0f);

    return (GimbalSineWave_s){
        .ramp = ramp,
        .phase = 2.0f * PI * gimbal_sine_frequency * wave_time,
    };
}

void GimbalSineTestUpdate(float *yaw_ref, float *pitch_ref)
{
    uint32_t axes = gimbal_sine_test.axes;

    if (axes == 0U)
    {
        gimbal_sine_active_axes = 0U;
        return;
    }

    if (axes != gimbal_sine_active_axes)
    {
        gimbal_sine_amplitude = gimbal_sine_test.amplitude_deg;
        gimbal_sine_frequency = gimbal_sine_test.frequency_hz;
        gimbal_sine_duration = gimbal_sine_test.duration_s;
        // 调试入口的运动范围限制, 同时保证 pitch 正弦不会被机械限位截断。
        if (axes > 3U || gimbal_sine_amplitude <= 0.0f || gimbal_sine_amplitude > 60.0f ||
            gimbal_sine_frequency <= 0.0f || gimbal_sine_frequency > 1.0f ||
            gimbal_sine_duration <= 0.0f || gimbal_sine_duration > 60.0f ||
            ((axes & 2U) &&
             (gimbal_IMU_data->Pitch - gimbal_sine_amplitude < PITCH_MIN_ANGLE ||
              gimbal_IMU_data->Pitch + gimbal_sine_amplitude > PITCH_MAX_ANGLE)))
        {
            gimbal_sine_test.axes = 0U;
            gimbal_sine_active_axes = 0U;
            return;
        }

        gimbal_sine_active_axes = axes;
        gimbal_sine_start_tick = HAL_GetTick();
        gimbal_sine_test.yaw_center_deg = gimbal_IMU_data->YawTotalAngle;
        gimbal_sine_test.pitch_center_deg = gimbal_IMU_data->Pitch;
        gimbal_sine_sample_count = 0U;
    }

    float elapsed = (uint32_t)(HAL_GetTick() - gimbal_sine_start_tick) * 0.001f;
    gimbal_sine_test.elapsed_s = elapsed;
    GimbalSineWave_s wave = GimbalSineWaveAt(elapsed);
    float offset = gimbal_sine_amplitude * wave.ramp * sinf(wave.phase);
    *yaw_ref = gimbal_sine_test.yaw_center_deg + ((axes & 1U) ? offset : 0.0f);
    *pitch_ref = gimbal_sine_test.pitch_center_deg + ((axes & 2U) ? offset : 0.0f);

    if (elapsed >= gimbal_sine_duration)
    {
        gimbal_sine_test.axes = 0U;
        gimbal_sine_active_axes = 0U;
    }
}

void GimbalSpeedFeedforwardUpdate(float yaw_ref, uint8_t enabled, float *yaw_speed_feedforward)
{
    static uint32_t last_tick;
    static float last_yaw_ref;
    static uint8_t was_enabled;

    if (!enabled || !was_enabled)
    {
        last_tick = DWT->CYCCNT;
        last_yaw_ref = yaw_ref;
        was_enabled = enabled;
        *yaw_speed_feedforward = 0.0f;
        return;
    }

    float dt = DWT_GetDeltaT(&last_tick);
    float yaw_speed_feedforward_ref = (yaw_ref - last_yaw_ref) / dt;
    last_yaw_ref = yaw_ref;
    *yaw_speed_feedforward = fminf(250.0f, fmaxf(-250.0f, 0.90f * yaw_speed_feedforward_ref));
}


void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 40,
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 250,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 12000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            .speed_feedforward_ptr = &gimbal_cmd_recv.yaw_speed_feedforward,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = SPEED_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 100,
                .Ki = 250,
                .Kd = 0.5f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 80,
                .Ki = 150,
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 6000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[1]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动

    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
    DJIMotorStop(yaw_motor);
    DJIMotorStop(pitch_motor);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if (!INS_IsReady())
    {
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        return;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    if (!gimbal_startup_ready)
    {
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
    }
    else switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }
    //SEGGER_RTT_printf(0,"0:%d,1:%d,2:%d\n",(int16_t)gimbal_IMU_data->Gyro[0], (int16_t)gimbal_IMU_data->Gyro[1],(int16_t)gimbal_IMU_data->Gyro[2]);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);

    // 正弦验收默认50ms; 到位抖动诊断使用5ms, 复用同一块采样缓冲。
    static uint32_t last_sample_tick;
    if ((gimbal_sine_test.axes != 0U || gimbal_trace_active != 0U) &&
        gimbal_sine_sample_count < 1280U &&
        (uint32_t)(HAL_GetTick() - last_sample_tick) >= gimbal_trace_period_ms)
    {
        last_sample_tick = HAL_GetTick();
        GimbalSineSample_s *sample = &gimbal_sine_samples[gimbal_sine_sample_count++];
        sample->time_s = gimbal_sine_test.axes ? gimbal_sine_test.elapsed_s : HAL_GetTick() * 0.001f;
        sample->yaw_ref = yaw_motor->motor_controller.angle_PID.Ref;
        sample->yaw_measure = yaw_motor->motor_controller.angle_PID.Measure;
        sample->yaw_error = yaw_motor->motor_controller.angle_PID.Err;
        sample->yaw_output = yaw_motor->motor_controller.speed_PID.Output;
        sample->pitch_ref = pitch_motor->motor_controller.angle_PID.Ref;
        sample->pitch_measure = pitch_motor->motor_controller.angle_PID.Measure;
        sample->pitch_error = pitch_motor->motor_controller.angle_PID.Err;
        sample->pitch_output = pitch_motor->motor_controller.speed_PID.Output;
    }

    // VOFA+ FireWater: yaw目标/反馈/误差, pitch目标/反馈/误差, 单位为度.
    static uint32_t last_print_tick;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_print_tick) >= 20U)
    {
        last_print_tick = now;
        SerialPrintf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                     yaw_motor->motor_controller.angle_PID.Ref,
                     yaw_motor->motor_controller.angle_PID.Measure,
                     yaw_motor->motor_controller.angle_PID.Err,
                     pitch_motor->motor_controller.angle_PID.Ref,
                     pitch_motor->motor_controller.angle_PID.Measure,
                     pitch_motor->motor_controller.angle_PID.Err);
    }
}
