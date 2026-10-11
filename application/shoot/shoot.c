#include "shoot.h"
#include "robot_def.h"

#include "math.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "jscope.h"
#include "heat_control.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l;
static DJIMotorInstance *friction_r;
static DJIMotorInstance *loader;

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Shoot_Upload_Data_s shoot_feedback_data;
static float loader_target_angle;

#define LOADER_ONE_BULLET_MOTOR_ANGLE (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
#define LOADER_ANGLE_TOLERANCE 5.0f

static ShootTiming_s shoot_timing;

static FrictionControl_s friction_control = {
    .state = FRICTION_STATE_OFF,
};

/*
 * 摩擦轮电机参考值配置表，索引为 Bullet_Speed_e 的数值。
 * 这些值是电机标定值，不是枪口实际初速；未标定档位保持为 0。
 */
static const float friction_target_speed_table[FRICTION_SPEED_TABLE_SIZE] = {
    [SMALL_AMU_15] = 4000.0f,
    [SMALL_AMU_18] = 32000.0f,
    [SMALL_AMU_30] = 8000.0f,
};

void ShootInit(void)
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508,
    };

    friction_config.can_init_config.tx_id = 3;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 4;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = 10,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };

    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    DWT_GetDeltaT(&shoot_timing.dwt_cnt);
}

static uint8_t ShootStartDiscrete(uint8_t shots)
{
    HeatControlInstance *heat_control = HeatControlGetInstance();

    if (heat_control == NULL || !HeatControlCanFire(heat_control, shots))
        return 0u;

    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    loader_target_angle = loader->measure.total_angle +
                          (float)shots * LOADER_ONE_BULLET_MOTOR_ANGLE;
    DJIMotorSetRef(loader, loader_target_angle);
    HeatControlRegisterShots(heat_control, shots);
    return 1u;
}

/* 机器人发射机构控制核心任务 */
void ShootTask(void)
{
    static loader_mode_e last_load_mode = LOAD_STOP;
    uint32_t bullet_speed_index;
    uint8_t friction_ready;
    uint8_t loader_busy;
    uint8_t firing_requested;
    uint8_t loader_blocked;

    shoot_timing.dt = DWT_GetDeltaT(&shoot_timing.dwt_cnt);
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    bullet_speed_index = (uint32_t)shoot_cmd_recv.bullet_speed;
    friction_control.target_speed =
        shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
                shoot_cmd_recv.friction_mode == FRICTION_ON &&
                bullet_speed_index < FRICTION_SPEED_TABLE_SIZE
            ? friction_target_speed_table[bullet_speed_index]
            : 0.0f;
    DJIMotorSetRef(friction_l, friction_control.target_speed);
    DJIMotorSetRef(friction_r, friction_control.target_speed);
    /* 摩擦轮状态机：达速后保持一段时间才允许拨弹，掉速后重新等待恢复。 */
    {
        FrictionControl_s *control = &friction_control;
        float left_ratio;
        float right_ratio;

        if (friction_control.target_speed <= 0.0f)
        {
            control->state = FRICTION_STATE_OFF;
            shoot_timing.ready_elapsed = 0.0f;
            shoot_timing.drop_elapsed = 0.0f;
            control->speed_ratio = 0.0f;
            control->last_target_speed = 0.0f;
        }
        else
        {
            if (fabsf(friction_control.target_speed - control->last_target_speed) > 0.001f)
            {
                control->state = FRICTION_STATE_SPIN_UP;
                shoot_timing.ready_elapsed = 0.0f;
                shoot_timing.drop_elapsed = 0.0f;
                control->last_target_speed = friction_control.target_speed;
            }

            left_ratio = fabsf(friction_l->measure.speed_aps) /
                         friction_control.target_speed;
            right_ratio = fabsf(friction_r->measure.speed_aps) /
                          friction_control.target_speed;
            control->speed_ratio = left_ratio < right_ratio ? left_ratio : right_ratio;

            switch (control->state)
            {
            case FRICTION_STATE_SPIN_UP:
                if (control->speed_ratio >= FRICTION_READY_RATIO)
                {
                    shoot_timing.ready_elapsed += shoot_timing.dt;
                    if (shoot_timing.ready_elapsed >= FRICTION_READY_TIME_S)
                    {
                        control->state = FRICTION_STATE_READY;
                        shoot_timing.drop_elapsed = 0.0f;
                    }
                }
                else
                {
                    shoot_timing.ready_elapsed = 0.0f;
                }
                break;

            case FRICTION_STATE_READY:
                if (control->speed_ratio < FRICTION_DROP_RATIO)
                {
                    if (control->speed_ratio < FRICTION_EMERGENCY_RATIO)
                    {
                        control->state = FRICTION_STATE_DEGRADED;
                        shoot_timing.ready_elapsed = 0.0f;
                        shoot_timing.drop_elapsed = 0.0f;
                    }
                    else
                    {
                        shoot_timing.drop_elapsed += shoot_timing.dt;
                        if (shoot_timing.drop_elapsed >= FRICTION_DROP_TIME_S)
                        {
                            control->state = FRICTION_STATE_DEGRADED;
                            shoot_timing.ready_elapsed = 0.0f;
                            shoot_timing.drop_elapsed = 0.0f;
                        }
                    }
                }
                else
                {
                    shoot_timing.drop_elapsed = 0.0f;
                }
                break;

            case FRICTION_STATE_DEGRADED:
                if (control->speed_ratio >= FRICTION_RECOVER_RATIO)
                {
                    shoot_timing.ready_elapsed += shoot_timing.dt;
                    if (shoot_timing.ready_elapsed >= FRICTION_READY_TIME_S)
                    {
                        control->state = FRICTION_STATE_READY;
                        shoot_timing.drop_elapsed = 0.0f;
                    }
                }
                else
                {
                    shoot_timing.ready_elapsed = 0.0f;
                }
                break;

            case FRICTION_STATE_OFF:
            default:
                control->state = FRICTION_STATE_SPIN_UP;
                shoot_timing.ready_elapsed = 0.0f;
                shoot_timing.drop_elapsed = 0.0f;
                break;
            }
        }
    }

    friction_ready = friction_control.state == FRICTION_STATE_READY;
    loader_busy = (last_load_mode == LOAD_1_BULLET || last_load_mode == LOAD_3_BULLET) &&
                  fabsf(loader->measure.total_angle - loader_target_angle) >
                      LOADER_ANGLE_TOLERANCE;
    if (loader_busy)
        shoot_timing.loader_elapsed += shoot_timing.dt;
    else
        shoot_timing.loader_elapsed = 0.0f;
    firing_requested = shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
                       (shoot_cmd_recv.load_mode == LOAD_1_BULLET ||
                        shoot_cmd_recv.load_mode == LOAD_3_BULLET ||
                        shoot_cmd_recv.load_mode == LOAD_BURSTFIRE);
    loader_blocked = !friction_ready && (firing_requested || loader_busy);

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        last_load_mode = LOAD_STOP;
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        if (loader_blocked)
            DJIMotorStop(loader);
        else
            DJIMotorEnable(loader);

        if (!loader_blocked)
        {
            switch (shoot_cmd_recv.load_mode)
            {
            case LOAD_STOP:
                if (!loader_busy)
                {
                    DJIMotorOuterLoop(loader, SPEED_LOOP);
                    DJIMotorSetRef(loader, 0.0f);
                    last_load_mode = LOAD_STOP;
                }
                break;

            case LOAD_1_BULLET:
                if (!loader_busy && last_load_mode != LOAD_1_BULLET &&
                    ShootStartDiscrete(1u))
                {
                    last_load_mode = LOAD_1_BULLET;
                }
                break;

            case LOAD_3_BULLET:
                if (!loader_busy && last_load_mode != LOAD_3_BULLET &&
                    ShootStartDiscrete(3u))
                {
                    last_load_mode = LOAD_3_BULLET;
                }
                break;

            case LOAD_BURSTFIRE:
                DJIMotorOuterLoop(loader, SPEED_LOOP);
                DJIMotorSetRef(loader,
                               shoot_cmd_recv.shoot_rate * 360.0f * REDUCTION_RATIO_LOADER /
                                   NUM_PER_CIRCLE);
                last_load_mode = LOAD_BURSTFIRE;
                break;

            case LOAD_REVERSE:
                DJIMotorOuterLoop(loader, SPEED_LOOP);
                DJIMotorSetRef(loader, 0.0f);
                last_load_mode = LOAD_REVERSE;
                break;

            default:
                DJIMotorOuterLoop(loader, SPEED_LOOP);
                DJIMotorSetRef(loader, 0.0f);
                last_load_mode = LOAD_STOP;
                break;
            }
        }
    }

    /* 热量上限、当前热量、目标射频、冷却能力折算射频。 */
    JScopePrintf("%f %f %f %f",
                 shoot_cmd_recv.heat_limit,
                 shoot_cmd_recv.current_heat,
                 shoot_cmd_recv.shoot_rate,
                 shoot_cmd_recv.cooling_rate / HEAT_CONTROL_BULLET_HEAT);


    shoot_feedback_data.friction_left_speed_aps  = friction_l->measure.speed_aps;
    shoot_feedback_data.friction_right_speed_aps = friction_r->measure.speed_aps;
    shoot_feedback_data.loader_speed_aps         = loader->measure.speed_aps;
    shoot_feedback_data.friction_speed_ratio     = friction_control.speed_ratio;
    shoot_feedback_data.friction_ready           = friction_control.state == FRICTION_STATE_READY;

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
