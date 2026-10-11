#ifndef SHOOT_H
#define SHOOT_H

#include <stdint.h>

/* 摩擦轮保护参数，供整车调试时统一调整。 */
#define FRICTION_READY_RATIO        0.90f
#define FRICTION_READY_TIME_S       0.10f
#define FRICTION_DROP_RATIO         0.85f
#define FRICTION_EMERGENCY_RATIO    0.70f
#define FRICTION_RECOVER_RATIO      0.92f
#define FRICTION_DROP_TIME_S        0.05f

#define FRICTION_SPEED_TABLE_SIZE 31U

typedef enum
{
    FRICTION_STATE_OFF = 0,
    FRICTION_STATE_SPIN_UP,
    FRICTION_STATE_READY,
    FRICTION_STATE_DEGRADED,
} FrictionState_e;

typedef struct
{
    FrictionState_e state;
    float last_target_speed;
    float target_speed;
    float speed_ratio;
} FrictionControl_s;

typedef struct
{
    uint32_t dwt_cnt;
    float dt;
    float ready_elapsed;
    float drop_elapsed;
    float loader_elapsed;
} ShootTiming_s;

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

#endif // SHOOT_H
