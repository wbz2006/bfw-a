#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#include "message_center.h"
#include "stdint.h"

/* 裁判系统热量数据主题，由底盘应用发布，热量控制模块订阅。 */
#define HEAT_CONTROL_DATA_TOPIC "heat_control_data"

/* 单发热量统一配置入口，应与裁判系统实际配置一致。 */
#define HEAT_CONTROL_BULLET_HEAT 10.0f
#define HEAT_CONTROL_DEFAULT_MAX_RATE 30.0f
#define HEAT_CONTROL_RESERVE_SHOTS 1.0f
#define HEAT_CONTROL_SETTLE_PERIOD_S 0.1f /* 裁判系统按 10 Hz 结算冷却 */
#define HEAT_CONTROL_PLAN_PERIOD_S 0.5   /* 热量规划窗口；越长越早降速，与冷却周期独立 */
#define HEAT_CONTROL_GAME_STATE_TIMEOUT_MS 300U
#define HEAT_CONTROL_POWER_HEAT_TIMEOUT_MS 300U

/**
 * @brief 从裁判系统转发的热量数据
 *
 * current_heat 对应 0x0202 的 shooter_17mm_1_barrel_heat；
 * heat_limit 和 cooling_rate 对应 0x0201 的热量上限、每秒冷却值。
 */
typedef struct
{
    uint16_t current_heat;
    uint16_t heat_limit;
    uint16_t cooling_rate;
    uint32_t game_state_age_ms;
    uint32_t power_heat_age_ms;
    uint8_t valid;
} HeatControlData_s;

typedef enum
{
    HEAT_CONTROL_NORMAL = 0,
    HEAT_CONTROL_OVER_LIMIT_LOCKED,
} HeatControlState_e;

/** 热量控制初始化配置。 */
typedef struct
{
    float bullet_heat;
    float max_shoot_rate;
} HeatControl_Init_Config_s;

/** 热量控制实例。 */
typedef struct
{
    float shoot_speed;     /* 目标射速，单位 发/s */
    float current_heat;    /* Q1 */
    float heat_limit;      /* Q0 */
    float cooling_rate;    /* 热量冷却速度/s */
    float bullet_heat;     /* 热量/发 */
    float max_shoot_rate;  /* 物理射速上限，单位 发/s */
    float settle_elapsed;  /* 本地射频更新累计时间，不代表裁判冷却相位 */
    float local_heat;      /* 本地保守热量估计，用于覆盖裁判回传延迟 */
    float data_age_elapsed;
    uint32_t dwt_cnt;      /* 实例独立的 DWT 时间戳 */
    uint8_t data_valid;
    uint8_t local_heat_valid;
    uint8_t was_firing;    /* 上一次调用是否处于连射，用于进入连射时立即结算 */
    HeatControlState_e state;
    Subscriber_t *data_sub;
} HeatControlInstance;

/**
 * @brief 初始化热量控制实例并订阅裁判热量数据。
 * @param instance 热量控制实例
 * @param config   初始化配置，可传 NULL 使用默认参数
 */
void HeatControlInit(HeatControlInstance *instance, const HeatControl_Init_Config_s *config);

/**
 * @brief 重置射频输出和 10 Hz 结算计时。
 * @param instance 热量控制实例
 */
void HeatControlReset(HeatControlInstance *instance);

/**
 * @brief 读取消息中心最新数据并计算目标射速。
 * @param instance 热量控制实例
 * @param firing   当前是否处于连续发射状态
 * @return 目标射速，单位为发/秒
 */
float HeatControlCalculate(HeatControlInstance *instance, uint8_t firing);

/** 判断指定数量的离散弹丸是否有热量预算。 */
uint8_t HeatControlCanFire(const HeatControlInstance *instance, uint8_t shots);

/** 在单发或三连发真正下发后登记本地热量消耗。 */
void HeatControlRegisterShots(HeatControlInstance *instance, uint8_t shots);

/** 获取当前应用注册的热量控制实例，供 shoot 应用登记实际离散发弹。 */
HeatControlInstance *HeatControlGetInstance(void);

#endif
