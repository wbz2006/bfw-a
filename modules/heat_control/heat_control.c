#include "heat_control.h"
#include "bsp_dwt.h"
#include "string.h"

static float HeatControlLimitRate(float rate, float max_rate)
{
    if (!(rate > 0.0f))
        return 0.0f;
    return rate > max_rate ? max_rate : rate;
}

void HeatControlInit(HeatControlInstance *instance, const HeatControl_Init_Config_s *config)
{
    if (instance == NULL)
        return;

    memset(instance, 0, sizeof(*instance));
    instance->bullet_heat = (config != NULL && config->bullet_heat > 0.001f) ?
                                config->bullet_heat : HEAT_CONTROL_BULLET_HEAT;
    instance->max_shoot_rate = (config != NULL && config->max_shoot_rate > 0.001f) ?
                                    config->max_shoot_rate : HEAT_CONTROL_DEFAULT_MAX_RATE;
    instance->data_sub = SubRegister(HEAT_CONTROL_DATA_TOPIC, sizeof(HeatControlData_s));
    /* 初始化时间基准，避免第一次计算把系统启动以来的时间算入射击时间。 */
    DWT_GetDeltaT(&instance->dwt_cnt);
}

void HeatControlReset(HeatControlInstance *instance)
{
    if (instance == NULL)
        return;

    instance->shoot_speed = 0.0f;
    instance->settle_elapsed = 0.0f;
    instance->was_firing = 0u;
}

float HeatControlCalculate(HeatControlInstance *instance, uint8_t firing)
{
    HeatControlData_s data;
    float dt;

    if (instance == NULL || instance->data_sub == NULL)
        return 0.0f;

    dt = DWT_GetDeltaT(&instance->dwt_cnt);
    /* 每次调用都锁存最新消息，目标射频按本地 100 ms 周期更新。 */
    if (SubGetMessage(instance->data_sub, &data))
    {
        instance->current_heat = (float)data.current_heat;
        instance->heat_limit = (float)data.heat_limit;
        instance->cooling_rate = (float)data.cooling_rate;
        instance->data_valid = data.valid;
    }

    if (!firing)
    {
        HeatControlReset(instance);
        return 0.0f;
    }

    /* 停射条件每次调用都检查，不能等待下一个射频更新周期。 */
    if (!instance->data_valid || instance->heat_limit <= 0.0f || instance->bullet_heat <= 0.001f ||
        instance->current_heat < 0.0f || instance->current_heat >= instance->heat_limit ||
        instance->cooling_rate < 0.0f)
    {
        HeatControlReset(instance);
        return 0.0f;
    }

    /* 进入连射时立即给出一次目标射速，避免首发额外等待一个更新周期。 */
    if (!instance->was_firing)
    {
        instance->was_firing = 1u;
        instance->settle_elapsed = HEAT_CONTROL_SETTLE_PERIOD_S;
    }

    /* 保持 100 ms 更新周期；它不等于热量规划窗口，也不与裁判冷却相位同步。 */
    instance->settle_elapsed += dt;
    if (instance->settle_elapsed < HEAT_CONTROL_SETTLE_PERIOD_S)
        return instance->shoot_speed;
    while (instance->settle_elapsed >= HEAT_CONTROL_SETTLE_PERIOD_S)
        instance->settle_elapsed -= HEAT_CONTROL_SETTLE_PERIOD_S;

    /*
     * 将剩余热量分配到独立规划窗口，扣除一个冷却周期的冷却贡献：
     * rate = cooling / bullet_heat
     *      + (available_heat - reserve_heat - cooling * settle_period)
     *        / (bullet_heat * plan_period)。
     * 在 available_heat = reserve_heat + cooling * settle_period 附近，
     * 射频自然收敛到 cooling / bullet_heat；更靠近上限时继续降速。
     */
    instance->shoot_speed =
        instance->cooling_rate / instance->bullet_heat +
        (instance->heat_limit - instance->current_heat -
         HEAT_CONTROL_RESERVE_SHOTS * instance->bullet_heat -
         instance->cooling_rate * HEAT_CONTROL_SETTLE_PERIOD_S) /
            (instance->bullet_heat * HEAT_CONTROL_PLAN_PERIOD_S);

    instance->shoot_speed = HeatControlLimitRate(instance->shoot_speed, instance->max_shoot_rate);
    return instance->shoot_speed;
}
