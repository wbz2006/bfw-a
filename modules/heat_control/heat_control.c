#include "heat_control.h"
#include "bsp_dwt.h"
#include "string.h"
#include "math.h"

static HeatControlInstance *active_instance;

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
    active_instance = instance;
    instance->bullet_heat = (config != NULL && config->bullet_heat > 0.001f) ?
                                config->bullet_heat : HEAT_CONTROL_BULLET_HEAT;
    instance->max_shoot_rate = (config != NULL && config->max_shoot_rate > 0.001f) ?
                                    config->max_shoot_rate : HEAT_CONTROL_DEFAULT_MAX_RATE;
    instance->data_sub = SubRegister(HEAT_CONTROL_DATA_TOPIC, sizeof(HeatControlData_s));
    /* 初始化时间基准，避免第一次计算把系统启动以来的时间算入射击时间。 */
    DWT_GetDeltaT(&instance->dwt_cnt);
}

HeatControlInstance *HeatControlGetInstance(void)
{
    return active_instance;
}

void HeatControlReset(HeatControlInstance *instance)
{
    if (instance == NULL)
        return;

    instance->shoot_speed = 0.0f;
    instance->settle_elapsed = 0.0f;
    instance->was_firing = 0u;
}

static void HeatControlUpdateData(HeatControlInstance *instance, float dt)
{
    HeatControlData_s data;

    if (instance->local_heat_valid && instance->cooling_rate > 0.0f)
    {
        instance->local_heat -= instance->cooling_rate * dt;
        if (instance->local_heat < 0.0f)
            instance->local_heat = 0.0f;
    }

    instance->data_age_elapsed += dt;
    if (SubGetMessage(instance->data_sub, &data))
    {
        instance->current_heat = (float)data.current_heat;
        instance->heat_limit = (float)data.heat_limit;
        instance->cooling_rate = (float)data.cooling_rate;
        instance->data_age_elapsed = (float)(data.game_state_age_ms > data.power_heat_age_ms
                                                 ? data.game_state_age_ms
                                                 : data.power_heat_age_ms) * 0.001f;
        instance->data_valid = data.valid &&
                               data.game_state_age_ms <= HEAT_CONTROL_GAME_STATE_TIMEOUT_MS &&
                               data.power_heat_age_ms <= HEAT_CONTROL_POWER_HEAT_TIMEOUT_MS;

        if (!instance->local_heat_valid)
        {
            instance->local_heat = instance->current_heat;
            instance->local_heat_valid = 1u;
        }
        else if (instance->current_heat > instance->local_heat)
        {
            /* 回传值高于本地预测时立即抬高估计，优先保证不超热。 */
            instance->local_heat = instance->current_heat;
        }

        if (instance->data_valid && instance->heat_limit > 0.0f &&
            instance->current_heat >= instance->heat_limit)
        {
            instance->state = HEAT_CONTROL_OVER_LIMIT_LOCKED;
        }
        else if (instance->state == HEAT_CONTROL_OVER_LIMIT_LOCKED &&
                 instance->data_valid && instance->current_heat <= 0.0f)
        {
            /* 规则要求超限锁定后冷却到 0 才解除。 */
            instance->state = HEAT_CONTROL_NORMAL;
            instance->local_heat = 0.0f;
        }
    }

    if (instance->data_age_elapsed >
        (float)(HEAT_CONTROL_GAME_STATE_TIMEOUT_MS > HEAT_CONTROL_POWER_HEAT_TIMEOUT_MS
                    ? HEAT_CONTROL_GAME_STATE_TIMEOUT_MS
                    : HEAT_CONTROL_POWER_HEAT_TIMEOUT_MS) * 0.001f)
    {
        instance->data_valid = 0u;
    }
}

float HeatControlCalculate(HeatControlInstance *instance, uint8_t firing)
{
    float dt;
    float effective_heat;

    if (instance == NULL || instance->data_sub == NULL)
        return 0.0f;

    dt = DWT_GetDeltaT(&instance->dwt_cnt);
    HeatControlUpdateData(instance, dt);

    if (!firing)
    {
        HeatControlReset(instance);
        return 0.0f;
    }

    /* 停射条件每次调用都检查，不能等待下一个射频更新周期。 */
    if (!instance->data_valid || instance->state == HEAT_CONTROL_OVER_LIMIT_LOCKED ||
        instance->heat_limit <= 0.0f || instance->bullet_heat <= 0.001f ||
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
    effective_heat = instance->local_heat > instance->current_heat ?
                         instance->local_heat : instance->current_heat;
    if (effective_heat >= instance->heat_limit)
    {
        HeatControlReset(instance);
        return 0.0f;
    }

    if (instance->settle_elapsed >= HEAT_CONTROL_SETTLE_PERIOD_S)
    {
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
            (instance->heat_limit - effective_heat -
             HEAT_CONTROL_RESERVE_SHOTS * instance->bullet_heat -
             instance->cooling_rate * HEAT_CONTROL_SETTLE_PERIOD_S) /
                (instance->bullet_heat * HEAT_CONTROL_PLAN_PERIOD_S);

        instance->shoot_speed = HeatControlLimitRate(instance->shoot_speed, instance->max_shoot_rate);
    }

    instance->local_heat += instance->shoot_speed * dt * instance->bullet_heat;
    return instance->shoot_speed;
}

uint8_t HeatControlCanFire(const HeatControlInstance *instance, uint8_t shots)
{
    float effective_heat;

    if (instance == NULL || shots == 0u || !instance->data_valid ||
        instance->state == HEAT_CONTROL_OVER_LIMIT_LOCKED ||
        instance->heat_limit <= 0.0f || instance->bullet_heat <= 0.001f)
        return 0u;

    effective_heat = instance->local_heat > instance->current_heat ?
                         instance->local_heat : instance->current_heat;
    return (effective_heat + (float)shots * instance->bullet_heat <= instance->heat_limit) ? 1u : 0u;
}

void HeatControlRegisterShots(HeatControlInstance *instance, uint8_t shots)
{
    if (instance == NULL || shots == 0u || !HeatControlCanFire(instance, shots))
        return;

    instance->local_heat += (float)shots * instance->bullet_heat;
    instance->local_heat_valid = 1u;
}
