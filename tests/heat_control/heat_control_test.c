#include "heat_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Subscriber_t subscriber;
static HeatControlData_s pending_data;
static uint8_t pending;
static float task_dt = 0.005f;

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    assert(strcmp(name, HEAT_CONTROL_DATA_TOPIC) == 0);
    assert(data_len == sizeof(HeatControlData_s));
    pending = 0;
    return &subscriber;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    assert(sub == &subscriber);
    if (!pending)
        return 0;
    memcpy(data_ptr, &pending_data, sizeof(pending_data));
    pending = 0;
    return 1;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    ++*cnt_last;
    return task_dt;
}

static void publish(uint16_t heat, uint16_t limit, uint16_t cooling, uint8_t valid)
{
    pending_data = (HeatControlData_s){heat, limit, cooling, valid};
    pending = 1;
}

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void test_curve(float bullet_heat)
{
    HeatControlInstance instance;
    HeatControl_Init_Config_s config = {bullet_heat, 30.0f};
    HeatControlInit(&instance, &config);
    float previous = 30.0f;
    task_dt = 0.1f;
    for (uint16_t heat = 0; heat <= 500; ++heat)
    {
        publish(heat, 500, 20, 1);
        float rate = HeatControlCalculate(&instance, 1);
        assert(rate >= 0.0f && rate <= 30.0f);
        assert(rate <= previous + 0.001f);
        if (heat < 500)
        {
            float expected = 20.0f / bullet_heat +
                (500.0f - heat - HEAT_CONTROL_RESERVE_SHOTS * bullet_heat -
                 20.0f * HEAT_CONTROL_SETTLE_PERIOD_S) /
                    (bullet_heat * HEAT_CONTROL_PLAN_PERIOD_S);
            expected = fminf(30.0f, fmaxf(0.0f, expected));
            near(rate, expected);
        }
        else
            near(rate, 0.0f);
        previous = rate;
    }
    float reserve = HEAT_CONTROL_RESERVE_SHOTS * bullet_heat;
    uint16_t equilibrium_heat = (uint16_t)(500.0f - reserve - 2.0f);
    publish(equilibrium_heat, 500, 20, 1);
    near(HeatControlCalculate(&instance, 1), 20.0f / bullet_heat);
}

static void test_task_behavior(void)
{
    HeatControlInstance instance;
    HeatControlInit(&instance, NULL);
    task_dt = 0.005f;
    near(HeatControlCalculate(&instance, 1), 0.0f);
    publish(0, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    publish(1000, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 1), 0.0f);
    publish(0, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    publish(0, 1000, 20, 0);
    near(HeatControlCalculate(&instance, 1), 0.0f);
    publish(0, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    publish(990, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    task_dt = 0.1f;
    near(HeatControlCalculate(&instance, 1), 0.0f);
    publish(0, 1000, 20, 1);
    near(HeatControlCalculate(&instance, 0), 0.0f);
    task_dt = 0.005f;
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    publish(990, 1000, 0, 1);
    task_dt = 0.1f;
    near(HeatControlCalculate(&instance, 1), 0.0f);
    publish(0, 1000, 1000, 1);
    near(HeatControlCalculate(&instance, 1), HEAT_CONTROL_DEFAULT_MAX_RATE);
    near(HeatControlCalculate(NULL, 1), 0.0f);
}

static void test_discrete_shooting(float bullet_heat)
{
    HeatControlInstance instance;
    HeatControl_Init_Config_s config = {bullet_heat, 30.0f};
    HeatControlInit(&instance, &config);
    const float limit = 300.0f, cooling = 20.0f;
    float heat = 0.0f, phase = 0.0f, heat_sum = 0.0f, rate_sum = 0.0f;
    unsigned shots = 0;
    HeatControlData_s delayed = {0, 300, 20, 1};
    task_dt = 0.005f;
    publish(0, 300, 20, 1);
    for (unsigned step = 0; step < 12000; ++step)
    {
        /* 发布上一采样时刻的数据，模拟约一个周期的反馈滞后。 */
        if (step % 20 == 0)
        {
            pending_data = delayed;
            pending = 1;
            delayed.current_heat = (uint16_t)heat;
        }
        float rate = HeatControlCalculate(&instance, 1);
        phase += rate * task_dt;
        while (phase >= 1.0f)
        {
            phase -= 1.0f;
            heat += bullet_heat;
            assert(heat < limit); /* 冷却前、逐发检查峰值。 */
            if (step >= 6000)
                ++shots;
        }
        if ((step + 1) % 20 == 0)
            heat = fmaxf(0.0f, heat - cooling * 0.1f);
        if (step >= 6000)
        {
            heat_sum += heat;
            rate_sum += rate;
        }
    }
    float mean_heat = heat_sum / 6000.0f;
    float mean_rate = rate_sum / 6000.0f;
    float expected_heat = limit - HEAT_CONTROL_RESERVE_SHOTS * bullet_heat - 2.0f;
    assert(fabsf(mean_heat - expected_heat) < bullet_heat + 2.0f);
    assert(fabsf(mean_rate - cooling / bullet_heat) < 0.2f);
    assert(fabsf(shots / 30.0f - cooling / bullet_heat) < 0.2f);
    printf("bullet_heat=%.0f: mean_heat=%.2f, target=%.2f, shots/s=%.2f\n",
           bullet_heat, mean_heat, mean_rate, shots / 30.0f);
}

int main(void)
{
    assert(HEAT_CONTROL_PLAN_PERIOD_S >= HEAT_CONTROL_SETTLE_PERIOD_S);
    test_curve(10.0f);
    test_curve(2.0f);
    test_task_behavior();
    test_discrete_shooting(10.0f);
    test_discrete_shooting(2.0f);
    puts("heat_control tests passed");
    return 0;
}
