#include "jscope.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#include "cmsis_gcc.h"

#include <stdarg.h>
#include <stddef.h>

#define JSCOPE_BUFFER_SIZE 1024U
#define JSCOPE_ENTER_CRITICAL() __disable_irq()
#define JSCOPE_EXIT_CRITICAL()  __enable_irq()

static uint8_t jscope_buffer[JSCOPE_BUFFER_SIZE];
static int jscope_buffer_index = -1;

int JScopeInit(void)
{
    int result;

    if (jscope_buffer_index >= 0)
    {
        return 0;
    }

    result = SEGGER_RTT_ConfigUpBuffer(
        JSCOPE_RTT_BUFFER_INDEX, JSCOPE_RTT_BUFFER_NAME, jscope_buffer,
        sizeof(jscope_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    if (result < 0)
    {
        return result;
    }

    jscope_buffer_index = (int)JSCOPE_RTT_BUFFER_INDEX;
    return 0;
}

int JScopeWrite(float ch0, float ch1, float ch2, float ch3)
{
    const float sample[JSCOPE_CHANNEL_COUNT] = {ch0, ch1, ch2, ch3};

    if (jscope_buffer_index < 0)
    {
        return 0;
    }

    return SEGGER_RTT_Write((unsigned)jscope_buffer_index, sample,
                            sizeof(sample)) == sizeof(sample)
               ? (int)sizeof(sample)
               : 0;
}

int JScopePrintf(const char *format, ...)
{
    float sample[JSCOPE_CHANNEL_COUNT] = {0};
    uint8_t channel = 0U;
    va_list args;
    const char *cursor;

    if ((format == NULL) || (jscope_buffer_index < 0))
    {
        return 0;
    }

    va_start(args, format);
    for (cursor = format; *cursor != '\0'; ++cursor)
    {
        char conversion;

        if (*cursor != '%')
        {
            continue;
        }
        ++cursor;
        if (*cursor == '%')
        {
            continue;
        }
        if (*cursor == '\0')
        {
            break;
        }

        while ((*cursor == '-') || (*cursor == '+') || (*cursor == ' ') ||
               (*cursor == '#') || (*cursor == '0') || (*cursor == '.') ||
               ((*cursor >= '0') && (*cursor <= '9')) || (*cursor == 'h') ||
               (*cursor == 'l') || (*cursor == 'z'))
        {
            ++cursor;
        }

        conversion = *cursor;
        if ((conversion != 'd') && (conversion != 'i') &&
            (conversion != 'u') && (conversion != 'x') &&
            (conversion != 'X') && (conversion != 'c') &&
            (conversion != 'f'))
        {
            va_end(args);
            return 0;
        }
        if (channel >= JSCOPE_CHANNEL_COUNT)
        {
            va_end(args);
            return 0;
        }

        if (conversion == 'f')
        {
            /* float arguments undergo default promotion to double. */
            sample[channel++] = (float)va_arg(args, double);
        }
        else if (conversion == 'c')
        {
            sample[channel++] = (float)va_arg(args, int);
        }
        else if ((cursor > format) && (cursor[-1] == 'l'))
        {
            if ((conversion == 'u') || (conversion == 'x') ||
                (conversion == 'X'))
            {
                sample[channel++] = (float)va_arg(args, unsigned long);
            }
            else
            {
                sample[channel++] = (float)va_arg(args, long);
            }
        }
        else if ((cursor > format) && (cursor[-1] == 'z'))
        {
            if ((conversion == 'u') || (conversion == 'x') ||
                (conversion == 'X'))
            {
                sample[channel++] = (float)va_arg(args, size_t);
            }
            else
            {
                sample[channel++] = (float)va_arg(args, ptrdiff_t);
            }
        }
        else if ((conversion == 'u') || (conversion == 'x') ||
                 (conversion == 'X'))
        {
            sample[channel++] = (float)va_arg(args, unsigned int);
        }
        else
        {
            sample[channel++] = (float)va_arg(args, int);
        }
    }
    va_end(args);

    return JScopeWrite(sample[0], sample[1], sample[2], sample[3]);
}

int JScopeWriteSamples(const float *samples, uint32_t sample_count,
                       uint8_t channel_count)
{
    uint32_t sample_index;
    uint32_t bytes_to_write;
    uint32_t sample_bytes;
    unsigned buffer_free;

    if ((jscope_buffer_index < 0) || (samples == NULL) ||
        (channel_count == 0U) || (channel_count > JSCOPE_CHANNEL_COUNT) ||
        (sample_count == 0U) ||
        (sample_count > (UINT32_MAX / (JSCOPE_CHANNEL_COUNT * sizeof(float)))))
    {
        return 0;
    }

    sample_bytes = JSCOPE_CHANNEL_COUNT * sizeof(float);
    bytes_to_write = sample_count * sample_bytes;

    JSCOPE_ENTER_CRITICAL();
    buffer_free = SEGGER_RTT_GetAvailWriteSpace((unsigned)jscope_buffer_index);
    if (bytes_to_write > buffer_free)
    {
        JSCOPE_EXIT_CRITICAL();
        return 0;
    }

    for (sample_index = 0U; sample_index < sample_count; ++sample_index)
    {
        float frame[JSCOPE_CHANNEL_COUNT] = {0};
        uint8_t channel;

        for (channel = 0U; channel < channel_count; ++channel)
        {
            frame[channel] = samples[sample_index * channel_count + channel];
        }
        SEGGER_RTT_WriteNoLock((unsigned)jscope_buffer_index, frame,
                               sizeof(frame));
    }
    JSCOPE_EXIT_CRITICAL();
    return (int)bytes_to_write;
}
