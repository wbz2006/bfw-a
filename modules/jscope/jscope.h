#ifndef __JSCOPE_H__
#define __JSCOPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JSCOPE_CHANNEL_COUNT 4U
/* 与工程中原来可正常工作的 J-Scope 配置保持一致。 */
#define JSCOPE_RTT_BUFFER_INDEX 1U
#define JSCOPE_RTT_BUFFER_NAME "JScope_f4f4f4f4"

/**
 * @brief Initialize the RTT up-buffer used by SEGGER J-Scope.
 * @return 0 on success, negative value if the buffer is unavailable.
 */
int JScopeInit(void);

/**
 * @brief Write one floating-point sample frame. Unspecified channels are sent as zero.
 * @return Number of bytes written (16) on success, otherwise 0.
 */
int JScopeWrite(float ch0, float ch1, float ch2, float ch3);

/**
 * @brief Write one sample frame using printf-style format specifiers.
 * @note Supported conversions are f, d, i, u, x, X and c. Integer values are
 *       converted to float before transmission. At most four conversions are
 *       accepted; missing channels are padded with zero.
 * @return Number of bytes written (16) on success, otherwise 0.
 */
int JScopePrintf(const char *format, ...);

/**
 * @brief Write multiple sample frames.
 * @param samples Sample-major array: samples[sample * channel_count + channel].
 * @param sample_count Number of sample frames.
 * @param channel_count Number of channels per frame, from 1 to 4.
 * @return Number of bytes written, or 0 if the complete batch was not written.
 * @note The complete batch must fit in the RTT buffer's current free space.
 */
int JScopeWriteSamples(const float *samples, uint32_t sample_count,
                       uint8_t channel_count);

#ifdef __cplusplus
}
#endif

#endif
