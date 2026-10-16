#ifndef THROTTLE_SENSOR_CONFIG_H
#define THROTTLE_SENSOR_CONFIG_H

#include "stm32f1xx_hal.h"

/*
 * Hardware and calibration configuration for STM32F103 + TLE4999C8 SPC.
 * Override these values in the company project if required.
 * The module does NOT configure the system clock.
 */

#ifndef THROTTLE_GPIO_PORT
#define THROTTLE_GPIO_PORT                 GPIOA
#endif
#ifndef THROTTLE_GPIO_PIN
#define THROTTLE_GPIO_PIN                  GPIO_PIN_12
#endif

#ifndef THROTTLE_PERIOD_MS
#define THROTTLE_PERIOD_MS                 3U
#endif

/* A valid non-zero Main value must be received within this interval. */
#ifndef THROTTLE_RESPONSE_TIMEOUT_MS
#define THROTTLE_RESPONSE_TIMEOUT_MS       20U
#endif

/*
 * Raw change from startup-zero to full throttle.
 * 1043 is carried over from the original project's 3091 - 2048 range;
 * CALIBRATE this constant using real sensor/mechanical end-stop measurements.
 */
#ifndef THROTTLE_FULL_SCALE_DELTA_RAW
#define THROTTLE_FULL_SCALE_DELTA_RAW       1043U
#endif

/* User-defined hardware convention: raw == 0 is reserved as a sensor fault. */
#ifndef THROTTLE_RAW_FAULT_SENTINEL
#define THROTTLE_RAW_FAULT_SENTINEL         0U
#endif

/* SPC/SENT-like timing assumptions carried over from the supplied code. */
#ifndef THROTTLE_SYNC_UT
#define THROTTLE_SYNC_UT                    56U
#endif
#ifndef THROTTLE_SYNC_MIN_UT_NS
#define THROTTLE_SYNC_MIN_UT_NS             400U
#endif
#ifndef THROTTLE_SYNC_MAX_UT_NS
#define THROTTLE_SYNC_MAX_UT_NS             3400U
#endif
#ifndef THROTTLE_NOMINAL_UT_NS
#define THROTTLE_NOMINAL_UT_NS              3000U
#endif
#ifndef THROTTLE_TRIGGER_LOW_NS
#define THROTTLE_TRIGGER_LOW_NS             9000U
#endif
#ifndef THROTTLE_END_GAP_UT
#define THROTTLE_END_GAP_UT                 30U
#endif
#ifndef THROTTLE_GUARD_UT
#define THROTTLE_GUARD_UT                   30U
#endif

#ifndef THROTTLE_MAX_NIBBLES
#define THROTTLE_MAX_NIBBLES                12U
#endif
#define THROTTLE_MAX_INTERVALS              (THROTTLE_MAX_NIBBLES + 1U)

#if (THROTTLE_FULL_SCALE_DELTA_RAW == 0U)
#error "THROTTLE_FULL_SCALE_DELTA_RAW must be greater than zero"
#endif

#endif /* THROTTLE_SENSOR_CONFIG_H */
