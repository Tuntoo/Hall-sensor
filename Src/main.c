#ifndef MAIN_H
#define MAIN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TLE4999C8 SPC on STM32F103C8T6 PA12. */
#define SPC_GPIO_PORT              GPIOA
#define SPC_GPIO_PIN               GPIO_PIN_12
#define SPC_EXTI_PR_BIT            EXTI_PR_PR12
#define SPC_EXTI_IMR_BIT           EXTI_IMR_MR12

/* HSI 8 MHz / 2 * PLL16 = 64 MHz; TIM2 runs at 64 MHz. */
#define SPC_TIM_PRESCALER          0U
#define SPC_TIM_MHZ                64UL
#define SPC_SYNC_UT                56UL
#define SPC_SYNC_MIN_COUNTS        (SPC_SYNC_UT * 400UL * SPC_TIM_MHZ / 1000UL)
#define SPC_SYNC_MAX_COUNTS        (SPC_SYNC_UT * 3400UL * SPC_TIM_MHZ / 1000UL)
#define SPC_GUARD_UT               30UL
#define SPC_GUARD_COUNTS           (SPC_GUARD_UT * 3000UL * SPC_TIM_MHZ / 1000UL)
#define SPC_END_GAP_UT             30UL
#define SPC_TRIG_LOW_COUNTS        (3000UL * 30UL * SPC_TIM_MHZ / 10000UL) /* 3 UT */
#define SPC_PERIOD_MS              3U
#define SPC_FORCE_RETRIGGER_MS     10U
#define SPC_MAX_NIBBLES            12U
#define SPC_MAX_INTERVALS          (SPC_MAX_NIBBLES + 1U)

/* Keep Frame A unless the sensor EEPROM is confirmed to use another format.
 * Frame A: status | M1 M2 M3 | S3 S2 S1 | CRC (8 nibbles).
 * Frame B: status | M1 M2 M3 M4 | S4 S3 S2 S1 | CRC (10 nibbles).
 * Frame C has the same 3-nibble Main as A, plus extra data. */
#define SPC_FRAME_A                0
#define SPC_FRAME_B                1
#define SPC_FRAME_C                2
#ifndef SPC_FRAME_TYPE
#define SPC_FRAME_TYPE             SPC_FRAME_A
#endif

/* Raw-to-level conversion configuration. */
#define LEVEL_RAW_MIN              0U
#define LEVEL_RAW_CENTER           2048U
#define LEVEL_RAW_MAX			   3091U
#define LEVEL_MIN                  1U
#define LEVEL_MAX                  100U

/* Conversion assumes zero code 2048 and sensitivity 36.875 LSB/mT. */
#define SENSOR_ZERO_CODE           2048L
#define SENSOR_LSB12_PER_MT_X1000  36875L


/* Wheel-speed detection from normalized sensor level (0..100). */
#define WHEEL_LEVEL_THRESHOLD       1U
#define WHEEL_RETURN_TOLERANCE      1U
#define WHEEL_STABLE_FRAMES         3U
#define WHEEL_STABLE_TOLERANCE      0U
#define WHEEL_BASELINE_IDLE_MS      2000U
#define WHEEL_MIN_REV_INTERVAL_MS   100U
#define WHEEL_STOP_TIMEOUT_MS       1000U
#define HOLE_PER_WHEEL				4U

typedef struct {
    uint16_t raw;                 /* Main raw code, not filtered */
    uint16_t raw_sub;             /* Sub channel, if present */
    uint8_t  level;         /* Rounded normalized scale; current conversion returns 0..100 */
    uint8_t  n_nib;               /* Number of nibbles in latest frame */
    int32_t  field_inst_uT;       /* Estimated field, microtesla */
    float    field_mT;            /* Estimated field, millitesla; not filtered */
    uint32_t decoded_frames;      /* Number of frames from which Main was decoded */
    uint32_t triggers;
    uint32_t last_ut_ns;
} Sensor_t;

typedef struct {
    uint32_t edges;
    uint32_t frames;
    uint16_t dt[SPC_MAX_INTERVALS];
    uint8_t  n_nib;
    uint8_t  nib[SPC_MAX_NIBBLES]; /* nib[1..3] = Main in Frame A/C */
} Spc_Dbg_t;

typedef struct {
    uint8_t  baseline_level;
    uint8_t  current_level;
    uint8_t  candidate_level;
    uint8_t  stable_frames;
    uint8_t  moving_seen;
    uint8_t  stable_event_done;
    uint8_t  initialized;
    uint8_t  have_cycle_time;
    uint32_t revolutions;
    uint32_t counter;
    uint32_t last_rev_interval_ms;
    uint32_t last_rev_time_ms;
    uint32_t stable_since_ms;
    uint32_t rpm;

} WheelSpeed_t;

extern volatile Sensor_t g_sensor;
extern volatile Spc_Dbg_t g_dbg;
extern volatile WheelSpeed_t g_wheel;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
