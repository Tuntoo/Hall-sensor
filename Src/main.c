#include "main.h"

volatile Sensor_t g_sensor;
volatile Spc_Dbg_t g_dbg;
volatile WheelSpeed_t g_wheel;

typedef enum {
    RX_IDLE = 0,
    RX_WAIT_E1,
    RX_WAIT_SYNC,
    RX_DATA
} RxState_t;

static volatile RxState_t s_state = RX_IDLE;
static volatile uint16_t s_last_edge;
static volatile uint16_t s_gap_counts;
static volatile uint8_t s_count;
static uint16_t s_dt[SPC_MAX_INTERVALS];

static volatile uint16_t s_raw_pending;
static volatile uint16_t s_sub_pending;
static volatile uint8_t s_have_sub;
static volatile uint32_t s_decoded_frames;
static volatile uint32_t s_ut_ns;

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void Timer_Init(void);
static void Spc_Schedule(void);
static void Send_Trigger(void);
static void Decode_Frame(uint8_t interval_count);
static void Sensor_Update(void);
static void WheelSpeed_Update(uint8_t level);
static void WheelSpeed_Service(void);

/* Integer division rounded to nearest (rather than always rounding down). */
static uint32_t Div_Round_Nearest(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U) return 0U;
    return (numerator + denominator / 2U) / denominator;
}

/* Map configured raw range to an integer scale 1..100. The integer rounding
 * deliberately makes small raw changes invisible until the next scale step. */

static uint8_t Raw_To_Level(uint16_t raw)
{
    uint32_t x = raw;
    uint32_t scaled;
    if (x == LEVEL_RAW_CENTER)
    {
        return LEVEL_RAW_MIN;
    }
    if (x < LEVEL_RAW_CENTER)
    {
        scaled = Div_Round_Nearest(
            (LEVEL_RAW_CENTER - x) * LEVEL_MAX,
			LEVEL_RAW_CENTER
        );
    }
    else
    {
        if (x >= LEVEL_RAW_MAX)
        {
            return LEVEL_MAX;
        }
        scaled = Div_Round_Nearest(
            (x - LEVEL_RAW_CENTER) * LEVEL_MAX,
			LEVEL_RAW_MAX - LEVEL_RAW_CENTER
        );
    }
    if (scaled > LEVEL_MAX)
    {
        scaled = LEVEL_MAX;
    }
    return (uint8_t)scaled;
}

static int32_t Raw_To_uT(uint16_t raw)
{
    return (((int32_t)raw - SENSOR_ZERO_CODE) * 1000000L) /
           SENSOR_LSB12_PER_MT_X1000;
}

/* Decode nibble timings into a Main raw value. No CRC, status, range or
 * plausibility check is used to block output. Sync detection is retained
 * because it is needed to determine the unit time and frame boundaries. */
static void Decode_Frame(uint8_t interval_count)
{
    uint8_t nib[SPC_MAX_NIBBLES] = {0};
    uint8_t nn;
    uint16_t sync;
    uint16_t main_raw = 0U;
    uint16_t sub_raw = 0U;
    uint8_t enough_main = 0U;

    if (interval_count < 2U || interval_count > SPC_MAX_INTERVALS) return;

    sync = s_dt[0];
    if (sync == 0U) return;

    nn = (uint8_t)(interval_count - 1U);
    g_dbg.frames++;
    g_dbg.n_nib = nn;
    g_sensor.n_nib = nn;
    g_sensor.last_ut_ns = (uint32_t)(((uint32_t)sync * 1000UL) /
                                     (SPC_TIM_MHZ * SPC_SYNC_UT));
    s_ut_ns = g_sensor.last_ut_ns;

    for (uint8_t i = 0U; i < SPC_MAX_INTERVALS; i++) {
        g_dbg.dt[i] = (i < interval_count) ? s_dt[i] : 0U;
    }

    /* SENT/SPC nibble interval is 12..27 UT. Clamp decoded values to 0..15
     * instead of rejecting the entire frame, so diagnostics still update. */
    for (uint8_t i = 0U; i < nn && i < SPC_MAX_NIBBLES; i++) {
        uint32_t ut = Div_Round_Nearest((uint32_t)s_dt[i + 1U] * SPC_SYNC_UT, sync);
        if (ut <= 12UL) nib[i] = 0U;
        else if (ut >= 27UL) nib[i] = 15U;
        else nib[i] = (uint8_t)(ut - 12UL);
    }

    for (uint8_t i = 0U; i < SPC_MAX_NIBBLES; i++) {
        g_dbg.nib[i] = (i < nn) ? nib[i] : 0U;
    }


    /* Frame A: Main = M1:M2:M3 = 12 bits. */
    if (nn >= 4U)
    {
        main_raw = (uint16_t)(((uint16_t)nib[1] << 8) |
                              ((uint16_t)nib[2] << 4) |
                              (uint16_t)nib[3]);

        enough_main = 1U;
    }

    /* Sub = S3:S2:S1 = 12 bits. */
    if (nn >= 7U)
    {
        sub_raw = (uint16_t)(((uint16_t)nib[6] << 8) |
                             ((uint16_t)nib[5] << 4) |
                             (uint16_t)nib[4]);

        s_have_sub = 1U;
    }
    else
    {
        s_have_sub = 0U;
    }


    if (enough_main) {
        s_raw_pending = main_raw;
        s_sub_pending = sub_raw;
        s_decoded_frames++;
    }
}

static void Arm_End_Timeout(uint16_t now)
{
    TIM2->CCR1 = (uint16_t)(now + s_gap_counts);
    TIM2->SR = (uint32_t)~TIM_SR_CC1IF;
    TIM2->DIER |= TIM_DIER_CC1IE;
}

static void On_Falling_Edge(uint16_t now)
{
    uint16_t dt = (uint16_t)(now - s_last_edge);
    s_last_edge = now;
    g_dbg.edges++;

    switch (s_state) {
    case RX_WAIT_E1:
        s_state = RX_WAIT_SYNC;
        break;

    case RX_WAIT_SYNC:
        if (dt >= SPC_SYNC_MIN_COUNTS && dt <= SPC_SYNC_MAX_COUNTS) {
            s_dt[0] = dt;
            s_count = 1U;
            s_gap_counts = (uint16_t)(((uint32_t)dt * SPC_END_GAP_UT) / SPC_SYNC_UT);
            s_state = RX_DATA;
            Arm_End_Timeout(now);
        } else {
            s_state = RX_IDLE;
        }
        break;

    case RX_DATA:
        if (s_count < SPC_MAX_INTERVALS) {
            s_dt[s_count++] = dt;
            Arm_End_Timeout(now);
        } else {
            s_count = 0U;
            s_state = RX_IDLE;
            TIM2->DIER &= (uint32_t)~TIM_DIER_CC1IE;
        }
        break;

    case RX_IDLE:
    default:
        break;
    }
}

void EXTI15_10_IRQHandler(void)
{
    uint16_t now = (uint16_t)TIM2->CNT;
    if (EXTI->PR & SPC_EXTI_PR_BIT) {
        EXTI->PR = SPC_EXTI_PR_BIT;
        On_Falling_Edge(now);
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR = (uint32_t)~TIM_SR_CC1IF;
        TIM2->DIER &= (uint32_t)~TIM_DIER_CC1IE;
        if (s_state == RX_DATA) {
            uint8_t n = s_count;
            s_count = 0U;
            s_state = RX_IDLE;
            Decode_Frame(n);
        }
    }
}

static void Send_Trigger(void)
{
    __disable_irq();
    TIM2->DIER &= (uint32_t)~TIM_DIER_CC1IE;
    TIM2->SR = (uint32_t)~TIM_SR_CC1IF;
    EXTI->IMR &= (uint32_t)~SPC_EXTI_IMR_BIT;

    /* PA12 is open-drain: pull low to trigger, then release to pull-up. */
    SPC_GPIO_PORT->BSRR = (uint32_t)SPC_GPIO_PIN << 16;
    {
        uint16_t start = (uint16_t)TIM2->CNT;
        while ((uint16_t)((uint16_t)TIM2->CNT - start) < SPC_TRIG_LOW_COUNTS) { }
    }
    SPC_GPIO_PORT->BSRR = SPC_GPIO_PIN;

    EXTI->PR = SPC_EXTI_PR_BIT;
    s_count = 0U;
    s_state = RX_WAIT_E1;
    EXTI->IMR |= SPC_EXTI_IMR_BIT;
    g_sensor.triggers++;
    __enable_irq();
}

static void Spc_Schedule(void)
{
    static uint32_t last_trigger_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_trigger_ms) < SPC_PERIOD_MS) return;
    if ((uint16_t)((uint16_t)TIM2->CNT - s_last_edge) < SPC_GUARD_COUNTS) return;

    Send_Trigger();
    last_trigger_ms = now;
}


/*
 * Wheel-speed algorithm based on g_sensor.level.
 * A cycle is counted when level leaves the baseline zone by the configured
 * threshold, then returns to the baseline zone. A level is accepted only
 * after several consecutive frames are close to one another.
 */
static uint8_t Level_Diff(uint8_t a, uint8_t b)
{
    return (a >= b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

static void WheelSpeed_Update(uint8_t level)
{
    static uint32_t last_rev_time_ms = 0U;
    static uint8_t have_rev_time = 0U;

    uint32_t now_ms = HAL_GetTick();

    g_wheel.current_level = level;

    /* Lần đầu: thiết lập baseline */
    if (g_wheel.initialized == 0U)
    {
        g_wheel.baseline_level = level;
        g_wheel.candidate_level = level;
        g_wheel.initialized = 1U;
        g_wheel.moving_seen = 0U;
        g_wheel.revolutions = 0U;
        g_wheel.rpm = 0U;
        g_wheel.last_rev_interval_ms = 0U;
        g_wheel.last_rev_time_ms = 0U;
        return;
    }

    /* Bước 1: level tăng ít nhất 1 so với baseline */
    if (g_wheel.moving_seen == 0U)
    {
        if (level >= (uint8_t)(g_wheel.baseline_level + 1U))
        {
            g_wheel.moving_seen = 1U;
        }
    }
    /* Bước 2: level quay về đúng baseline thì đếm 1 */
    else
    {
        if (level == g_wheel.baseline_level)
        {
            g_wheel.revolutions++;

            if (have_rev_time != 0U)
            {
                uint32_t interval_ms = now_ms - last_rev_time_ms;

                g_wheel.last_rev_interval_ms = interval_ms;

                if (interval_ms >= WHEEL_MIN_REV_INTERVAL_MS)
                {
                    g_wheel.rpm = 60000U / interval_ms;
                }
            }

            last_rev_time_ms = now_ms;
            g_wheel.last_rev_time_ms = now_ms;
            have_rev_time = 1U;

            /* Chờ chu kỳ tiếp theo */
            g_wheel.moving_seen = 0U;
        }
    }
}

static void WheelSpeed_Service(void)
{
    uint32_t now_ms = HAL_GetTick();

    /* Reset bo dem neu qua 2 giay khong co vong quay moi */
    if ((g_wheel.revolutions > 0U) &&
        ((now_ms - g_wheel.last_rev_time_ms) >= 2000U))
    {
        g_wheel.revolutions = 0U;
        g_wheel.rpm = 0U;
    }
    if (g_wheel.initialized == 0U) return;

    /* If no new completed revolution arrives, report zero RPM after timeout. */
    if (g_wheel.have_cycle_time != 0U &&
        (uint32_t)(now_ms - g_wheel.last_rev_time_ms) >= WHEEL_STOP_TIMEOUT_MS)
    {
        g_wheel.rpm = 0U;
    }

    /* Recalibrate to the current stable resting position after 2 s.
     * This also handles stopping at a position different from the old baseline. */
    if (g_wheel.stable_frames >= WHEEL_STABLE_FRAMES &&
        (uint32_t)(now_ms - g_wheel.stable_since_ms) >= WHEEL_BASELINE_IDLE_MS)
    {
        g_wheel.baseline_level = g_wheel.candidate_level;
        g_wheel.moving_seen = 0U;
        g_wheel.stable_event_done = 1U;
        g_wheel.stable_since_ms = now_ms;
    }
}

static void Sensor_Update(void)
{
    static uint32_t last_count = 0U;
    uint32_t count;
    uint16_t raw;
    uint16_t sub;
    uint8_t have_sub;
    uint32_t ut_ns;

    __disable_irq();
    count = s_decoded_frames;
    raw = s_raw_pending;
    sub = s_sub_pending;
    have_sub = s_have_sub;
    ut_ns = s_ut_ns;
    __enable_irq();

    if (count != last_count) {
        last_count = count;
        g_sensor.raw = raw;
        g_sensor.level = Raw_To_Level(raw);
        g_sensor.field_inst_uT = Raw_To_uT(raw);
        g_sensor.field_mT = (float)g_sensor.field_inst_uT / 1000.0f;
        g_sensor.raw_sub = have_sub ? sub : 0U;
        g_sensor.decoded_frames = count;
        g_sensor.last_ut_ns = ut_ns;
        WheelSpeed_Update(g_sensor.level);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    Timer_Init();

    while (1) {
        Spc_Schedule();
        Sensor_Update();
        WheelSpeed_Service();
    }
}
/*  */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(SPC_GPIO_PORT, SPC_GPIO_PIN, GPIO_PIN_SET);
    gi.Pin = SPC_GPIO_PIN;
    gi.Mode = GPIO_MODE_OUTPUT_OD;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPC_GPIO_PORT, &gi);

    AFIO->EXTICR[3] &= (uint32_t)~AFIO_EXTICR4_EXTI12;
    EXTI->RTSR &= (uint32_t)~EXTI_RTSR_TR12;
    EXTI->FTSR |= EXTI_FTSR_TR12;
    EXTI->PR = SPC_EXTI_PR_BIT;
    EXTI->IMR |= SPC_EXTI_IMR_BIT;

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void Timer_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = SPC_TIM_PRESCALER;
    TIM2->ARR = 0xFFFFU;
    TIM2->DIER = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* Board has no external crystal: use internal 8 MHz HSI.
     * HSI / 2 * PLL16 = 64 MHz system clock. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
