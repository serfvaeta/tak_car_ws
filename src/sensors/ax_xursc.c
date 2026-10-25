#include "ax_xursc.h"

#include "ax_delay.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

/* A 10 m echo takes about 58.3 ms to make the round trip. */
#define XURSC_ECHO_TIMEOUT_US   65000u
#define XURSC_MIN_TRIGGER_GAP_US 60000u
#define XURSC_MIN_DISTANCE_MM   30u
#define XURSC_MAX_DISTANCE_MM   10000u

typedef enum {
    XURSC_PHASE_IDLE = 0,
    XURSC_PHASE_RISING,
    XURSC_PHASE_FALLING
} XURSC_Phase;

static volatile uint32_t timer_overflows;
static volatile uint32_t trigger_time_us;
static volatile uint8_t triggered_once;
static volatile uint32_t rise_time_us;
static volatile uint16_t measured_mm;
static volatile AX_XURSC_Status measurement_status = AX_XURSC_UNINITIALIZED;
static volatile XURSC_Phase phase = XURSC_PHASE_IDLE;

/* TIM8 is 16 bit. Its update interrupt extends the 1 MHz counter to 32 bit. */
static uint32_t XURSC_NowUs(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t overflows;
    uint16_t count;

    __disable_irq();
    overflows = timer_overflows;
    count = (uint16_t)TIM_GetCounter(TIM8);
    if (TIM_GetFlagStatus(TIM8, TIM_FLAG_Update) != RESET && count < 0x8000u)
        ++overflows;
    __set_PRIMASK(primask);

    return (overflows << 16) | count;
}

/* Assign a captured edge to the correct side of a timer rollover even if
 * the update interrupt and capture interrupt are serviced in either order. */
static uint32_t XURSC_CaptureUs(uint16_t capture)
{
    uint32_t overflows = timer_overflows;
    uint16_t count = (uint16_t)TIM_GetCounter(TIM8);

    if (TIM_GetFlagStatus(TIM8, TIM_FLAG_Update) != RESET) {
        if (capture < 0x8000u) ++overflows;
    } else if (capture > count && (uint16_t)(capture - count) > 0x8000u) {
        --overflows;
    }
    return (overflows << 16) | capture;
}

void AX_XURSC_PinsSafe(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOC, &gpio);
}

void AX_XURSC_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_ICInitTypeDef capture;
    NVIC_InitTypeDef nvic;

    if (measurement_status != AX_XURSC_UNINITIALIZED) return;

    AX_XURSC_PinsSafe();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    TIM_DeInit(TIM8);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 168u - 1u; /* H60 TIM8 clock is 168 MHz. */
    timer.TIM_Period = 0xffffu;
    TIM_TimeBaseInit(TIM8, &timer);

    TIM_ICStructInit(&capture);
    capture.TIM_Channel = TIM_Channel_1;
    capture.TIM_ICPolarity = TIM_ICPolarity_Rising;
    capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
    capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    capture.TIM_ICFilter = 2;
    TIM_ICInit(TIM8, &capture);

    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1 | TIM_IT_Update);
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = TIM8_CC_IRQn;
    NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_Init(&nvic);
    TIM_ITConfig(TIM8, TIM_IT_CC1 | TIM_IT_Update, ENABLE);

    timer_overflows = 0;
    triggered_once = 0;
    measured_mm = 0;
    phase = XURSC_PHASE_IDLE;
    measurement_status = AX_XURSC_IDLE;
    TIM_Cmd(TIM8, ENABLE);
}

uint8_t AX_XURSC_Trigger(void)
{
    uint32_t primask;
    uint32_t now_us;

    if (measurement_status == AX_XURSC_UNINITIALIZED) return 0;
    primask = __get_PRIMASK();
    __disable_irq();
    now_us = XURSC_NowUs();
    if (phase != XURSC_PHASE_IDLE ||
        (triggered_once &&
         (uint32_t)(now_us - trigger_time_us) < XURSC_MIN_TRIGGER_GAP_US)) {
        __set_PRIMASK(primask);
        return 0;
    }
    TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising);
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
    trigger_time_us = now_us;
    triggered_once = 1;
    phase = XURSC_PHASE_RISING;
    measurement_status = AX_XURSC_WAITING;

    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    AX_Delayus(20); /* XURSC requires at least 10 us high on TRIG. */
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    __set_PRIMASK(primask);
    return 1;
}

AX_XURSC_Status AX_XURSC_GetStatus(uint16_t *distance_mm)
{
    uint32_t now_us;
    uint32_t primask;
    AX_XURSC_Status status;

    if (measurement_status == AX_XURSC_UNINITIALIZED) return AX_XURSC_UNINITIALIZED;
    primask = __get_PRIMASK();
    __disable_irq();
    now_us = XURSC_NowUs();
    if (phase != XURSC_PHASE_IDLE &&
        (uint32_t)(now_us - trigger_time_us) >= XURSC_ECHO_TIMEOUT_US) {
        phase = XURSC_PHASE_IDLE;
        measurement_status = AX_XURSC_TIMEOUT;
        TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising);
    }
    status = measurement_status;
    if (distance_mm != 0 && status == AX_XURSC_READY) *distance_mm = measured_mm;
    __set_PRIMASK(primask);
    return status;
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
        ++timer_overflows;
    }
}

void TIM8_CC_IRQHandler(void)
{
    uint16_t captured_count;
    uint32_t captured_us;
    uint32_t pulse_us;
    uint32_t distance_mm;

    if (TIM_GetITStatus(TIM8, TIM_IT_CC1) == RESET) return;
    captured_count = TIM_GetCapture1(TIM8);
    captured_us = XURSC_CaptureUs(captured_count);
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

    if (phase == XURSC_PHASE_RISING) {
        rise_time_us = captured_us;
        phase = XURSC_PHASE_FALLING;
        TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling);
    } else if (phase == XURSC_PHASE_FALLING) {
        phase = XURSC_PHASE_IDLE;
        TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising);
        pulse_us = captured_us - rise_time_us;
        if (pulse_us == 0 || pulse_us > XURSC_ECHO_TIMEOUT_US) {
            measurement_status = AX_XURSC_TIMEOUT;
            return;
        }
        /* 343 m/s = 0.343 mm/us. The echo covers the round trip. */
        distance_mm = (pulse_us * 343u + 1000u) / 2000u;
        if (distance_mm < XURSC_MIN_DISTANCE_MM ||
            distance_mm > XURSC_MAX_DISTANCE_MM) {
            measurement_status = AX_XURSC_OUT_OF_RANGE;
        } else {
            measured_mm = (uint16_t)distance_mm;
            measurement_status = AX_XURSC_READY;
        }
    } else {
        TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising);
    }
}
