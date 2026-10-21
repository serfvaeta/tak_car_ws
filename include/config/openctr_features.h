#ifndef OPENCTR_FEATURES_H
#define OPENCTR_FEATURES_H

/* The XURSC is wired to PC6 (ECHO) and PC7 (TRIG), which the legacy arm
 * uses as TIM8 servo outputs. The range/pan application owns these pins. */
#ifndef OPENCTR_ENABLE_LEGACY_ARM
#define OPENCTR_ENABLE_LEGACY_ARM 0
#endif

#ifndef OPENCTR_ENABLE_SG90_PAN
#define OPENCTR_ENABLE_SG90_PAN 1
#endif

#ifndef OPENCTR_ENABLE_XURSC
#define OPENCTR_ENABLE_XURSC 1
#endif

#ifndef OPENCTR_ENABLE_AOA_RECEIVER
#define OPENCTR_ENABLE_AOA_RECEIVER 0
#endif

#if OPENCTR_ENABLE_AOA_RECEIVER && OPENCTR_ENABLE_TTL_CONTROL
#error "AOA receiver and legacy TTL control cannot share USART2"
#endif

#if OPENCTR_ENABLE_LEGACY_ARM && (OPENCTR_ENABLE_SG90_PAN || OPENCTR_ENABLE_XURSC)
#error "The legacy arm cannot share S1 or TIM8 with the SG90/XURSC modules"
#endif

#endif
