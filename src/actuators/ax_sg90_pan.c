#include "ax_sg90_pan.h"
#include "ax_servo.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

static uint8_t pan_initialized;

void AX_SG90_PanPinsSafe(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
}

void AX_SG90_PanInit(void)
{
    if (pan_initialized) return;
    AX_SERVO_S1_Init();
    pan_initialized = 1;
    AX_SERVO_S1_SetAngle(0);
}

void AX_SG90_PanSetAngle(int16_t degrees)
{
    if (!pan_initialized) return;
    if (degrees > 90) degrees = 90;
    if (degrees < -90) degrees = -90;
    AX_SERVO_S1_SetAngle((int16_t)(degrees * 10));
}

void AX_SG90_PanRelease(void)
{
    if (pan_initialized) AX_SERVO_S1_Release();
}
