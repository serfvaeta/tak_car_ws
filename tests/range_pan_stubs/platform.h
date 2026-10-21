#ifndef TEST_RANGE_PAN_PLATFORM_H
#define TEST_RANGE_PAN_PLATFORM_H

#include <stdint.h>

/* Keep the legacy, hardware-heavy ax_robot.h out of this host test. */
#define __AX_ROBOT_H
#define ID_UTX_XURSC 0x11u
#define ID_UTX_SG90_PAN 0x12u
#define ID_UTX_SCAN 0x15u
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
TickType_t xTaskGetTickCount(void);
void AX_UART3_SendPacket(uint8_t *data, uint8_t len, uint8_t id);

#endif
