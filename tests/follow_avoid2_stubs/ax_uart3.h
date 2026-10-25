#ifndef TEST_AX_UART3_H
#define TEST_AX_UART3_H

#include <stdint.h>

void AX_UART3_SendPacket(uint8_t *payload, uint8_t length, uint8_t id);

#endif
