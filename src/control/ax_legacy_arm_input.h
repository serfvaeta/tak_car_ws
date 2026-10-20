#ifndef AX_LEGACY_ARM_INPUT_H
#define AX_LEGACY_ARM_INPUT_H

#include <stdint.h>
#include "ax_ps2.h"

/* Old arm input paths, used only when OPENCTR_ENABLE_LEGACY_ARM is enabled. */
void AX_LEGACY_ARM_Ps2Step(JOYSTICK_TypeDef joystick);
void AX_LEGACY_ARM_AppFrame(const uint8_t comdata[16]);
void AX_LEGACY_ARM_AppHeld(const uint8_t comdata[16]);

#endif
