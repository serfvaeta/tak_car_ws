#ifndef AX_FOLLOW_AVOID_DIAG_H
#define AX_FOLLOW_AVOID_DIAG_H

#include <stdint.h>
#include "ax_follow_avoid_policy.h"

/* USB-COM frame 0x16. All two-byte fields use signed/unsigned big-endian. */
#define AX_AVOID_DIAG_LENGTH 26u

AX_AVOID_DiagReason AX_AVOID_DiagEncode(const AX_AVOID_State *state,
                                         const AX_AVOID_Config *config,
                                         const AX_AVOID_Input *input,
                                         const AX_AVOID_Output *output,
                                         uint8_t chassis_fwd,
                                         uint16_t aoa_distance_cm,
                                         uint8_t payload[AX_AVOID_DIAG_LENGTH]);

#endif
