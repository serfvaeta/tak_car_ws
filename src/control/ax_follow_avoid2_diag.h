#ifndef AX_FOLLOW_AVOID2_DIAG_H
#define AX_FOLLOW_AVOID2_DIAG_H

#include <stdint.h>
#include "ax_follow_avoid2_policy.h"

/* USB-COM frame 0x17; multi-byte fields are big-endian. */
#define AX_AVOID2_DIAG_LENGTH 30u

void AX_AVOID2_DiagEncode(const AX_AVOID2_State *state,
                          const AX_AVOID2_Input *input,
                          const AX_AVOID2_Output *output,
                          uint8_t payload[AX_AVOID2_DIAG_LENGTH]);

#endif
