#ifndef AX_PS2_MODE_H
#define AX_PS2_MODE_H

#include <stdint.h>

typedef enum {
    AX_PS2_MODE_DRIVE = 0,
    AX_PS2_MODE_FOLLOW,
    AX_PS2_MODE_FOLLOW_AVOID,
    AX_PS2_MODE_FOLLOW_AVOID2,
    AX_PS2_MODE_SCAN
} AX_PS2_Mode;

typedef enum {
    AX_PS2_INPUT_INVALID = 0,
    AX_PS2_INPUT_SWITCHING,
    AX_PS2_INPUT_MODE_CHANGED,
    AX_PS2_INPUT_DRIVE,
    AX_PS2_INPUT_FOLLOW,
    AX_PS2_INPUT_FOLLOW_AVOID,
    AX_PS2_INPUT_FOLLOW_AVOID2,
    AX_PS2_INPUT_SCAN
} AX_PS2_InputAction;

/* Robot_Task calls Reset while another control source owns the chassis. */
void AX_PS2_ModeReset(void);
/* Called with a complete PS2 sample. SELECT release cycles the modes once. */
AX_PS2_InputAction AX_PS2_ModeUpdate(uint8_t valid, uint8_t select_down);
AX_PS2_Mode AX_PS2_ModeGet(void);

#endif
