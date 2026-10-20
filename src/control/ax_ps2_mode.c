#include "ax_ps2_mode.h"

static volatile AX_PS2_Mode current_mode = AX_PS2_MODE_DRIVE;
static uint8_t first_valid_sample = 1;
static uint8_t ignore_initial_select;
static uint8_t select_was_down;

void AX_PS2_ModeReset(void)
{
    current_mode = AX_PS2_MODE_DRIVE;
    first_valid_sample = 1;
    ignore_initial_select = 0;
    select_was_down = 0;
}

AX_PS2_InputAction AX_PS2_ModeUpdate(uint8_t valid, uint8_t select_down)
{
    if (!valid) {
        current_mode = AX_PS2_MODE_DRIVE;
        first_valid_sample = 1;
        ignore_initial_select = 0;
        select_was_down = 0;
        return AX_PS2_INPUT_INVALID;
    }

    if (first_valid_sample) {
        first_valid_sample = 0;
        ignore_initial_select = select_down;
        select_was_down = 0;
        if (select_down) return AX_PS2_INPUT_SWITCHING;
    }

    if (ignore_initial_select) {
        if (!select_down) ignore_initial_select = 0;
        return AX_PS2_INPUT_SWITCHING;
    }

    if (select_down) {
        select_was_down = 1;
        return AX_PS2_INPUT_SWITCHING;
    }

    if (select_was_down) {
        select_was_down = 0;
        current_mode = current_mode == AX_PS2_MODE_DRIVE ? AX_PS2_MODE_FOLLOW
                     : current_mode == AX_PS2_MODE_FOLLOW ?
                           AX_PS2_MODE_FOLLOW_AVOID
                     : current_mode == AX_PS2_MODE_FOLLOW_AVOID ?
                           AX_PS2_MODE_SCAN
                     : AX_PS2_MODE_DRIVE;
        return AX_PS2_INPUT_MODE_CHANGED;
    }

    return current_mode == AX_PS2_MODE_DRIVE ? AX_PS2_INPUT_DRIVE
         : current_mode == AX_PS2_MODE_FOLLOW ? AX_PS2_INPUT_FOLLOW
         : current_mode == AX_PS2_MODE_FOLLOW_AVOID ?
               AX_PS2_INPUT_FOLLOW_AVOID
         : AX_PS2_INPUT_SCAN;
}

AX_PS2_Mode AX_PS2_ModeGet(void)
{
    return current_mode;
}
