#include <assert.h>
#include "ax_ps2_mode.h"

static AX_PS2_InputAction press_release(void)
{
    assert(AX_PS2_ModeUpdate(1, 1) == AX_PS2_INPUT_SWITCHING);
    return AX_PS2_ModeUpdate(1, 0);
}

int main(void)
{
    AX_PS2_ModeReset();
    assert(AX_PS2_ModeUpdate(1, 1) == AX_PS2_INPUT_SWITCHING);
    assert(AX_PS2_ModeUpdate(1, 1) == AX_PS2_INPUT_SWITCHING);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_SWITCHING);
    assert(AX_PS2_ModeGet() == AX_PS2_MODE_DRIVE);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_DRIVE);

    AX_PS2_ModeReset();
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_DRIVE);
    assert(press_release() == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_FOLLOW);
    assert(press_release() == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_FOLLOW_AVOID);
    assert(press_release() == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_FOLLOW_AVOID2);
    assert(press_release() == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_SCAN);
    assert(press_release() == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_DRIVE);

    assert(AX_PS2_ModeUpdate(1, 1) == AX_PS2_INPUT_SWITCHING);
    assert(AX_PS2_ModeUpdate(1, 1) == AX_PS2_INPUT_SWITCHING);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_MODE_CHANGED);
    assert(AX_PS2_ModeUpdate(1, 0) == AX_PS2_INPUT_FOLLOW);
    assert(AX_PS2_ModeUpdate(0, 0) == AX_PS2_INPUT_INVALID);
    assert(AX_PS2_ModeGet() == AX_PS2_MODE_DRIVE);
    return 0;
}
