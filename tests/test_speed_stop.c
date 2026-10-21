#include <assert.h>
#include "platform.h"

int16_t ax_motor_kp = 100;
int16_t ax_motor_kd = 0;

typedef int16_t (*PidFunction)(float, float);

int main(void)
{
    const PidFunction pid[4] = {
        AX_SPEED_PidCtlA, AX_SPEED_PidCtlB,
        AX_SPEED_PidCtlC, AX_SPEED_PidCtlD
    };
    for (unsigned i = 0; i < 4; ++i) {
        assert(pid[i](1.0f, 0.0f) == 100);
        assert(pid[i](0.0f, 0.0f) == 0);
        assert(pid[i](1.0f, 0.0f) == 100);
    }
    return 0;
}
