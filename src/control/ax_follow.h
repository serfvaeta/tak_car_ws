#ifndef AX_FOLLOW_H
#define AX_FOLLOW_H

/* Called when PS2 FOLLOW mode is active. */
void AX_FOLLOW_Step(void);

/* Clear hysteresis when leaving FOLLOW mode. */
void AX_FOLLOW_ModeReset(void);

#endif
