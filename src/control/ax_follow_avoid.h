#ifndef AX_FOLLOW_AVOID_H
#define AX_FOLLOW_AVOID_H

/* Called only while PS2 FOLLOW_AVOID is the active input action. */
void AX_FOLLOW_AVOID_Step(void);

/* Clear policy and follow hysteresis when leaving the mode. */
void AX_FOLLOW_AVOID_Reset(void);

#endif
