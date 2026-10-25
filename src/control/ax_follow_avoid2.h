#ifndef AX_FOLLOW_AVOID2_H
#define AX_FOLLOW_AVOID2_H

/* Called only while PS2 FOLLOW_AVOID2 is the active input action. */
void AX_FOLLOW_AVOID2_Step(void);

/* Clear the independent AVOID2 search state when leaving the mode. */
void AX_FOLLOW_AVOID2_Reset(void);

#endif
