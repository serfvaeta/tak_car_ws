#ifndef AX_FOLLOW_PROFILE_H
#define AX_FOLLOW_PROFILE_H

#include "ax_follow_policy.h"

/* Shared FOLLOW tuning; each mode supplies its own forward speed ceiling. */
AX_FOLLOW_Config AX_FOLLOW_Profile(int16_t forward_max_mm_s);

#endif
