#ifndef TEST_AX_AOA_H
#define TEST_AX_AOA_H

#include <stdint.h>

typedef struct {
    struct {
        uint16_t distance_cm;
        int16_t azimuth_deg;
    } measurement;
} AX_AOA_Sample;

uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms);

#endif
