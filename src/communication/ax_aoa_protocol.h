#ifndef AX_AOA_PROTOCOL_H
#define AX_AOA_PROTOCOL_H

#include <stdint.h>

#define AX_AOA_LOCATION_FRAME_SIZE 37u

typedef struct {
    uint16_t sequence;
    uint32_t anchor_id;
    uint32_t tag_id;
    uint32_t distance_cm;
    int16_t azimuth_deg;
    int16_t elevation_deg;
} AX_AOA_Measurement;

typedef struct {
    uint8_t bytes[AX_AOA_LOCATION_FRAME_SIZE];
    uint8_t count;
} AX_AOA_Parser;

void AX_AOA_ParserReset(AX_AOA_Parser *parser);

/* Returns 1 only after a complete, XOR-valid 0x2001 location frame.
 * Heartbeats (0x2002), incomplete and damaged frames produce 0. */
uint8_t AX_AOA_ParserPushByte(AX_AOA_Parser *parser, uint8_t byte,
                              AX_AOA_Measurement *out);

#endif
