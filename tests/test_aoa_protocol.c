#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "ax_aoa_protocol.h"

/* A 37-byte 0x2001 frame captured from the connected base station. */
static const uint8_t location_frame[37] = {
    0xff, 0xff, 0xff, 0xff, 0x00, 0x25, 0x03, 0x67,
    0x20, 0x01, 0x01, 0x02, 0x00, 0x00, 0x12, 0x21,
    0x00, 0x00, 0x12, 0x21, 0x00, 0x00, 0x00, 0x11,
    0x00, 0x03, 0xff, 0xb5, 0x12, 0x34, 0xae, 0x57,
    0xca, 0x00, 0x00, 0x00, 0x2e
};

static const uint8_t offline_heartbeat[16] = {
    0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0x00, 0x0b,
    0x20, 0x02, 0x01, 0x01, 0x00, 0x00, 0xaa, 0xa1
};

static unsigned feed(AX_AOA_Parser *parser, const uint8_t *bytes,
                     unsigned length, AX_AOA_Measurement *result)
{
    unsigned i;
    unsigned decoded = 0;
    for (i = 0; i < length; ++i)
        decoded += AX_AOA_ParserPushByte(parser, bytes[i], result);
    return decoded;
}

int main(void)
{
    AX_AOA_Parser parser;
    AX_AOA_Measurement result;
    uint8_t corrupted[sizeof(location_frame)];
    uint8_t negative_angle[sizeof(location_frame)];

    AX_AOA_ParserReset(&parser);
    assert(feed(&parser, location_frame, 3, &result) == 0);
    assert(feed(&parser, location_frame + 3, 17, &result) == 0);
    assert(feed(&parser, location_frame + 20, 17, &result) == 1);
    assert(result.sequence == 0x0367);
    assert(result.anchor_id == 0x1221);
    assert(result.tag_id == 0x1221);
    assert(result.distance_cm == 17);
    assert(result.azimuth_deg == 3);
    assert(result.elevation_deg == -75);

    memcpy(corrupted, location_frame, sizeof(corrupted));
    corrupted[23] ^= 1;
    AX_AOA_ParserReset(&parser);
    assert(feed(&parser, corrupted, sizeof(corrupted), &result) == 0);
    assert(feed(&parser, location_frame, sizeof(location_frame), &result) == 1);

    AX_AOA_ParserReset(&parser);
    assert(feed(&parser, offline_heartbeat, sizeof(offline_heartbeat), &result) == 0);
    assert(feed(&parser, location_frame, sizeof(location_frame), &result) == 1);

    memcpy(negative_angle, location_frame, sizeof(negative_angle));
    negative_angle[24] = 0xff;
    negative_angle[25] = 0xd9; /* -39 degrees */
    negative_angle[36] = 0x0b; /* recalculated XOR checksum */
    AX_AOA_ParserReset(&parser);
    assert(feed(&parser, negative_angle, sizeof(negative_angle), &result) == 1);
    assert(result.azimuth_deg == -39);

    return 0;
}
