#include "ax_aoa_protocol.h"

#include <string.h>

static uint16_t ReadU16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint32_t ReadU32(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

static int16_t ReadS16(const uint8_t *bytes)
{
    uint16_t raw = ReadU16(bytes);
    return raw < 0x8000u ? (int16_t)raw : (int16_t)((int32_t)raw - 65536);
}

static void DropFirst(AX_AOA_Parser *parser)
{
    --parser->count;
    memmove(parser->bytes, parser->bytes + 1, parser->count);
}

void AX_AOA_ParserReset(AX_AOA_Parser *parser)
{
    parser->count = 0;
}

uint8_t AX_AOA_ParserPushByte(AX_AOA_Parser *parser, uint8_t byte,
                              AX_AOA_Measurement *out)
{
    uint8_t i;
    uint8_t checksum;
    const uint8_t *frame = parser->bytes;

    parser->bytes[parser->count++] = byte;
    while (parser->count >= 4u) {
        if (frame[0] != 0xffu || frame[1] != 0xffu ||
            frame[2] != 0xffu || frame[3] != 0xffu) {
            DropFirst(parser);
            continue;
        }
        if (parser->count < 6u) return 0;
        if (ReadU16(frame + 4) != AX_AOA_LOCATION_FRAME_SIZE) {
            DropFirst(parser);
            continue;
        }
        if (parser->count < AX_AOA_LOCATION_FRAME_SIZE) return 0;

        checksum = 0;
        for (i = 0; i < AX_AOA_LOCATION_FRAME_SIZE; ++i)
            checksum ^= frame[i];
        if (checksum != 0u) {
            DropFirst(parser);
            continue;
        }

        parser->count = 0;
        if (ReadU16(frame + 8) != 0x2001u) return 0;
        out->sequence = ReadU16(frame + 6);
        out->anchor_id = ReadU32(frame + 12);
        out->tag_id = ReadU32(frame + 16);
        out->distance_cm = ReadU32(frame + 20);
        out->azimuth_deg = ReadS16(frame + 24);
        out->elevation_deg = ReadS16(frame + 26);
        return 1;
    }
    return 0;
}
