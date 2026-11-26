#include "ld19.h"
#include <string.h>

#define LD19_HEADER 0x54u
#define LD19_VER_LEN 0x2cu

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t ld19_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80u) ? 0x4du : 0u));
        }
    }
    return crc;
}

static bool ld19_valid_frame(const uint8_t *frame)
{
    return frame[0] == LD19_HEADER && frame[1] == LD19_VER_LEN &&
           ld19_crc8(frame, LD19_PACKET_SIZE - 1u) == frame[LD19_PACKET_SIZE - 1u];
}

size_t ld19_count_frames(uint8_t *buffer, size_t *length)
{
    size_t frames = 0;
    size_t offset = 0;
    while (*length - offset >= LD19_PACKET_SIZE) {
        const uint8_t *frame = &buffer[offset];
        if (ld19_valid_frame(frame)) {
            ++frames;
            offset += LD19_PACKET_SIZE;
        } else {
            ++offset;
        }
    }
    *length -= offset;
    memmove(buffer, buffer + offset, *length);
    return frames;
}

static void ld19_byte_to_packet(const uint8_t *data, ld19_packet_t *packet)
{
    packet->rotation_speed = read_u16_le(data + 2);
    packet->first_degree = read_u16_le(data + 4) / 100.0;
    packet->last_degree = read_u16_le(data + 42) / 100.0;

    for (size_t i = 0; i < LD19_POINTS_PER_PACKET; ++i) {
        packet->distances_mm[i] = read_u16_le(data + 6 + 3 * i);
        packet->intensity[i] = data[8 + 3 * i];
    }

    packet->time_stamp = read_u16_le(data + 44);
    packet->crc = data[46];
}

/* 破損したフレームの途中に次のヘッダが来ている場合は、その位置から受信を続ける。 */
static void ld19_keep_frame_suffix(ld19_parser_t *parser)
{
    for (size_t i = 1; i < parser->length; ++i) {
        if (parser->frame[i] == LD19_HEADER &&
            (i + 1 == parser->length || parser->frame[i + 1] == LD19_VER_LEN)) {
            parser->length -= i;
            memmove(parser->frame, parser->frame + i, parser->length);
            return;
        }
    }
    parser->length = 0;
}

static void ld19_discard_scan(ld19_parser_t *parser, ld19_scan_t *scan)
{
    parser->have_previous_angle = false;
    parser->collecting = false;
    scan->count = 0;
}

static ld19_scan_state_t ld19_append_packet(ld19_parser_t *parser,
                                            const ld19_packet_t *packet,
                                            ld19_scan_t *current, ld19_scan_t *next)
{
    double span = packet->last_degree - packet->first_degree;
    if (span < 0.0)
        span += 360.0;

    if (packet->first_degree >= 360.0 || packet->last_degree >= 360.0 || span >= 180.0) {
        ld19_discard_scan(parser, current);
        return SCAN_NEED_MORE;
    }

    double step = span / (LD19_POINTS_PER_PACKET - 1u);
    ld19_scan_state_t state = SCAN_NEED_MORE;
    for (size_t i = 0; i < LD19_POINTS_PER_PACKET; ++i) {
        float angle = (float)(packet->first_degree + i * step);
        if (angle >= 360.0f)
            angle -= 360.0f;

        bool starts_scan = parser->have_previous_angle ?
                           parser->previous_angle - angle > 180.0f : angle == 0.0f;
        if (starts_scan) {
            if (parser->collecting) {
                state = SCAN_OK;
                current = next;
            }
            current->count = 0;
            parser->collecting = true;
        }
        parser->previous_angle = angle;
        parser->have_previous_angle = true;

        if (parser->collecting) {
            if (current->count == LD19_MAX_POINTS) {
                /* 上限に達した周回は破棄し、次の0度通過で再開する。 */
                parser->collecting = false;
                current->count = 0;
                continue;
            }
            ld19_point_t *point = &current->points[current->count++];
            point->angle_deg = angle;
            point->distance_mm = packet->distances_mm[i];
            point->intensity = packet->intensity[i];
        }
    }
    return state;
}

ld19_scan_state_t ld19_byte_parse(ld19_parser_t *parser, uint8_t byte,
                                  ld19_scan_t *current, ld19_scan_t *next)
{
    parser->frame[parser->length++] = byte;
    if (parser->frame[0] == LD19_HEADER &&
        (parser->length == 1 || parser->frame[1] == LD19_VER_LEN)) {
        if (parser->length < LD19_PACKET_SIZE)
            return SCAN_NEED_MORE;

        if (ld19_valid_frame(parser->frame)) {
            ld19_packet_t packet;
            ld19_byte_to_packet(parser->frame, &packet);
            parser->length = 0;
            return ld19_append_packet(parser, &packet, current, next);
        }
    }

    ld19_keep_frame_suffix(parser);
    ld19_discard_scan(parser, current);
    return SCAN_NEED_MORE;
}
