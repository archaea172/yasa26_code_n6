#ifndef __LD19_H
#define __LD19_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ld19_config.h"

#define LD19_PACKET_SIZE 47u
#define LD19_POINTS_PER_PACKET 12u

typedef enum
{
    SCAN_OK,
    SCAN_NEED_MORE,
} ld19_scan_state_t;

typedef struct
{
    uint16_t rotation_speed;
    double first_degree;
    double last_degree;
    uint16_t distances_mm[LD19_POINTS_PER_PACKET];
    uint8_t intensity[LD19_POINTS_PER_PACKET];
    uint16_t time_stamp;
    uint8_t crc;
} ld19_packet_t;


typedef struct {
    uint16_t distance_mm;
    uint8_t intensity;
    float angle_deg; /* [0, 360), clockwise from the sensor's forward axis. */
} ld19_point_t;

typedef struct {
    ld19_point_t points[LD19_MAX_POINTS];  // 1回転分の点を書き込む配列
    size_t count;              // 実際に書き込んだ点の数
} ld19_scan_t;

typedef struct {
    uint8_t frame[LD19_PACKET_SIZE];
    size_t length;
    float previous_angle;
    bool have_previous_angle;
    bool collecting;
} ld19_parser_t;

/* Consume complete CRC-valid frames and move the remaining bytes to the front. */
size_t ld19_count_frames(uint8_t *buffer, size_t *length);

/* parserはゼロ初期化する。SCAN_OKでcurrentの1周が完成し、次の点はnextに入る。
 * 次の呼び出しからnextをcurrentにして、別の書き込み可能なバッファを渡す。 */
ld19_scan_state_t ld19_byte_parse(ld19_parser_t *parser, uint8_t byte,
                                  ld19_scan_t *current, ld19_scan_t *next);

#endif
