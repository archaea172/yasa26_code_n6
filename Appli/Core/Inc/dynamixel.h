#ifndef __DYNAMIXEL_H
#define __DYNAMIXEL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AX-style control table, Protocol 1.0, USART3, PA1: high = TX, low = RX.
 * Blocking API: call from one task, or serialize the entire transaction with
 * an external mutex. Do not call from an ISR or share USART3 with another user.
 * MX_GPIO_Init() and MX_USART3_UART_Init() must have completed before use.
 */
#define DYNAMIXEL_BROADCAST_ID       0xFEU
#define DYNAMIXEL_INVALID_ID         0xFFU
#define DYNAMIXEL_MAX_PARAMS         253U
#ifndef DYNAMIXEL_TIMEOUT_MS
#define DYNAMIXEL_TIMEOUT_MS         20U
#endif

// instructions
#define PING                  0x01
#define READ                  0x02
#define WRITE                 0x03
#define REG_WRITE             0x04
#define ACTION                0x05
#define FACTORY_RESET         0x06
#define REBOOT                0x08
#define SYNC_WRITE            0x83
#define BULK_READ             0x92

// errors
#define INPUT_VOLTAGE_ERROR   0x01
#define ANGLE_LIMIT_ERROR     0x02
#define OVERHEATING_ERROR     0x04
#define RANGE_ERROR           0x08
#define CHECKSUL_ERROR        0x10
#define CHECKSUM_ERROR        CHECKSUL_ERROR
#define OVERLOAD_ERROR        0x20
#define INSTRUCTION_ERROR_V1  0x40

// control table
// EEPROM AREA
#define ID_ADDR                  3
#define BAUD_RATE_ADDR           4
#define RETURN_DELAY_TIME_ADDR   5
#define CW_ANGLE_LIMIT_ADDR      6
#define CCW_ANGLE_LIMIT_ADDR     8
#define TEMP_LIMIT_ADDR         11
#define MIN_VOLTAGE_LIMIT_ADDR  12
#define MAX_VOLTAGE_LIMIT_ADDR  13
#define MAX_TORQUE_ADDR         14
#define STATUS_RET_LEVEL_ADDR   16
#define ALARM_LED_ADDR          17
#define SHUTDOWN_ADDR           18

// RAM AREA
#define TORQUE_ENABLE_ADDR      24
#define LED_ADDR                25
#define CW_COMP_MARGIN_ADDR     26
#define CCW_COMP_MARGIN_ADDR    27
#define CW_COMP_SLOPE_ADDR      28
#define CCW_COMP_SLOPE_ADDR     29
#define GOAL_POSITION_ADDR      30
#define MOVING_SPEED_ADDR       32
#define TORQUE_LIMIT_ADDR       34
#define PRESENT_POSITION_ADDR   36
#define PRESENT_SPEED_ADDR      38
#define PRESENT_LOAD_ADDR       40
#define PRESENT_VOLTAGE_ADDR    42
#define PRESENT_TEMP_ADDR       43
#define REGISTERED_ADDR         44
#define MOVING_ADDR             46
#define LOCK_ADDR               47
#define PUNCH_ADDR              48

// Data
#define MAX_TORQUE            1023
#define MAX_MOVING_SPEED      1023
#define CW_ANGLE_LIMIT           0
#define CCW_ANGLE_LIMIT       1023
#define CW_DIRECTION             0
#define CCW_DIRECTION            1
#define LOCK                     1
#define JOINT_MODE               0
#define WHEEL_MODE               1


void uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
void uart_recv(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);

/* Raw send does not consume a response; use parse_status_packet_v1 afterwards.
 * Write helpers consume an optional unicast ACK, waiting at most the receive
 * timeout if Status Return Level is 0/1. Broadcast writes do not wait.
 */
void send_packet_v1(uint8_t id, uint8_t inst, uint8_t *params, uint8_t params_len);
void dynamixel_write_v1(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len);
/* Returns 1 on success, 0 on invalid arguments, UART/packet/device error.
 * r_data must hold data_len bytes; *r_data_len is zero on failure.
 * Broadcast reads are rejected. Maximum read size is DYNAMIXEL_MAX_PARAMS.
 */
uint8_t dynamixel_read_v1(uint8_t id, uint8_t address, uint8_t data_len, uint8_t *r_data, uint8_t *r_data_len);
/* Receives the next status packet from USART3. packet_len is the total size
 * including header/checksum (minimum 6), or 0 to accept any size.
 * params must hold packet_len - 6 bytes, or DYNAMIXEL_MAX_PARAMS for size 0.
 * Returns 1 for a valid packet (even if *error != 0), otherwise 0.
 * *cks_check is 1 only for a valid checksum; *params_len is zero on failure.
 */
uint8_t parse_status_packet_v1(uint8_t packet_len, uint8_t *id, uint8_t *params, uint8_t *params_len, uint8_t *error, uint8_t *cks_check);
/* Protocol 2.0 CRC utility only; packet APIs above use the v1 checksum. */
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
/* data is packed by device: data_len bytes for ids[0], then ids[1], etc. */
void dynamixel_sync_write_v1(uint8_t *ids, uint8_t address, uint8_t *data, uint8_t data_len, uint8_t nb_ids);
void dynamixel_set_punch(uint8_t id, uint16_t punch);
void dynamixel_set_joint_mode(uint8_t id);
void dynamixel_set_endless_turn(uint8_t id, uint16_t velocity, uint8_t rot_dir);
void dynamixel_set_mode(uint8_t id, uint16_t cw_angle_lim, uint16_t ccw_angle_lim);
void dynamixel_set_id(uint8_t id, uint8_t new_id);
void dynamixel_set_delay_time(uint8_t id, uint8_t delay_time);
void dynamixel_set_baudrate(uint8_t id, uint8_t baudrate);
void dynamixel_set_led(uint8_t id, uint8_t enable);
void dynamixel_ping(uint8_t id);
void dynamixel_action(void);
void dynamixel_reset(uint8_t id);
void dynamixel_set_torque_enable(uint8_t id, uint8_t enable);
void dynamixel_set_goal_position(uint8_t id, uint16_t position);
void dynamixel_set_moving_speed(uint8_t id, uint16_t velocity);
void dynamixel_set_position_and_velocity(uint8_t id, uint16_t position, uint16_t velocity);
void dynamixel_set_max_torque(uint8_t id, uint16_t max_torque);
void dynamixel_set_compliance_margin(uint8_t id, uint8_t cw_margin, uint8_t ccw_margin);
void dynamixel_set_compliance_slope(uint8_t id, uint8_t cw_slope, uint8_t ccw_slope);

// read: integer helpers return UINT16_MAX/UINT8_MAX on failure; floats return NAN.
uint16_t dynamixel_read_present_position(uint8_t id);
uint16_t dynamixel_read_present_moving_speed(uint8_t id);
float dynamixel_read_present_load(uint8_t id);
float dynamixel_read_present_voltage(uint8_t id);
uint8_t dynamixel_read_present_temperature(uint8_t id);
uint8_t dynamixel_read_registered(uint8_t id);
uint8_t dynamixel_read_baudrate(uint8_t id);
uint8_t dynamixel_read_moving(uint8_t id);
/* Scans IDs 0..253 and returns the first responding ID, or INVALID_ID.
 * This can take several seconds when no device responds.
 */
uint8_t dynamixel_read_id(void);

// util
/* Baud register conversion: 2,000,000 / (value + 1), values 0..254.
 * baudrate_to_value returns UINT8_MAX for zero; other inputs are clamped.
 * value_to_baudrate returns 0 for the reserved value 255.
 */
uint8_t baudrate_to_value(uint32_t baudrate);
uint32_t value_to_baudrate(uint8_t value);
/* AX position: 0..300 degrees. Joint speed: nonnegative degrees/second;
 * register value 0 means maximum speed, not stop, in joint mode.
 * Wheel output: signed percent, positive CCW / negative CW.
 * Load: signed percent, positive CCW / negative CW.
 */
uint16_t pos_to_value(float pos_deg);
uint16_t vel_to_value(float speed_deg_s);
uint16_t vel_to_value_wheel_mode(float speed_pct);
float value_to_load(uint16_t value);
#ifdef __cplusplus
}
#endif
#endif
