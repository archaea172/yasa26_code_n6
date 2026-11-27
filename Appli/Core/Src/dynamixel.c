#include "dynamixel.h"

#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/* Static storage keeps large packets off the small FreeRTOS task stacks. */
static uint8_t tx_packet[DYNAMIXEL_MAX_PARAMS + 6U];
static uint8_t rx_packet[DYNAMIXEL_MAX_PARAMS + 6U];

static uint32_t transfer_timeout(UART_HandleTypeDef *huart, uint16_t length)
{
  uint32_t baud = huart->Init.BaudRate;
  uint32_t wire_ms = (baud == 0U) ? 0U :
      (uint32_t)(((uint64_t)length * 10000U + baud - 1U) / baud);
  return wire_ms + DYNAMIXEL_TIMEOUT_MS;
}

static HAL_StatusTypeDef transmit(UART_HandleTypeDef *huart,
                                  uint8_t *data, uint16_t length)
{
  if (huart == NULL || data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  uint8_t servo_uart = (huart->Instance == USART3);
  if (servo_uart)
  {
    /* Discard old ACKs/errors before sending, never after switching to RX. */
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                UART_CLEAR_FEF | UART_CLEAR_PEF);
    HAL_GPIO_WritePin(servo_DIR_GPIO_Port, servo_DIR_Pin, GPIO_PIN_SET);
    /* Prevent local bus echo from overflowing RDR while transmitting. */
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);
  }

  /* Blocking HAL_UART_Transmit waits for TC, including the final stop bit. */
  HAL_StatusTypeDef status = HAL_UART_Transmit(
      huart, data, length, transfer_timeout(huart, length));
  if (servo_uart)
  {
    SET_BIT(huart->Instance->CR1, USART_CR1_RE);
    HAL_GPIO_WritePin(servo_DIR_GPIO_Port, servo_DIR_Pin, GPIO_PIN_RESET);
  }
  return status;
}

void uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
  (void)transmit(huart, data, length);
}

void uart_recv(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
  if (huart == NULL || data == NULL || length == 0U)
  {
    return;
  }
  if (huart->Instance == USART3)
  {
    HAL_GPIO_WritePin(servo_DIR_GPIO_Port, servo_DIR_Pin, GPIO_PIN_RESET);
  }
  (void)HAL_UART_Receive(huart, data, length, transfer_timeout(huart, length));
}

static uint8_t send_instruction(uint8_t id, uint8_t inst,
                                const uint8_t *params, uint8_t params_len)
{
  if (id > DYNAMIXEL_BROADCAST_ID || params_len > DYNAMIXEL_MAX_PARAMS ||
      (params_len != 0U && params == NULL))
  {
    return 0U;
  }
  /* params may already point into tx_packet (write/sync-write builders). */
  if (params_len != 0U)
  {
    memmove(&tx_packet[5], params, params_len);
  }
  tx_packet[0] = 0xFFU;
  tx_packet[1] = 0xFFU;
  tx_packet[2] = id;
  tx_packet[3] = (uint8_t)(params_len + 2U);
  tx_packet[4] = inst;
  uint8_t sum = 0U;
  for (uint16_t i = 2U; i < (uint16_t)params_len + 5U; ++i)
  {
    sum = (uint8_t)(sum + tx_packet[i]);
  }
  tx_packet[params_len + 5U] = (uint8_t)~sum;
  return transmit(&huart3, tx_packet, (uint16_t)params_len + 6U) == HAL_OK;
}

void send_packet_v1(uint8_t id, uint8_t inst, uint8_t *params, uint8_t params_len)
{
  (void)send_instruction(id, inst, params, params_len);
}

static uint8_t receive_bytes(uint8_t *data, uint16_t length,
                             uint32_t start, uint32_t timeout)
{
  uint32_t elapsed = HAL_GetTick() - start;
  if (elapsed >= timeout)
  {
    return 0U;
  }
  return HAL_UART_Receive(&huart3, data, length, timeout - elapsed) == HAL_OK;
}

/* All receives in a transaction share one deadline, including header search. */
static uint16_t receive_status(uint32_t start, uint32_t timeout)
{
  uint8_t byte;
  uint8_t headers = 0U;
  HAL_GPIO_WritePin(servo_DIR_GPIO_Port, servo_DIR_Pin, GPIO_PIN_RESET);
  while (receive_bytes(&byte, 1U, start, timeout))
  {
    if (byte == 0xFFU)
    {
      if (headers < 2U)
      {
        ++headers;
      }
      continue;
    }
    if (headers != 2U || byte >= DYNAMIXEL_BROADCAST_ID)
    {
      headers = 0U;
      continue;
    }
    rx_packet[0] = 0xFFU;
    rx_packet[1] = 0xFFU;
    rx_packet[2] = byte;
    if (!receive_bytes(&rx_packet[3], 1U, start, timeout))
    {
      return 0U;
    }
    if (rx_packet[3] < 2U)
    {
      headers = 0U;
      continue;
    }
    if (!receive_bytes(&rx_packet[4], rx_packet[3], start, timeout))
    {
      return 0U;
    }
    return (uint16_t)rx_packet[3] + 4U;
  }
  return 0U;
}

static uint8_t checksum_valid(uint16_t length)
{
  uint8_t sum = 0U;
  for (uint16_t i = 2U; i < length; ++i)
  {
    sum = (uint8_t)(sum + rx_packet[i]);
  }
  return sum == 0xFFU;
}

uint8_t parse_status_packet_v1(uint8_t packet_len, uint8_t *id, uint8_t *params,
                               uint8_t *params_len, uint8_t *error,
                               uint8_t *cks_check)
{
  if (params_len != NULL) { *params_len = 0U; }
  if (cks_check != NULL) { *cks_check = 0U; }
  if (id != NULL) { *id = DYNAMIXEL_INVALID_ID; }
  if (error != NULL) { *error = 0U; }
  if (id == NULL || params_len == NULL || error == NULL || cks_check == NULL ||
      (packet_len != 0U && packet_len < 6U))
  {
    return 0U;
  }
  uint16_t length = receive_status(HAL_GetTick(),
      transfer_timeout(&huart3, (packet_len == 0U) ? sizeof(rx_packet) : packet_len));
  if (length == 0U) { return 0U; }
  *id = rx_packet[2];
  *error = rx_packet[4];
  *cks_check = checksum_valid(length);
  uint8_t count = (uint8_t)(length - 6U);
  if (*cks_check == 0U || (packet_len != 0U && length != packet_len) ||
      (count != 0U && params == NULL))
  {
    return 0U;
  }
  if (count != 0U) { memcpy(params, &rx_packet[5], count); }
  *params_len = count;
  return 1U;
}

static void consume_ack(uint8_t id)
{
  uint32_t start = HAL_GetTick();
  uint32_t timeout = transfer_timeout(&huart3, 6U);
  uint16_t length;
  while ((length = receive_status(start, timeout)) != 0U)
  {
    if (length == 6U && rx_packet[2] == id && checksum_valid(length))
    {
      return;
    }
  }
}

void dynamixel_write_v1(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len)
{
  if (data == NULL || data_len == 0U || data_len > DYNAMIXEL_MAX_PARAMS - 1U ||
      (uint16_t)address + data_len > 256U)
  {
    return;
  }
  tx_packet[5] = address;
  memcpy(&tx_packet[6], data, data_len);
  if (send_instruction(id, WRITE, &tx_packet[5], (uint8_t)(data_len + 1U)) &&
      id != DYNAMIXEL_BROADCAST_ID)
  {
    consume_ack(id);
  }
}

uint8_t dynamixel_read_v1(uint8_t id, uint8_t address, uint8_t data_len,
                          uint8_t *r_data, uint8_t *r_data_len)
{
  if (r_data_len != NULL) { *r_data_len = 0U; }
  if (id >= DYNAMIXEL_BROADCAST_ID || r_data == NULL || r_data_len == NULL ||
      data_len == 0U || data_len > DYNAMIXEL_MAX_PARAMS ||
      (uint16_t)address + data_len > 256U)
  {
    return 0U;
  }
  uint8_t params[2] = {address, data_len};
  if (!send_instruction(id, READ, params, sizeof(params))) { return 0U; }
  uint32_t start = HAL_GetTick();
  uint32_t timeout = transfer_timeout(&huart3, (uint16_t)data_len + 6U);
  uint16_t length;
  while ((length = receive_status(start, timeout)) != 0U)
  {
    if (rx_packet[2] != id || !checksum_valid(length)) { continue; }
    if (rx_packet[4] != 0U) { return 0U; }
    /* Ignore a delayed write ACK or unrelated response with another size. */
    if (length != (uint16_t)data_len + 6U) { continue; }
    memcpy(r_data, &rx_packet[5], data_len);
    *r_data_len = data_len;
    return 1U;
  }
  return 0U;
}

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  if (data_blk_ptr == NULL) { return crc_accum; }
  for (uint16_t i = 0U; i < data_blk_size; ++i)
  {
    crc_accum ^= (uint16_t)data_blk_ptr[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc_accum = (crc_accum & 0x8000U) ?
          (uint16_t)((crc_accum << 1) ^ 0x8005U) : (uint16_t)(crc_accum << 1);
    }
  }
  return crc_accum;
}

void dynamixel_sync_write_v1(uint8_t *ids, uint8_t address, uint8_t *data,
                             uint8_t data_len, uint8_t nb_ids)
{
  uint16_t count = 2U + ((uint16_t)data_len + 1U) * nb_ids;
  if (ids == NULL || data == NULL || data_len == 0U || nb_ids == 0U ||
      count > DYNAMIXEL_MAX_PARAMS || (uint16_t)address + data_len > 256U)
  {
    return;
  }
  uint16_t offset = 5U;
  tx_packet[offset++] = address;
  tx_packet[offset++] = data_len;
  for (uint16_t i = 0U; i < nb_ids; ++i)
  {
    if (ids[i] >= DYNAMIXEL_BROADCAST_ID) { return; }
    tx_packet[offset++] = ids[i];
    memcpy(&tx_packet[offset], &data[i * data_len], data_len);
    offset += data_len;
  }
  (void)send_instruction(DYNAMIXEL_BROADCAST_ID, SYNC_WRITE,
                         &tx_packet[5], (uint8_t)count);
}

static void write_word(uint8_t id, uint8_t address, uint16_t value)
{
  uint8_t data[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
  dynamixel_write_v1(id, address, data, sizeof(data));
}

static uint16_t clamp_word(uint16_t value, uint16_t max)
{
  return (value > max) ? max : value;
}

void dynamixel_set_punch(uint8_t id, uint16_t punch)
{
  punch = clamp_word(punch, MAX_TORQUE);
  write_word(id, PUNCH_ADDR, (punch < 32U) ? 32U : punch);
}

void dynamixel_set_joint_mode(uint8_t id)
{
  dynamixel_set_mode(id, CW_ANGLE_LIMIT, CCW_ANGLE_LIMIT);
}

void dynamixel_set_endless_turn(uint8_t id, uint16_t velocity, uint8_t rot_dir)
{
  if (rot_dir != CW_DIRECTION && rot_dir != CCW_DIRECTION) { return; }
  dynamixel_set_mode(id, 0U, 0U);
  uint16_t speed = clamp_word(velocity, MAX_MOVING_SPEED);
  if (rot_dir == CW_DIRECTION) { speed |= 0x0400U; }
  dynamixel_set_moving_speed(id, speed);
}

void dynamixel_set_mode(uint8_t id, uint16_t cw_angle_lim, uint16_t ccw_angle_lim)
{
  if (cw_angle_lim > CCW_ANGLE_LIMIT || ccw_angle_lim > CCW_ANGLE_LIMIT ||
      cw_angle_lim > ccw_angle_lim) { return; }
  uint8_t data[4] = {(uint8_t)cw_angle_lim, (uint8_t)(cw_angle_lim >> 8),
                     (uint8_t)ccw_angle_lim, (uint8_t)(ccw_angle_lim >> 8)};
  dynamixel_write_v1(id, CW_ANGLE_LIMIT_ADDR, data, sizeof(data));
}

void dynamixel_set_id(uint8_t id, uint8_t new_id)
{
  if (new_id < DYNAMIXEL_BROADCAST_ID)
  {
    dynamixel_write_v1(id, ID_ADDR, &new_id, 1U);
  }
}

void dynamixel_set_delay_time(uint8_t id, uint8_t delay_time)
{
  if (delay_time <= 254U)
  {
    dynamixel_write_v1(id, RETURN_DELAY_TIME_ADDR, &delay_time, 1U);
  }
}

void dynamixel_set_baudrate(uint8_t id, uint8_t baudrate)
{
  if (baudrate <= 254U)
  {
    /* Changes the device register only; USART3 must be reconfigured by caller. */
    dynamixel_write_v1(id, BAUD_RATE_ADDR, &baudrate, 1U);
  }
}

void dynamixel_set_led(uint8_t id, uint8_t enable)
{
  enable = (enable != 0U);
  dynamixel_write_v1(id, LED_ADDR, &enable, 1U);
}

void dynamixel_ping(uint8_t id)
{
  if (id < DYNAMIXEL_BROADCAST_ID && send_instruction(id, PING, NULL, 0U))
  {
    consume_ack(id);
  }
}

void dynamixel_action(void)
{
  (void)send_instruction(DYNAMIXEL_BROADCAST_ID, ACTION, NULL, 0U);
}

void dynamixel_reset(uint8_t id)
{
  if (id < DYNAMIXEL_BROADCAST_ID && send_instruction(id, FACTORY_RESET, NULL, 0U))
  {
    consume_ack(id);
  }
}

void dynamixel_set_torque_enable(uint8_t id, uint8_t enable)
{
  enable = (enable != 0U);
  dynamixel_write_v1(id, TORQUE_ENABLE_ADDR, &enable, 1U);
}

void dynamixel_set_goal_position(uint8_t id, uint16_t position)
{
  write_word(id, GOAL_POSITION_ADDR, clamp_word(position, CCW_ANGLE_LIMIT));
}

void dynamixel_set_moving_speed(uint8_t id, uint16_t velocity)
{
  write_word(id, MOVING_SPEED_ADDR, clamp_word(velocity, 0x07FFU));
}

void dynamixel_set_position_and_velocity(uint8_t id, uint16_t position, uint16_t velocity)
{
  position = clamp_word(position, CCW_ANGLE_LIMIT);
  velocity = clamp_word(velocity, MAX_MOVING_SPEED);
  uint8_t data[4] = {(uint8_t)position, (uint8_t)(position >> 8),
                     (uint8_t)velocity, (uint8_t)(velocity >> 8)};
  dynamixel_write_v1(id, GOAL_POSITION_ADDR, data, sizeof(data));
}

void dynamixel_set_max_torque(uint8_t id, uint16_t max_torque)
{
  write_word(id, MAX_TORQUE_ADDR, clamp_word(max_torque, MAX_TORQUE));
}

void dynamixel_set_compliance_margin(uint8_t id, uint8_t cw_margin, uint8_t ccw_margin)
{
  uint8_t data[2] = {cw_margin, ccw_margin};
  dynamixel_write_v1(id, CW_COMP_MARGIN_ADDR, data, sizeof(data));
}

void dynamixel_set_compliance_slope(uint8_t id, uint8_t cw_slope, uint8_t ccw_slope)
{
  uint8_t data[2] = {cw_slope, ccw_slope};
  dynamixel_write_v1(id, CW_COMP_SLOPE_ADDR, data, sizeof(data));
}

static uint16_t read_word(uint8_t id, uint8_t address)
{
  uint8_t data[2];
  uint8_t length;
  if (!dynamixel_read_v1(id, address, sizeof(data), data, &length))
  {
    return UINT16_MAX;
  }
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint8_t read_byte(uint8_t id, uint8_t address)
{
  uint8_t data;
  uint8_t length;
  return dynamixel_read_v1(id, address, 1U, &data, &length) ? data : UINT8_MAX;
}

uint16_t dynamixel_read_present_position(uint8_t id)
{
  return read_word(id, PRESENT_POSITION_ADDR);
}

uint16_t dynamixel_read_present_moving_speed(uint8_t id)
{
  return read_word(id, PRESENT_SPEED_ADDR);
}

float dynamixel_read_present_load(uint8_t id)
{
  uint16_t value = read_word(id, PRESENT_LOAD_ADDR);
  return (value == UINT16_MAX) ? NAN : value_to_load(value);
}

float dynamixel_read_present_voltage(uint8_t id)
{
  uint8_t value = read_byte(id, PRESENT_VOLTAGE_ADDR);
  return (value == UINT8_MAX) ? NAN : (float)value / 10.0f;
}

uint8_t dynamixel_read_present_temperature(uint8_t id)
{
  return read_byte(id, PRESENT_TEMP_ADDR);
}

uint8_t dynamixel_read_registered(uint8_t id)
{
  return read_byte(id, REGISTERED_ADDR);
}

uint8_t dynamixel_read_baudrate(uint8_t id)
{
  return read_byte(id, BAUD_RATE_ADDR);
}

uint8_t dynamixel_read_moving(uint8_t id)
{
  return read_byte(id, MOVING_ADDR);
}

uint8_t dynamixel_read_id(void)
{
  for (uint16_t id = 0U; id < DYNAMIXEL_BROADCAST_ID; ++id)
  {
    uint8_t value = read_byte((uint8_t)id, ID_ADDR);
    if (value == id) { return value; }
  }
  return DYNAMIXEL_INVALID_ID;
}

uint8_t baudrate_to_value(uint32_t baudrate)
{
  if (baudrate == 0U) { return UINT8_MAX; }
  uint32_t divisor = (uint32_t)((2000000ULL + baudrate / 2U) / baudrate);
  if (divisor < 1U) { divisor = 1U; }
  if (divisor > 255U) { divisor = 255U; }
  return (uint8_t)(divisor - 1U);
}

uint32_t value_to_baudrate(uint8_t value)
{
  return (value == UINT8_MAX) ? 0U : 2000000U / ((uint32_t)value + 1U);
}

static uint16_t scaled_value(float value, float scale, uint16_t maximum)
{
  if (!(value > 0.0f)) { return 0U; } /* Also handles NaN. */
  float raw = value * scale;
  if (raw >= (float)maximum) { return maximum; }
  return (uint16_t)(raw + 0.5f);
}

uint16_t pos_to_value(float pos_deg)
{
  return scaled_value(pos_deg, 1023.0f / 300.0f, CCW_ANGLE_LIMIT);
}

uint16_t vel_to_value(float speed_deg_s)
{
  return scaled_value(speed_deg_s, 1.0f / (0.111f * 6.0f), MAX_MOVING_SPEED);
}

uint16_t vel_to_value_wheel_mode(float speed_pct)
{
  uint16_t value = scaled_value(fabsf(speed_pct), 1023.0f / 100.0f, MAX_MOVING_SPEED);
  return (speed_pct < 0.0f) ? (uint16_t)(value | 0x0400U) : value;
}

float value_to_load(uint16_t value)
{
  float percent = (float)(value & 0x03FFU) * (100.0f / 1023.0f);
  return (value & 0x0400U) ? -percent : percent;
}
