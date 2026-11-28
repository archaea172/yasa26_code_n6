#include "robot_drive.h"
#include "dynamixel.h"

Drive_Output Drive_Convert(float v_m_s, float omega_rad_s)
{
    const Drive_Output stop = {0.0f, 0.0f};
    if (!isfinite(v_m_s) || !isfinite(omega_rad_s)) return stop;

    const float rad_s_to_rpm = 60.0f / (2.0f * 3.14159265358979323846f);
    float right_rpm = (v_m_s + omega_rad_s * DRIVE_TRACK_WIDTH_M * 0.5f)
                    / DRIVE_WHEEL_RADIUS_M * rad_s_to_rpm;
    float left_rpm = (v_m_s - omega_rad_s * DRIVE_TRACK_WIDTH_M * 0.5f)
                   / DRIVE_WHEEL_RADIUS_M * rad_s_to_rpm;
    Drive_Output output = {
        right_rpm / DRIVE_RIGHT_FULL_OUTPUT_RPM * 100.0f * DRIVE_RIGHT_FORWARD_SIGN,
        left_rpm / DRIVE_LEFT_FULL_OUTPUT_RPM * 100.0f * DRIVE_LEFT_FORWARD_SIGN
    };
    if (!isfinite(output.right_percent) || !isfinite(output.left_percent)) return stop;

    float peak = fmaxf(fabsf(output.right_percent), fabsf(output.left_percent));
    if (peak > 100.0f) {
        float scale = 100.0f / peak;
        output.right_percent *= scale;
        output.left_percent *= scale;
    }
    return output;
}

void drive_init()
{
    dynamixel_set_torque_enable(RIGHT_ID, 0U);
    dynamixel_set_torque_enable(LEFT_ID, 0U);

    dynamixel_set_mode(RIGHT_ID, 0U, 0U);
    dynamixel_set_mode(LEFT_ID, 0U, 0U);

    dynamixel_set_torque_enable(RIGHT_ID, 1U);
    dynamixel_set_torque_enable(LEFT_ID, 1U);
}

void drive_send(float v_parallel_m_s, float omega_rad_s)
{
    Drive_Output output = Drive_Convert(v_parallel_m_s, omega_rad_s);
    uint16_t right = vel_to_value_wheel_mode(output.right_percent);
    uint16_t left = vel_to_value_wheel_mode(output.left_percent);
    uint8_t ids[] = {RIGHT_ID, LEFT_ID};
    uint8_t data[] = {
        (uint8_t)right, (uint8_t)(right >> 8),
        (uint8_t)left, (uint8_t)(left >> 8)
    };
    dynamixel_sync_write_v1(ids, MOVING_SPEED_ADDR, data, 2U, 2U);
}
