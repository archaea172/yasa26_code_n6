#ifndef __ROBOT_DRIVE_H
#define __ROBOT_DRIVE_H

#define RIGHT_ID 2
#define LEFT_ID 1

#define DRIVE_TRACK_WIDTH_M 0.21f
#define DRIVE_WHEEL_RADIUS_M 0.035f

#define DRIVE_RIGHT_FULL_OUTPUT_RPM 59.0f
#define DRIVE_LEFT_FULL_OUTPUT_RPM 59.0f
#define DRIVE_RIGHT_FORWARD_SIGN 1.0f
#define DRIVE_LEFT_FORWARD_SIGN (-1.0f)

typedef struct {
    float right_percent;
    float left_percent;
} Drive_Output;

void drive_init();
void drive_send(float v_parallel_m_s, float omega_rad_s);

#endif
