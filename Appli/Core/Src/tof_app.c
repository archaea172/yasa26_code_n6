#include "tof_app.h"

#include <string.h>

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

static VL53L5CX_Configuration tof_dev;
static VL53L5CX_ResultsData tof_results;

static uint8_t tof_started = 0U;

uint8_t TOF_App_Init(void)
{
    uint8_t status;
    uint8_t is_alive = 0U;

    memset(&tof_dev, 0, sizeof(tof_dev));
    memset(&tof_results, 0, sizeof(tof_results));

    tof_dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    tof_dev.platform.i2c = &hi2c1;
    tof_dev.platform.lpn_port = ToF_lpn_GPIO_Port;
    tof_dev.platform.lpn_pin = ToF_lpn_Pin;

    status = VL53L5CX_Reset_Sensor(&tof_dev.platform);

    if (status != VL53L5CX_STATUS_OK) return status;

    status = vl53l5cx_is_alive(&tof_dev, &is_alive);
    if (status != VL53L5CX_STATUS_OK) return status;
    if (is_alive == 0U) return VL53L5CX_STATUS_ERROR;

    status = vl53l5cx_init(&tof_dev);
    if (status != VL53L5CX_STATUS_OK) return status;

    status = vl53l5cx_set_resolution(&tof_dev, VL53L5CX_RESOLUTION_8X8);
    if (status != VL53L5CX_STATUS_OK) return status;

    /* Match the 100 ms temporal spacing used by the controller's training. */
    status = vl53l5cx_set_ranging_frequency_hz(&tof_dev, 10U);
    if (status != VL53L5CX_STATUS_OK) return status;

    status = VL53L5CX_InitFrameDMA();
    if (status != VL53L5CX_STATUS_OK) return status;

    status = vl53l5cx_start_ranging(&tof_dev);
    if (status != VL53L5CX_STATUS_OK) return status;
    
    tof_dev.platform.frame_size = tof_dev.data_read_size;
    tof_started = 1U;

    return VL53L5CX_STATUS_OK;
}

uint8_t TOF_App_Process(uint8_t *p_new_frame)
{
    uint8_t status;

    if (p_new_frame == NULL) return VL53L5CX_STATUS_INVALID_PARAM;

    *p_new_frame = 0U;

    if (tof_started == 0U) return VL53L5CX_STATUS_ERROR;

    status = vl53l5cx_get_ranging_data(
        &tof_dev,
        &tof_results
    );
    if (status != VL53L5CX_STATUS_OK) return status;

    *p_new_frame = 1U;

    return VL53L5CX_STATUS_OK;
}

const VL53L5CX_ResultsData *TOF_App_GetResults(void)
{
    return &tof_results;
}

uint8_t TOF_App_Stop(void)
{
    uint8_t status;

    if (tof_started == 0U) {
        return VL53L5CX_STATUS_OK;
    }

    status = vl53l5cx_stop_ranging(&tof_dev);

    if (status == VL53L5CX_STATUS_OK) {
        tof_started = 0U;
    }

    return status;
}
