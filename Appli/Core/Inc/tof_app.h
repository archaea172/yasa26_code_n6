#ifndef __TOF_APP_H
#define __TOF_APP_H

#include <stdint.h>
#include "vl53l5cx_api.h"

uint8_t TOF_App_Init(void);

uint8_t TOF_App_Process(uint8_t *p_new_frame);

const VL53L5CX_ResultsData *TOF_App_GetResults(void);

uint8_t TOF_App_Stop(void);

#endif
