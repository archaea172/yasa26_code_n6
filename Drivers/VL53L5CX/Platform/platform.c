/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */



#include "platform.h"

uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1U);
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1U);
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    if (size > UINT16_MAX) {
        return 255U;
    }

    return HAL_I2C_Mem_Write(
        p_platform->i2c,
        p_platform->address,
        RegisterAdress,
        I2C_MEMADD_SIZE_16BIT,
        p_values,
        (uint16_t)size,
        HAL_MAX_DELAY
    ) == HAL_OK ? 0U : 255U;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    if (size > UINT16_MAX) {
        return 255U;
    }

    return HAL_I2C_Mem_Read(
        p_platform->i2c,
        p_platform->address,
        RegisterAdress,
        I2C_MEMADD_SIZE_16BIT,
        p_values,
        (uint16_t)size,
        HAL_MAX_DELAY
    ) == HAL_OK ? 0U : 255U;
}

uint8_t VL53L5CX_Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	HAL_GPIO_WritePin(p_platform->lpn_port, p_platform->lpn_pin, GPIO_PIN_RESET);
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	VL53L5CX_WaitMs(p_platform, 100);

	HAL_GPIO_WritePin(p_platform->lpn_port, p_platform->lpn_pin, GPIO_PIN_SET);
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L5CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
    (void)p_platform;

    if (osKernelGetState() == osKernelRunning) {
        return osDelay(TimeMs) == osOK ? 0U : 255U;
    }

    HAL_Delay(TimeMs);
    return 0U;
}
