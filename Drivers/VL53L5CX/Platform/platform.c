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

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

static osSemaphoreId_t rx_done;
static volatile uint8_t rx_pending;
static volatile uint8_t rx_result;

/* 現設定の1フレームは452バイト。キャッシュラインを他の変数と共有しない */
static uint8_t rx_buffer[1024] __attribute__((aligned(32)));

uint8_t VL53L5CX_InitFrameDMA(void)
{
    if (rx_done == NULL) {
        rx_done = osSemaphoreNew(1U, 0U, NULL);
    }
    return (rx_done != NULL) ? 0U : 255U;
}

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

static bool FrameDMAIsIdle(const I2C_HandleTypeDef *i2c)
{
    DMA_HandleTypeDef *dma = i2c->hdmarx;
    return dma != NULL && dma->Instance != NULL &&
           HAL_I2C_GetState(i2c) == HAL_I2C_STATE_READY &&
           HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY &&
           (dma->Instance->CCR & DMA_CCR_EN) == 0U;
}

static uint8_t ReadFrameDMA(
	VL53L5CX_Platform *platform,
	uint8_t *dst,
	uint16_t size
)
{
    if (platform == NULL || platform->i2c == NULL || dst == NULL ||
        size == 0U || size > sizeof(rx_buffer) || rx_done == NULL) {
        return 255U;
    }

    /* バッファを再利用する前に、前回の転送が終了したことを確認する。 */
    if (rx_pending != 0U || !FrameDMAIsIdle(platform->i2c)) {
        return TOF_DMA_STOP_FAILED;
    }

    /* 前回の完了通知が遅れて届いていた場合に備えて消費する */
    (void)osSemaphoreAcquire(rx_done, 0U);
    rx_result = 255U;
    rx_pending = 1U;

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanInvalidateDCache_by_Addr(rx_buffer, size);
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        platform->i2c,
        platform->address,       /* 既存のRdMultiと同じ値をそのまま渡す */
        0x0000U,                 /* 測距フレームの先頭レジスタ */
        I2C_MEMADD_SIZE_16BIT,
        rx_buffer,
        size
    );

    if (status != HAL_OK) {
        rx_pending = 0U;
        return 255U;
    }

    /* 現プロジェクトは1 tick = 1 ms。ここで眠るのはToFタスクだけ */
	if (osSemaphoreAcquire(rx_done, 20U) != osOK) {
		I2C_HandleTypeDef *i2c = platform->i2c;
		HAL_StatusTypeDef abort_status = HAL_OK;

		/* タイムアウト直後に受信完了した場合は、中断要求を出さない */
		taskENTER_CRITICAL();
		if (rx_pending != 0U &&
			HAL_I2C_GetState(i2c) == HAL_I2C_STATE_BUSY_RX) {
			abort_status = HAL_I2C_Master_Abort_IT(i2c, platform->address);
		}
		taskEXIT_CRITICAL();

		if (abort_status != HAL_OK) {
			return TOF_DMA_STOP_FAILED;
		}

		/* 中断完了、またはタイムアウトと行き違った受信完了を待つ */
		if (osSemaphoreAcquire(rx_done, 20U) != osOK) {
			return TOF_DMA_STOP_FAILED;
		}

		if (!FrameDMAIsIdle(i2c)) {
			return TOF_DMA_STOP_FAILED;
		}

		/* 転送は終了済み。今回は破棄し、次のセンサINTから再開できる */
		return 255U;
	}

    if (rx_result != 0U) {
        return 255U;
    }

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr(rx_buffer, size);
    }

    memcpy(dst, rx_buffer, size);
    return 0U;
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

    if (p_platform->frame_size != 0U && RegisterAdress == 0x0000U &&
        size == p_platform->frame_size)
			return ReadFrameDMA(p_platform, p_values, (uint16_t)size);

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

static void FinishFrameDMA(I2C_HandleTypeDef *hi2c, uint8_t result)
{
    if (hi2c->Instance == I2C1 && rx_pending != 0U) {
        rx_result = result;
        rx_pending = 0U;
        (void)osSemaphoreRelease(rx_done);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    FinishFrameDMA(hi2c, 0U);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    FinishFrameDMA(hi2c, 255U);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    FinishFrameDMA(hi2c, 255U);
}
