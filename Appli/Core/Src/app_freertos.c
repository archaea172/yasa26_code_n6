/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stream_buffer.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "tof_app.h"
#include "dynamixel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t length;
  char data[126];
} debug_message_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart8;

StreamBufferHandle_t ld19_stream;
volatile uint32_t ld19_rx_errors;

uint8_t ld19_dma_rx[LD19_DMA_SIZE] __NON_CACHEABLE __attribute__((aligned(32)));

static ld19_scan_t ld19_scans[3];
static ld19_scan_t *ld19_latest_scan = &ld19_scans[2];
static osMutexId_t ld19_scan_mutex;
/* USER CODE END Variables */
/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for LD19Task */
osThreadId_t LD19TaskHandle;
const osThreadAttr_t LD19Task_attributes = {
  .name = "LD19Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for mclTask */
osThreadId_t mclTaskHandle;
const osThreadAttr_t mclTask_attributes = {
  .name = "mclTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for vl53l5Task */
osThreadId_t vl53l5TaskHandle;
const osThreadAttr_t vl53l5Task_attributes = {
  .name = "vl53l5Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 4096 * 4
};
/* Definitions for dynamixelTask */
osThreadId_t dynamixelTaskHandle;
const osThreadAttr_t dynamixelTask_attributes = {
  .name = "dynamixelTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for debugQueue */
osMessageQueueId_t debugQueueHandle;
const osMessageQueueAttr_t debugQueue_attributes = {
  .name = "debugQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static HAL_StatusTypeDef ld19_start_rx(void);

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  ld19_scan_mutex = osMutexNew(NULL);
  if (ld19_scan_mutex == NULL)
    Error_Handler();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of debugQueue */
  debugQueueHandle = osMessageQueueNew (16, sizeof(debug_message_t), &debugQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of debugTask */
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* creation of LD19Task */
  LD19TaskHandle = osThreadNew(StartLD19Task, NULL, &LD19Task_attributes);

  /* creation of mclTask */
  mclTaskHandle = osThreadNew(StartMclTask, NULL, &mclTask_attributes);

  /* creation of vl53l5Task */
  vl53l5TaskHandle = osThreadNew(StartVl53l5Task, NULL, &vl53l5Task_attributes);

  /* creation of dynamixelTask */
  dynamixelTaskHandle = osThreadNew(StartDynamixelTask, NULL, &dynamixelTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the debugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN debugTask */
  /* Infinite loop */
  debug_message_t request;
  const uint32_t txFlags = DEBUG_TX_DONE | DEBUG_TX_ERROR;
  for(;;)
  {
    if (osOK != osMessageQueueGet(debugQueueHandle, &request, NULL, osWaitForever))
      continue;

    osThreadFlagsClear(txFlags);

    if (HAL_OK != HAL_UART_Transmit_DMA(&huart1, (uint8_t *)request.data, request.length))
      continue;

    uint32_t result = osThreadFlagsWait(txFlags, osFlagsWaitAny, 100U);
    if (result != DEBUG_TX_DONE)
    {
      /* エラーまたはタイムアウト。バッファを再利用する前に送信を止める */
      if (HAL_UART_AbortTransmit(&huart1) != HAL_OK)
      {
        osThreadSuspend(osThreadGetId());
      }
    }
  }
  /* USER CODE END debugTask */
}

/* USER CODE BEGIN Header_StartLD19Task */
/**
* @brief Function implementing the LD19Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLD19Task */
void StartLD19Task(void *argument)
{
  /* USER CODE BEGIN LD19Task */
  ld19_parser_t parser = {0};
  ld19_scan_t *current = &ld19_scans[0];
  ld19_scan_t *next = &ld19_scans[1];
  uint8_t receive_data[LD19_DMA_HALF];
  uint32_t handled_errors = 0;
  HAL_StatusTypeDef rx_status = HAL_ERROR;
  ld19_stream = xStreamBufferCreate(2048, 1);
  if (ld19_stream == NULL)
    Error_Handler();

  for(;;)
  {
    uint32_t errors = ld19_rx_errors;
    if (rx_status != HAL_OK || errors != handled_errors)
    {
      handled_errors = errors;
      parser = (ld19_parser_t){0};
      current->count = 0;
      next->count = 0;
      rx_status = ld19_start_rx();
      if (rx_status != HAL_OK)
      {
        osDelay(pdMS_TO_TICKS(20));
        continue;
      }
    }

    /* DMAがエラーで停止した場合も、タイムアウトで再始動処理に戻る。 */
    size_t received = xStreamBufferReceive(
      ld19_stream,
      receive_data,
      sizeof receive_data,
      pdMS_TO_TICKS(20)
    );
    if (handled_errors != ld19_rx_errors)
      continue;

    for (size_t i = 0; i < received; ++i)
    {
      if (ld19_byte_parse(&parser, receive_data[i], current, next) == SCAN_OK)
      {
        if (osMutexAcquire(ld19_scan_mutex, osWaitForever) != osOK)
          Error_Handler();
        ld19_scan_t *spare = ld19_latest_scan;
        ld19_latest_scan = current;
        osMutexRelease(ld19_scan_mutex);
        current = next;
        next = spare;
      }
    }
  }
  /* USER CODE END LD19Task */
}

/* USER CODE BEGIN Header_StartMclTask */
/**
* @brief Function implementing the mclTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMclTask */
void StartMclTask(void *argument)
{
  /* USER CODE BEGIN mclTask */
  static ld19_scan_t scan;
  /* Infinite loop */
  for(;;)
  {
    if(!ld19_copy_latest_scan(&scan))
      continue;

    printf("distance:%d mm\r\n", scan.points[0].distance_mm);
    osDelay(100);
  }
  /* USER CODE END mclTask */
}

/* USER CODE BEGIN Header_StartVl53l5Task */
/**
* @brief Function implementing the vl53l5Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVl53l5Task */
void StartVl53l5Task(void *argument)
{
  /* USER CODE BEGIN vl53l5Task */
  uint8_t ToF_zone = 64;
  int16_t distance_mm[ToF_zone];

  uint8_t ToF_status = TOF_App_Init();
  if (ToF_status != VL53L5CX_STATUS_OK)
  {
    printf("ToF init failed: status=%u\r\n", (unsigned)ToF_status);
    osThreadSuspend(osThreadGetId());
  }
  /* Infinite loop */
  for(;;)
  {
    if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)))
      continue;

    uint8_t new_frame = 0U;
    ToF_status = TOF_App_Process(&new_frame);
    if (ToF_status != VL53L5CX_STATUS_OK)
      continue;

    if (new_frame != 0U)
    {
      const VL53L5CX_ResultsData *results = TOF_App_GetResults();
      for (uint32_t zone = 0U; zone < ToF_zone; ++zone)
      {
        uint32_t target = zone * VL53L5CX_NB_TARGET_PER_ZONE;
        // uint8_t target_status = results->target_status[target];
        distance_mm[zone] = results->distance_mm[target];
      }
      printf("distance: %d\r\n", distance_mm[32]);
    }
  }
  /* USER CODE END vl53l5Task */
}

/* USER CODE BEGIN Header_StartDynamixelTask */
/**
* @brief Function implementing the dynamixelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDynamixelTask */
void StartDynamixelTask(void *argument)
{
  /* USER CODE BEGIN dynamixelTask */
  /* Infinite loop */
  for(;;)
  {
    dynamixel_set_led(1, 1);
    osDelay(1000);
    dynamixel_set_led(1, 0);
    osDelay(1000);
  }
  /* USER CODE END dynamixelTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
bool ld19_copy_latest_scan(ld19_scan_t *scan)
{
  if (scan == NULL || ld19_scan_mutex == NULL ||
      osMutexAcquire(ld19_scan_mutex, osWaitForever) != osOK)
    return false;

  size_t count = ld19_latest_scan->count;
  if (count != 0)
  {
    memcpy(scan->points, ld19_latest_scan->points, count * sizeof scan->points[0]);
    scan->count = count;
  }
  osMutexRelease(ld19_scan_mutex);
  return count != 0;
}

static HAL_StatusTypeDef ld19_start_rx(void)
{
  /* The LD19 may already be transmitting. Arm DMA before enabling UART8. */
  __HAL_UART_DISABLE(&huart8);
  if (HAL_UART_AbortReceive(&huart8) != HAL_OK)
    return HAL_ERROR;

  xStreamBufferReset(ld19_stream);
  HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart8, ld19_dma_rx, sizeof ld19_dma_rx);
  if (status == HAL_OK)
    __HAL_UART_ENABLE(&huart8);
  return status;
}

int _write(int file, char *ptr, int len)
{
  if (file != STDOUT_FILENO && file != STDERR_FILENO)
  {
    errno = EBADF;
    return -1;
  }
  if (len == 0)
  {
    return 0;
  }
  if (len < 0 || ptr == NULL)
  {
    errno = EINVAL;
    return -1;
  }
  if (debugQueueHandle == NULL || __get_IPSR() != 0U)
  {
    errno = EAGAIN;
    return -1;
  }

  debug_message_t message = {0};
  int written = 0;
  while (written < len)
  {
    size_t count = (size_t)(len - written);
    if (count > sizeof(message.data))
    {
      count = sizeof(message.data);
    }
    memcpy(message.data, ptr + written, count);
    message.length = (uint16_t)count;
    if (osMessageQueuePut(debugQueueHandle, &message, 0U, 0U) != osOK)
    {
      if (written == 0)
      {
        errno = EAGAIN;
        return -1;
      }
      break;
    }
    written += (int)count;
  }
  return written;
}

/* USER CODE END Application */

