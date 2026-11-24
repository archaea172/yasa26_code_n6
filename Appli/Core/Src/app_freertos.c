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
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
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
  .stack_size = 128 * 4
};
/* Definitions for debugQueue */
osMessageQueueId_t debugQueueHandle;
const osMessageQueueAttr_t debugQueue_attributes = {
  .name = "debugQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  /* add mutexes, ... */
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
  /* Infinite loop */
  for(;;)
  {
    printf("Hello World!\r\n");
    osDelay(1000);
  }
  /* USER CODE END LD19Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

