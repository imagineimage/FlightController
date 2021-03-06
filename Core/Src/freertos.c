/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

typedef enum{
	I2C_read,
	I2C_write,
	I2C_readIT,
	I2C_writeIT,
	I2C_readDMA,
	I2C_writeDMA
}I2Ctask_t;

typedef struct I2CItem{
	I2Ctask_t type;
	uint16_t DevAddress;
	uint16_t MemAddress;
	uint16_t MemAddSize;
	uint8_t *pData;
	uint16_t Size;
} I2CItem_t;

uint8_t test = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for MPU9250_Task */
osThreadId_t MPU9250_TaskHandle;
uint32_t MPU9250_TaskBuffer[ 128 ];
osStaticThreadDef_t MPU9250_TaskControlBlock;
const osThreadAttr_t MPU9250_Task_attributes = {
  .name = "MPU9250_Task",
  .stack_mem = &MPU9250_TaskBuffer[0],
  .stack_size = sizeof(MPU9250_TaskBuffer),
  .cb_mem = &MPU9250_TaskControlBlock,
  .cb_size = sizeof(MPU9250_TaskControlBlock),
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for BME280_Task */
osThreadId_t BME280_TaskHandle;
uint32_t BME280_TaskBuffer[ 128 ];
osStaticThreadDef_t BME280_TaskControlBlock;
const osThreadAttr_t BME280_Task_attributes = {
  .name = "BME280_Task",
  .stack_mem = &BME280_TaskBuffer[0],
  .stack_size = sizeof(BME280_TaskBuffer),
  .cb_mem = &BME280_TaskControlBlock,
  .cb_size = sizeof(BME280_TaskControlBlock),
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for IST8310_Task */
osThreadId_t IST8310_TaskHandle;
uint32_t IST8310_TaskBuffer[ 128 ];
osStaticThreadDef_t IST8310_TaskControlBlock;
const osThreadAttr_t IST8310_Task_attributes = {
  .name = "IST8310_Task",
  .stack_mem = &IST8310_TaskBuffer[0],
  .stack_size = sizeof(IST8310_TaskBuffer),
  .cb_mem = &IST8310_TaskControlBlock,
  .cb_size = sizeof(IST8310_TaskControlBlock),
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for SD_Task */
osThreadId_t SD_TaskHandle;
uint32_t SD_TaskBuffer[ 1024 ];
osStaticThreadDef_t SD_TaskControlBlock;
const osThreadAttr_t SD_Task_attributes = {
  .name = "SD_Task",
  .stack_mem = &SD_TaskBuffer[0],
  .stack_size = sizeof(SD_TaskBuffer),
  .cb_mem = &SD_TaskControlBlock,
  .cb_size = sizeof(SD_TaskControlBlock),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AHRS_Task */
osThreadId_t AHRS_TaskHandle;
uint32_t AHRS_TaskBuffer[ 256 ];
osStaticThreadDef_t AHRS_TaskControlBlock;
const osThreadAttr_t AHRS_Task_attributes = {
  .name = "AHRS_Task",
  .stack_mem = &AHRS_TaskBuffer[0],
  .stack_size = sizeof(AHRS_TaskBuffer),
  .cb_mem = &AHRS_TaskControlBlock,
  .cb_size = sizeof(AHRS_TaskControlBlock),
  .priority = (osPriority_t) osPriorityRealtime1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void MPU9250_StartTask(void *argument);
extern void BME280_StartTask(void *argument);
extern void IST8310_StartTask(void *argument);
extern void SD_StartTask(void *argument);
extern void AHRS_StartTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  const uint8_t MPU9250_UPDATE_HZ = 200;
  const uint8_t BME280_UPDATE_HZ = 50;
  const uint8_t IST8310_UPDATE_HZ = 100;

  const uint32_t MPU9250_TICK = 1000/MPU9250_UPDATE_HZ;
  const uint32_t BME280_TICK = 1000/BME280_UPDATE_HZ;
  const uint32_t IST8310_TICK = 1000/IST8310_UPDATE_HZ;

//  if( osTimerStart( MPU9250_TimerHandle, MPU9250_TICK ) != osOK )
//  {
//    // The timer could not be set into the Active state.
//  }
//  if( osTimerStart( BME280_TimerHandle, BME280_TICK ) != osOK )
//  {
//    // The timer could not be set into the Active state.
//  }
//  if( osTimerStart( IST8310_TimerHandle, IST8310_TICK ) != osOK )
//  {
//    // The timer could not be set into the Active state.
//  }


  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MPU9250_Task */
  MPU9250_TaskHandle = osThreadNew(MPU9250_StartTask, NULL, &MPU9250_Task_attributes);

  /* creation of BME280_Task */
  BME280_TaskHandle = osThreadNew(BME280_StartTask, NULL, &BME280_Task_attributes);

  /* creation of IST8310_Task */
  IST8310_TaskHandle = osThreadNew(IST8310_StartTask, NULL, &IST8310_Task_attributes);

  /* creation of SD_Task */
  SD_TaskHandle = osThreadNew(SD_StartTask, NULL, &SD_Task_attributes);

  /* creation of AHRS_Task */
  AHRS_TaskHandle = osThreadNew(AHRS_StartTask, NULL, &AHRS_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
