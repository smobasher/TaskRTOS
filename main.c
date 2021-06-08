/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <FreeRTOS.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task03 */
osThreadId_t Task03Handle;
const osThreadAttr_t Task03_attributes = {
  .name = "Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task04 */
osThreadId_t Task04Handle;
const osThreadAttr_t Task04_attributes = {
  .name = "Task04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task05 */
osThreadId_t Task05Handle;
const osThreadAttr_t Task05_attributes = {
  .name = "Task05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Queue01 */
osMessageQueueId_t Queue01Handle;
const osMessageQueueAttr_t Queue01_attributes = {
  .name = "Queue01"
};
/* Definitions for Queue02 */
osMessageQueueId_t Queue02Handle;
const osMessageQueueAttr_t Queue02_attributes = {
  .name = "Queue02"
};
/* Definitions for Queue03 */
osMessageQueueId_t Queue03Handle;
const osMessageQueueAttr_t Queue03_attributes = {
  .name = "Queue03"
};
/* Definitions for Queue04 */
osMessageQueueId_t Queue04Handle;
const osMessageQueueAttr_t Queue04_attributes = {
  .name = "Queue04"
};
/* Definitions for Queue05 */
osMessageQueueId_t Queue05Handle;
const osMessageQueueAttr_t Queue05_attributes = {
  .name = "Queue05"
};
/* USER CODE BEGIN PV */
TaskHandle_t xHandle01 = NULL;
TaskHandle_t xHandle02 = NULL;
TaskHandle_t xHandle03 = NULL;
TaskHandle_t xHandle04 = NULL;
TaskHandle_t xHandle05 = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */
bool ChangeTask(char* taskName, int priority);
void DumpTasks();
void AddTask(char* taskName, int priority);
bool DeleteTask(char* taskName);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for(i=0; i<len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue01 */
  Queue01Handle = osMessageQueueNew (30, sizeof(uint8_t), &Queue01_attributes);

  /* creation of Queue02 */
  Queue02Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue02_attributes);

  /* creation of Queue03 */
  Queue03Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue03_attributes);

  /* creation of Queue04 */
  Queue04Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue04_attributes);

  /* creation of Queue05 */
  Queue05Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue05_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartDefaultTask, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* creation of Task03 */
  Task03Handle = osThreadNew(StartTask03, NULL, &Task03_attributes);

  /* creation of Task04 */
  Task04Handle = osThreadNew(StartTask04, NULL, &Task04_attributes);

  /* creation of Task05 */
  Task05Handle = osThreadNew(StartTask05, NULL, &Task05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CDC_ReceiveCallback(uint8_t *Buf, uint16_t Len)
{
	int i;
	int stopPoint;
	uint8_t buf_cb[30] = "callback: ";
	strcat((char *) buf_cb, (char *) Buf);
	BaseType_t xHigherPriorityTaskWoken;

	for(i = 0; i < 30; i++)
	{
		if(buf_cb[i] == '\0')
		{
			stopPoint = i;
			break;
		}
	}

	xHigherPriorityTaskWoken = pdFALSE;
	for(i = 10; i < stopPoint; i++)
	{
		xQueueSendToBackFromISR(Queue01Handle, &buf_cb[i], &xHigherPriorityTaskWoken);
	}
	//CDC_Transmit_FS(buf_cb, strlen(buf_cb));

}

void CheckCommand(char* DataToReceive)
{

	  char* holder;

	  //CDC_Transmit_FS(DataToReceive, strlen(DataToReceive));
	  if(DataToReceive[1] == 'H' && DataToReceive[2] == 'e' && DataToReceive[3] == 'l' && DataToReceive[4] == 'p')
	  			  {
	  		  		  holder = "\r\nEnter the wanted command with the required data between the ''\r\nDump: dumps the names and states of the active tasks\n\rAdd ['name']['priority']: Add a task\n\rDelete ['name']: Delete a task\n\rChange ['name']['priority']: Change the priority of a task.\n\r";
	  		  		  CDC_Transmit_FS(holder, strlen(holder));
	  		  		  return;
	  			  }
	  		  else if(DataToReceive[1] == 'D' && DataToReceive[2] == 'u' && DataToReceive[3] == 'm' && DataToReceive[4] == 'p')
	  		  	  {
	  			  	  holder = "\r\nTesting1\r\n";
	  			  	  //CDC_Transmit_FS(holder, strlen(holder));
	  			  	  DumpTasks();
	  			  	  return;
	  		  	  }
	  		  else if(DataToReceive[1] == 'A' && DataToReceive[2] == 'd' && DataToReceive[3] == 'd')
	  		  	  {
					  char name[4];
					  int priority;
					  int i = 6;
					  int x = 0;

					  if(DataToReceive[12] == '1' || DataToReceive[12] == '2' || DataToReceive[12] == '3' || DataToReceive[12] == '4' || DataToReceive[12] == '5' || DataToReceive[12] == '6' || DataToReceive[12] == '7' || DataToReceive[12] == '8' || DataToReceive[12] == '9')
						  priority = atoi(&DataToReceive[15]);
					  else
					  {
						  holder = "\r\nIncorrect Priority Input\r\n";
						  CDC_Transmit_FS(holder, strlen(holder));
						  return;
					  }
	  			  	  for(x = 0; x < 4; x++)
	  			  	  {
	  			  		  name[x] = DataToReceive[i];
	  			  		  i++;
	  			  	  }
	  			  	  AddTask((char*)name, priority);

	  			  	  return;
	  		  	  }
	  		  else if(DataToReceive[1] == 'D' && DataToReceive[2] == 'e' && DataToReceive[3] == 'l' && DataToReceive[4] == 'e' && DataToReceive[5] == 't' && DataToReceive[6] == 'e')
	  		  	  {
  			  	  	  char name[4];
  			  	  	  int i = 9;
  			  	  	  int x = 0;

	  			  	  for(x = 0; x < 4; x++)
	  			  	  {
	  			  		  name[x] = DataToReceive[i];
	  			  		  i++;
	  			  	  }
	  			  	  if(DeleteTask(name) == true)
	  			  	  {
						  holder = "\r\nTask Deleted\r\n";
						  CDC_Transmit_FS(holder, strlen(holder));
	  			  	  }
	  			  	  return;
	  		  	  }
	  		  else if(DataToReceive[1] == 'C' && DataToReceive[2] == 'h' && DataToReceive[3] == 'a' && DataToReceive[4] == 'n' && DataToReceive[5]== 'g' && DataToReceive[6] == 'e')
	  		  	  {
	  			  	  //CDC_Transmit_FS(DataToReceive, strlen(DataToReceive));
	  			  	  char name[4];
	  			  	  int priority;
	  			  	  int i = 9;
	  			  	  int x = 0;

	  			  	  if(DataToReceive[15] == '1' || DataToReceive[15] == '2' || DataToReceive[15] == '3' || DataToReceive[15] == '4' || DataToReceive[15] == '5' || DataToReceive[15] == '6' || DataToReceive[15] == '7' || DataToReceive[15] == '8' || DataToReceive[15] == '9')
	  			  		  priority = atoi(&DataToReceive[15]);
	  			  	  else
	  			  	  {
		  			  	  holder = "\r\nIncorrect Priority Input\r\n";
		  			  	  CDC_Transmit_FS(holder, strlen(holder));
		  			  	  return;
	  			  	  }
	  			  	  for(x = 0; x < 4; x++)
	  			  	  {
	  			  		  name[x] = DataToReceive[i];
	  			  		  i++;
	  			  	  }
	  			  	  if(ChangeTask(name, priority) == true)
	  			  	  {
						  holder = "\r\nNew Priority Set\r\n";
						  CDC_Transmit_FS(holder, strlen(holder));
	  			  	  }
	  			  	  return;
	  		  	  }
	  		  else
	  			CDC_Transmit_FS(DataToReceive, strlen(DataToReceive));
}
void DumpTasks()
{
	char* holder = "Name          State  Priority  Stack   Num\r\n";
	char buffer[256];
	vTaskList(buffer);
	CDC_Transmit_FS(holder, strlen(holder));
	while(CDC_Transmit_FS(buffer, strlen(buffer)) ==1)
	{}
}
void AddTask(char* taskName, int priority)
{
	char* holder;
	char name[5];
	int i = 0;
	for (i = 0; i < 4; i++)
	{
		name[i] = taskName[i];
	}
	name[4] = NULL;
	TaskHandle_t* task;
	UBaseType_t taskPriority = priority;
	if(xHandle01 == NULL)
		task = &xHandle01;
	else if(xHandle02 == NULL)
		task = &xHandle02;
	else if(xHandle03 == NULL)
		task = &xHandle03;
	else if(xHandle04 == NULL)
		task = &xHandle04;
	else if(xHandle05 == NULL)
		task = &xHandle05;
	else
	{
		holder = "\r\nMax number of tasks already created, or other error\r\n";
		CDC_Transmit_FS(holder, strlen(holder));
	}
	BaseType_t xReturned = xTaskCreate(StartTask02, name, 128*4, NULL, taskPriority, task);
	if (xReturned == pdPASS)
	{
		holder = taskName;
		char* success = " Has been created\r\n";
		strcat(holder, success);
		CDC_Transmit_FS(holder, strlen(holder));
	}
}
bool DeleteTask(char* taskName)
{
	char* holder;
	char name[5];
	int i = 0;
	for (i = 0; i < 4; i++)
	{
		name[i] = taskName[i];
	}
	name[4] = NULL;
	TaskHandle_t task;
	task = xTaskGetHandle(name);
	if (task == NULL)
	{
	  	  holder = "\r\nInvaild Task Name\r\n";
	  	  CDC_Transmit_FS(holder, strlen(holder));
	  	  return false;
	}
	vTaskDelete(task);
	return true;
}
bool ChangeTask(char* taskName, int priority)
{
	char* holder;
	UBaseType_t newPriority = priority;
	TaskHandle_t task;
	task = xTaskGetHandle(taskName);
	if (task == NULL)
	{
	  	  holder = "\r\nInvaild Task Name\r\n";
	  	  CDC_Transmit_FS(holder, strlen(holder));
	  	  return false;
	}
	vTaskPrioritySet(task, newPriority);
	return true;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t Buf;
  char message[20] = "$";
  for(;;)
  {
	  if(uxQueueMessagesWaiting(Queue01Handle) != 0)
	  {
		  while(uxQueueMessagesWaiting(Queue01Handle) != 0)
		  {
			  xQueueReceive(Queue01Handle, &Buf, 0);
		  	  strncat(message,(char*) &Buf, 1);
		  }
		  CheckCommand(message);
		  memset(message, 0, sizeof(message));
		  message[0] = '$';
	  }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartTask05 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
