/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/color_rgba.h>
//#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

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
/* Definitions for rosTaskLed */
osThreadId_t rosTaskLedHandle;
uint32_t rosTaskLedBuffer[ 512 ];
osStaticThreadDef_t rosTaskLedControlBlock;
const osThreadAttr_t rosTaskLed_attributes = {
  .name = "rosTaskLed",
  .cb_mem = &rosTaskLedControlBlock,
  .cb_size = sizeof(rosTaskLedControlBlock),
  .stack_mem = &rosTaskLedBuffer[0],
  .stack_size = sizeof(rosTaskLedBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for rosTaskCom */
osThreadId_t rosTaskComHandle;
uint32_t rosTaskComBuffer[ 12000 ];
osStaticThreadDef_t rosTaskComControlBlock;
const osThreadAttr_t rosTaskCom_attributes = {
  .name = "rosTaskCom",
  .cb_mem = &rosTaskComControlBlock,
  .cb_size = sizeof(rosTaskComControlBlock),
  .stack_mem = &rosTaskComBuffer[0],
  .stack_size = sizeof(rosTaskComBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void StartTaskLed(void *argument);
void StartTaskCom(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of rosTaskLed */
  rosTaskLedHandle = osThreadNew(StartTaskLed, NULL, &rosTaskLed_attributes);

  /* creation of rosTaskCom */
  rosTaskComHandle = osThreadNew(StartTaskCom, NULL, &rosTaskCom_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskLed */
/**
  * @brief  Function implementing the rosTaskLed thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskLed */
void StartTaskLed(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartTaskLed */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(O_LED_D2_GPIO_Port, O_LED_D2_Pin, 1);
		osDelay(100);
		HAL_GPIO_WritePin(O_LED_D2_GPIO_Port, O_LED_D2_Pin, 0);
		osDelay(200);
  }
  /* USER CODE END StartTaskLed */
}

/* USER CODE BEGIN Header_StartTaskCom */
/**
* @brief Function implementing the rosTaskCom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCom */
void StartTaskCom(void *argument)
{
  /* USER CODE BEGIN StartTaskCom */
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator))
	{	printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app
	rcl_publisher_t publisher0;
	std_msgs__msg__Int32 msg0;

	rcl_publisher_t publisher1;
	std_msgs__msg__Int64 msg1;

	rcl_publisher_t publisher2;
	sensor_msgs__msg__BatteryState msg2;

	rcl_publisher_t publisher3;
	sensor_msgs__msg__Temperature msg3;

	rcl_publisher_t publisher4;
	std_msgs__msg__Float64MultiArray msg4;

	rcl_publisher_t publisher5;
	std_msgs__msg__ColorRGBA msg5;


	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;

	allocator = rcl_get_default_allocator();

	//create init_options
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rclc_node_init_default(&node, "pnav32", "", &support);

	// create publisher
	rclc_publisher_init_default( &publisher0, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pInt32");
	rclc_publisher_init_default( &publisher1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "pInt64");
	rclc_publisher_init_default( &publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "pBatt");
	rclc_publisher_init_default( &publisher3, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "pTemp");
	rclc_publisher_init_default( &publisher4, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "pFloat64M");
	rclc_publisher_init_default( &publisher5, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "pColorRGBA");

	// preinit with random test value
	msg0.data = 1;
	msg1.data = 10;
	msg2.voltage = 24.9;
	msg2.charge = 35;
	msg2.current = 10;
	msg3.temperature = 45;
	msg5.r= 127;
	msg5.g= 127;
	msg5.b= 100;
	msg5.a= 200;

  for(;;)
  {

	// Random data update before publish
	msg0.data++;
	msg1.data++;
	msg2.voltage += 0.001;
	msg3.temperature += 0.001;
	msg5.g++;


	// Led ON
	HAL_GPIO_WritePin(O_LED_D3_GPIO_Port, O_LED_D3_Pin, 0);

	rcl_ret_t ret;
	ret = rcl_publish(&publisher0, &msg0, NULL);
	ret += rcl_publish(&publisher1, &msg1, NULL);
//	ret += rcl_publish(&publisher2, &msg2, NULL);
//	ret += rcl_publish(&publisher3, &msg3, NULL);
//	ret += rcl_publish(&publisher4, &msg4, NULL);
	ret += rcl_publish(&publisher4, &msg4, NULL);

	if (ret != RCL_RET_OK)
	{
	  printf("Error publishing (line %d)\n", __LINE__);
	}

	// Led turn Off
	HAL_GPIO_WritePin(O_LED_D3_GPIO_Port, O_LED_D3_Pin, 1);
	osDelay(100);
  }



  /* USER CODE END StartTaskCom */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

