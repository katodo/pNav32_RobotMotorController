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
#include <control_msgs/msg/joint_jog.h>

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

rcl_publisher_t publisher_int32;
std_msgs__msg__Int32 msgInt32;

rcl_publisher_t publisher_int64;
std_msgs__msg__Int64 msgInt64;

rcl_publisher_t publisher_temp;
sensor_msgs__msg__Temperature msgTemperature;

rcl_publisher_t publisher_color;
std_msgs__msg__ColorRGBA msgColorRGBA;

rcl_publisher_t publisher_batt;
sensor_msgs__msg__BatteryState msgBattery;
rosidl_runtime_c__float__Sequence battVoltage;
const uint NUMBEROFFCELL = 6;

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
uint32_t rosTaskComBuffer[ 5000 ];
osStaticThreadDef_t rosTaskComControlBlock;
const osThreadAttr_t rosTaskCom_attributes = {
  .name = "rosTaskCom",
  .cb_mem = &rosTaskComControlBlock,
  .cb_size = sizeof(rosTaskComControlBlock),
  .stack_mem = &rosTaskComBuffer[0],
  .stack_size = sizeof(rosTaskComBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rosTaskAnalog */
osThreadId_t rosTaskAnalogHandle;
uint32_t rosTaskAnalogBuffer[ 256 ];
osStaticThreadDef_t rosTaskAnalogControlBlock;
const osThreadAttr_t rosTaskAnalog_attributes = {
  .name = "rosTaskAnalog",
  .cb_mem = &rosTaskAnalogControlBlock,
  .cb_size = sizeof(rosTaskAnalogControlBlock),
  .stack_mem = &rosTaskAnalogBuffer[0],
  .stack_size = sizeof(rosTaskAnalogBuffer),
  .priority = (osPriority_t) osPriorityLow,
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
void StartTaskAnalog(void *argument);

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

  /* creation of rosTaskAnalog */
  rosTaskAnalogHandle = osThreadNew(StartTaskAnalog, NULL, &rosTaskAnalog_attributes);

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

	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;

	allocator = rcl_get_default_allocator();

	//create init_options
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rclc_node_init_default(&node, "pnav32", "", &support);

	// create publisher
	rclc_publisher_init_default( &publisher_int32, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pInt32");
	rclc_publisher_init_default( &publisher_int64, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "pInt64");
	rclc_publisher_init_default( &publisher_color, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "pColorRGBA");
	rclc_publisher_init_default( &publisher_batt, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "pBatt");
	rclc_publisher_init_default( &publisher_temp, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "pTemp");

	// preinit with random test value
	msgInt32.data = 1;

	msgInt64.data = 10;

	msgColorRGBA.r= 127;
	msgColorRGBA.g= 127;
	msgColorRGBA.b= 100;
	msgColorRGBA.a= 200;

	msgBattery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
	msgBattery.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
	msgBattery.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
	msgBattery.charge = 2;
	msgBattery.current = 10;
    battVoltage.size = NUMBEROFFCELL;
    msgBattery.cell_voltage.capacity = NUMBEROFFCELL;
    msgBattery.cell_voltage.size = NUMBEROFFCELL;
    msgBattery.cell_voltage.data = ( float*) malloc( msgBattery.cell_voltage.capacity * sizeof(float));
    msgBattery.cell_voltage.data[0] = 0;
    msgBattery.cell_voltage.data[1] = 1;
    msgBattery.cell_voltage.data[3] = 3;

	msgTemperature.temperature = 25;

  for(;;)
  {
	// TEST: Random data update before publish
	msgInt32.data++;
	msgInt64.data++;
	msgBattery.voltage += 0.001;
	msgTemperature.temperature += 0.001;
	msgColorRGBA.g++;
	// TEST END

	// Led ON
	HAL_GPIO_WritePin(O_LED_D3_GPIO_Port, O_LED_D3_Pin, 0);

	rcl_ret_t ret;
	ret = rcl_publish(&publisher_int32, &msgInt32, NULL);
	ret += rcl_publish(&publisher_int64, &msgInt64, NULL);
	ret += rcl_publish(&publisher_color, &msgColorRGBA, NULL);
	ret += rcl_publish(&publisher_batt, &msgBattery, NULL);
	ret += rcl_publish(&publisher_temp, &msgTemperature, NULL);

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

/* USER CODE BEGIN Header_StartTaskAnalog */
/**
* @brief Function implementing the rosTaskAnalog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskAnalog */
void StartTaskAnalog(void *argument)
{
  /* USER CODE BEGIN StartTaskAnalog */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskAnalog */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

