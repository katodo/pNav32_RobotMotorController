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


/*
  USEFULL LINK:
 	https://micro.ros.org/docs/tutorials/core/first_application_linux

 BASE COMMAND:
 cd ~/STM32CubeIDE/workspace_1.17.0/pNav32_RobotMotorController/agent_ws
 ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0

*/

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "globals.h"

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
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/color_rgba.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <control_msgs/msg/joint_jog.h>

// Aggiunta per il subscriber twist:
#include <geometry_msgs/msg/twist.h>

// UART, motors, encoder
#include "usart.h"
#include "motors.h"
#include "encoder.h"

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

/* === Publisher e messaggi standard === */
//rcl_publisher_t publisher_int32;
//std_msgs__msg__Int32 msgInt32;

//rcl_publisher_t publisher_int64;
//std_msgs__msg__Int64 msgInt64;

rcl_publisher_t publisher_temp;
sensor_msgs__msg__Temperature msgTemperature;

//rcl_publisher_t publisher_color;
//std_msgs__msg__ColorRGBA msgColorRGBA;

rcl_publisher_t publisher_batt;
sensor_msgs__msg__BatteryState msgBattery;
rosidl_runtime_c__float__Sequence battVoltage;
const size_t NUMBEROFFCELL = 6;

/* === Publisher e messaggi per Encoder 1 e 2 === */
rcl_publisher_t publisher_Enc1_pos1;
std_msgs__msg__Int32   msg_Enc1_pos1;

rcl_publisher_t publisher_Enc1_vel1;
std_msgs__msg__Float64 msg_Enc1_vel1; // Corretto in Float64


rcl_publisher_t publisher_Enc2_pos2;
std_msgs__msg__Int32   msg_Enc2_pos2;

rcl_publisher_t publisher_Enc2_vel2;
std_msgs__msg__Float64 msg_Enc2_vel2; // Corretto in Float64

/* === Publisher e messaggi per impulsi encoder === */
rcl_publisher_t publisher_Enc1_impulse_time;
std_msgs__msg__UInt32 msg_Enc1_impulse_time;

rcl_publisher_t publisher_Enc2_impulse_time;
std_msgs__msg__UInt32 msg_Enc2_impulse_time;


/* === Variabili encoder reali (aggiornate altrove) === */
volatile int32_t pos1;
volatile float vel1; // Modificato da int32_t a float
volatile int32_t pos2;
volatile float vel2; // Modificato da int32_t a float

/* === Subscriber al topic cmd_vel (Twist) === */
rcl_subscription_t sub_cmdVel;
geometry_msgs__msg__Twist msg_cmdVel;  // dati in arrivo

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
/* Definitions for myTimer_1m */
osTimerId_t myTimer_1mHandle;
const osTimerAttr_t myTimer_1m_attributes = {
  .name = "myTimer_1m"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
 * @brief Semplice funzione che calcola il fattore K di un robot differenziale,
 *        partendo dalla distanza tra le ruote (track_width_m).
 * @param track_width_m  Distanza (m) tra ruota sinistra e ruota destra
 * @return  K = track_width_m / 2
 */
static float ComputeDifferentialK(float track_width_m)
{
    return track_width_m * 0.5f;
}

/**
 * @brief Callback di ricezione del messaggio cmd_vel (geometry_msgs::msg::Twist)
 * @param msgin  messaggio generico da castare a Twist
 */
static void cmdVelCallback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // Lettura velocità lineare e angolare
    float v_lin = msg->linear.x;   // [m/s]
    float v_ang = msg->angular.z;  // [rad/s]

    // Calcolo K dalla geometria del robot (ad es. 0.40 m tra i cingoli)
    float track_width = 0.40f;
    float K = ComputeDifferentialK(track_width);

    // Velocità motori:
    float speedM1 = v_lin - (v_ang * K);
    float speedM2 = v_lin + (v_ang * K);

    // Abilita entrambi i motori (true, true), AUX disabilitato (false, false)
    MotorControl_SetMotors(speedM1, speedM2, true, true, false, false);
}

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
void CallbackTimer_1m(void *argument);

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

  /* Create the timer(s) */
  /* creation of myTimer_1m */
  myTimer_1mHandle = osTimerNew(CallbackTimer_1m, osTimerPeriodic, NULL, &myTimer_1m_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  // start timers, add new ones, ...
  // Avvio del timer a 1 ms (se configTICK_RATE_HZ = 1000)
  osStatus_t status = osTimerStart(myTimer_1mHandle, 1);
  if (status != osOK)
  {
    // Error handling timer
  }
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
    HAL_GPIO_WritePin(O_LED_D2_GPIO_Port, O_LED_D2_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(O_LED_D2_GPIO_Port, O_LED_D2_Pin, GPIO_PIN_RESET);
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

  /* ===========================================================================
   * 1) Configurazione micro-ROS su UART
   * ==========================================================================*/
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate      = microros_allocate;
  freeRTOS_allocator.deallocate    = microros_deallocate;
  freeRTOS_allocator.reallocate    = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  /* ===========================================================================
   * 2) Inizializzazione rclc e creazione nodo
   * ==========================================================================*/
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "pnav32", "", &support);

  /* ===========================================================================
   * 3) Creazione Publisher
   * ==========================================================================*/
  // Esempi: pInt32, pInt64, pBatt, pTemp, pColorRGBA, e i publisher encoder
//  rclc_publisher_init_default(&publisher_int32, &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//    "pInt32");
//
//  rclc_publisher_init_default(&publisher_int64, &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
//    "pInt64");
//
//  rclc_publisher_init_default(&publisher_color, &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
//    "pColorRGBA");

  rclc_publisher_init_default(&publisher_batt, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "pBatt");

  rclc_publisher_init_default(&publisher_temp, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    "pTemp");



  // Encoder 1
  rclc_publisher_init_default(&publisher_Enc1_pos1, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Enc1Pos1");

  rclc_publisher_init_default(&publisher_Enc1_vel1, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), // Modificato in Float64
      "Enc1Vel1");

  // Encoder 2
  rclc_publisher_init_default(&publisher_Enc2_pos2, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Enc2Pos2");

  rclc_publisher_init_default(&publisher_Enc2_vel2, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), // Modificato in Float64
      "Enc2Vel2");

  rclc_publisher_init_default(&publisher_Enc1_impulse_time, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "Enc1ImpulseTime");

  rclc_publisher_init_default(&publisher_Enc2_impulse_time, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "Enc2ImpulseTime");

  /* ===========================================================================
   * 4) Creazione Subscriber a cmd_vel
   * ==========================================================================*/
  rclc_subscription_init_default(
    &sub_cmdVel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  /* ===========================================================================
   * 5) Creazione Executor per gestire la callback su cmd_vel
   * ==========================================================================*/
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  // '1' = una subscription/callback, se ne hai di più aumenta

  // Aggiunge la subscription e la callback
  rclc_executor_add_subscription(
    &executor,
    &sub_cmdVel,
    &msg_cmdVel,
    &cmdVelCallback,
    ON_NEW_DATA
  );

  /* ===========================================================================
   * 6) Assegnazione valori iniziali per i messaggi standard
   * ==========================================================================*/
//  msgInt32.data = 1;
//  msgInt64.data = 10;
//
//  msgColorRGBA.r = 127;
//  msgColorRGBA.g = 127;
//  msgColorRGBA.b = 100;
//  msgColorRGBA.a = 200;

  msgBattery.power_supply_status     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  msgBattery.power_supply_health     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  msgBattery.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
  msgBattery.charge  = 2;
  msgBattery.current = 10;
  battVoltage.size   = NUMBEROFFCELL;
  msgBattery.cell_voltage.capacity = NUMBEROFFCELL;
  msgBattery.cell_voltage.size     = NUMBEROFFCELL;
  msgBattery.cell_voltage.data     = (float*) malloc(NUMBEROFFCELL * sizeof(float));
  msgBattery.cell_voltage.data[0]  = 0.0f;
  msgBattery.cell_voltage.data[1]  = 1.0f;
  msgBattery.cell_voltage.data[2]  = 2.0f;
  msgBattery.cell_voltage.data[3]  = 3.0f;
  msgBattery.cell_voltage.data[4]  = 4.0f;
  msgBattery.cell_voltage.data[5]  = 5.0f;
  // e così via se servono

  msgTemperature.temperature = 25.0f;

  /* ===========================================================================
   * 7) Loop infinito: pubblicazione e spin dell'executor
   * ==========================================================================*/
  for(;;)
  {
    // Esempio di aggiornamento messaggi
//    msgInt32.data++;
//    msgInt64.data++;
//    msgColorRGBA.g++;
    msgBattery.voltage += 0.001f;
    msgTemperature.temperature += 0.001f;

    // Aggiorna messaggi encoder
    msg_Enc1_pos1.data  = g_Encoder1.position;
    msg_Enc1_vel1.data  = g_Encoder1.velocity; // Usa il valore float

    msg_Enc2_pos2.data  = g_Encoder2.position;
    msg_Enc2_vel2.data  = g_Encoder2.velocity; // Usa il valore float

    msg_Enc1_impulse_time.data = g_Encoder1.impulse_time;
    msg_Enc2_impulse_time.data = g_Encoder2.impulse_time;

    // LED D3 ON
    HAL_GPIO_WritePin(O_LED_D3_GPIO_Port, O_LED_D3_Pin, GPIO_PIN_RESET);

    // Pubblicazioni di esempio
    rcl_ret_t ret = RCL_RET_OK;
//    ret += rcl_publish(&publisher_int32, &msgInt32, NULL);
//    ret += rcl_publish(&publisher_int64, &msgInt64, NULL);
//    ret += rcl_publish(&publisher_color, &msgColorRGBA, NULL);
    ret += rcl_publish(&publisher_batt, &msgBattery, NULL);
    ret += rcl_publish(&publisher_temp, &msgTemperature, NULL);

    // Pubblica messaggi encoder
    ret += rcl_publish(&publisher_Enc1_pos1,  &msg_Enc1_pos1,  NULL);
    ret += rcl_publish(&publisher_Enc1_vel1,  &msg_Enc1_vel1,  NULL);
    //ret += rcl_publish(&publisher_Enc1VelTPS, &msg_Enc1VelTPS, NULL);

    ret += rcl_publish(&publisher_Enc2_pos2,  &msg_Enc2_pos2,  NULL);
    ret += rcl_publish(&publisher_Enc2_vel2,  &msg_Enc2_vel2,  NULL);
    //ret += rcl_publish(&publisher_Enc2VelTPS, &msg_Enc2VelTPS, NULL);

    ret += rcl_publish(&publisher_Enc1_impulse_time, &msg_Enc1_impulse_time, NULL);
    if (ret != RCL_RET_OK) {
      printf("Error publishing Enc1ImpulseTime, ret = %ld\n", ret);
    }

    ret += rcl_publish(&publisher_Enc2_impulse_time, &msg_Enc2_impulse_time, NULL);
    if (ret != RCL_RET_OK) {
      printf("Error publishing Enc2ImpulseTime, ret = %ld\n", ret);
    }


    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }

    // Esegui spin dell'executor per gestire la callback del subscriber
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // LED D3 OFF
    HAL_GPIO_WritePin(O_LED_D3_GPIO_Port, O_LED_D3_Pin, GPIO_PIN_SET);

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
    if(dmaTransferComplete == 1)
    {
      dmaTransferComplete = 0;
      osDelay(1);
    }
  }
  /* USER CODE END StartTaskAnalog */
}

/* CallbackTimer_1m function */
void CallbackTimer_1m(void *argument)
{
  /* USER CODE BEGIN CallbackTimer_1m */
  // Codice eseguito ogni 1 ms
  //  ENC_Update();  // Aggiorna gli encoder ogni millisecondo
  /* USER CODE END CallbackTimer_1m */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

