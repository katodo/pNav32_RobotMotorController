/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_KEY1_Pin GPIO_PIN_3
#define I_KEY1_GPIO_Port GPIOE
#define I_KEY0_Pin GPIO_PIN_4
#define I_KEY0_GPIO_Port GPIOE
#define PC0_M1_CURRENT_Pin GPIO_PIN_0
#define PC0_M1_CURRENT_GPIO_Port GPIOC
#define PC1_M2_CURRENT_Pin GPIO_PIN_1
#define PC1_M2_CURRENT_GPIO_Port GPIOC
#define ADC_VREF_Pin GPIO_PIN_2
#define ADC_VREF_GPIO_Port GPIOC
#define PA0_MAIN_SUPPLY_Pin GPIO_PIN_0
#define PA0_MAIN_SUPPLY_GPIO_Port GPIOA
#define TIM_ENC2_B_Pin GPIO_PIN_1
#define TIM_ENC2_B_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_2
#define ADC_TEMP_GPIO_Port GPIOA
#define PA3_M1_V_Pin GPIO_PIN_3
#define PA3_M1_V_GPIO_Port GPIOA
#define PA4_M2_V_Pin GPIO_PIN_4
#define PA4_M2_V_GPIO_Port GPIOA
#define TIM_ENC2_A_Pin GPIO_PIN_5
#define TIM_ENC2_A_GPIO_Port GPIOA
#define O_LED_D2_Pin GPIO_PIN_6
#define O_LED_D2_GPIO_Port GPIOA
#define O_LED_D3_Pin GPIO_PIN_7
#define O_LED_D3_GPIO_Port GPIOA
#define TIM_FAN_PWM_Pin GPIO_PIN_0
#define TIM_FAN_PWM_GPIO_Port GPIOB
#define TIM_FAN_INDEX_Pin GPIO_PIN_1
#define TIM_FAN_INDEX_GPIO_Port GPIOB
#define PE8_MOT1_AUX_Pin GPIO_PIN_8
#define PE8_MOT1_AUX_GPIO_Port GPIOE
#define PE9_MOT2_AUX_Pin GPIO_PIN_9
#define PE9_MOT2_AUX_GPIO_Port GPIOE
#define PE12_TEMP1_Pin GPIO_PIN_12
#define PE12_TEMP1_GPIO_Port GPIOE
#define PE13_TEMP2_Pin GPIO_PIN_13
#define PE13_TEMP2_GPIO_Port GPIOE
#define PE14_TEMP3_Pin GPIO_PIN_14
#define PE14_TEMP3_GPIO_Port GPIOE
#define PE15_TEMP4_Pin GPIO_PIN_15
#define PE15_TEMP4_GPIO_Port GPIOE
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define PB12_MOT1_EN_Pin GPIO_PIN_12
#define PB12_MOT1_EN_GPIO_Port GPIOB
#define PB14_TIM8_MOT1_N_Pin GPIO_PIN_14
#define PB14_TIM8_MOT1_N_GPIO_Port GPIOB
#define PB15_TIM8_MOT2_N_Pin GPIO_PIN_15
#define PB15_TIM8_MOT2_N_GPIO_Port GPIOB
#define PD11_EXTI11_E1_IDX_Pin GPIO_PIN_11
#define PD11_EXTI11_E1_IDX_GPIO_Port GPIOD
#define PD12_EXTI12_E2_IDX_Pin GPIO_PIN_12
#define PD12_EXTI12_E2_IDX_GPIO_Port GPIOD
#define PC7_TIM8_MOT1_P_Pin GPIO_PIN_7
#define PC7_TIM8_MOT1_P_GPIO_Port GPIOC
#define PC8_TIM8_MOT2_P_Pin GPIO_PIN_8
#define PC8_TIM8_MOT2_P_GPIO_Port GPIOC
#define O_PWR_SBC_EN_Pin GPIO_PIN_9
#define O_PWR_SBC_EN_GPIO_Port GPIOC
#define TIM_ENC1_A_Pin GPIO_PIN_8
#define TIM_ENC1_A_GPIO_Port GPIOA
#define TIM_ENC1_B_Pin GPIO_PIN_9
#define TIM_ENC1_B_GPIO_Port GPIOA
#define O_PWR_MOT_EN_Pin GPIO_PIN_10
#define O_PWR_MOT_EN_GPIO_Port GPIOA
#define O_SPI_CS2_Pin GPIO_PIN_10
#define O_SPI_CS2_GPIO_Port GPIOC
#define O_SPI_CS1_Pin GPIO_PIN_11
#define O_SPI_CS1_GPIO_Port GPIOC
#define SER2_TX_Pin GPIO_PIN_12
#define SER2_TX_GPIO_Port GPIOC
#define SER2_RX_Pin GPIO_PIN_2
#define SER2_RX_GPIO_Port GPIOD
#define PD3_MOT2_EN_Pin GPIO_PIN_3
#define PD3_MOT2_EN_GPIO_Port GPIOD
#define O_PWR_REG_EN_Pin GPIO_PIN_8
#define O_PWR_REG_EN_GPIO_Port GPIOB
#define TIM_AUX2_PWM_Pin GPIO_PIN_9
#define TIM_AUX2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
