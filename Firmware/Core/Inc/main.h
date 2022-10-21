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
#define ADC_MOT1_CUR_Pin GPIO_PIN_0
#define ADC_MOT1_CUR_GPIO_Port GPIOC
#define ADC_MOT2_CUR_Pin GPIO_PIN_1
#define ADC_MOT2_CUR_GPIO_Port GPIOC
#define ADC_VREF_Pin GPIO_PIN_2
#define ADC_VREF_GPIO_Port GPIOC
#define TIM_ENC2_B_Pin GPIO_PIN_1
#define TIM_ENC2_B_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_2
#define ADC_TEMP_GPIO_Port GPIOA
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
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define TIM_MOT1_B_Pin GPIO_PIN_14
#define TIM_MOT1_B_GPIO_Port GPIOB
#define TIM_MOT2_B_Pin GPIO_PIN_15
#define TIM_MOT2_B_GPIO_Port GPIOB
#define O_MOT_ENABLE_Pin GPIO_PIN_6
#define O_MOT_ENABLE_GPIO_Port GPIOC
#define TIM_MOT1_A_Pin GPIO_PIN_7
#define TIM_MOT1_A_GPIO_Port GPIOC
#define TIM_MOT2_A_Pin GPIO_PIN_8
#define TIM_MOT2_A_GPIO_Port GPIOC
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
