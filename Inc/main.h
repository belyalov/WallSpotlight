/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOA
#define PWR_BATT_Pin GPIO_PIN_2
#define PWR_BATT_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_3
#define LIGHT_GPIO_Port GPIOA
#define PWR_LIGHT_Pin GPIO_PIN_4
#define PWR_LIGHT_GPIO_Port GPIOA
#define BATT_CHRG_Pin GPIO_PIN_5
#define BATT_CHRG_GPIO_Port GPIOA
#define BATT_CHRG_EXTI_IRQn EXTI9_5_IRQn
#define BATT_STDBY_Pin GPIO_PIN_6
#define BATT_STDBY_GPIO_Port GPIOA
#define BATT_STDBY_EXTI_IRQn EXTI9_5_IRQn
#define SW_RADAR_Pin GPIO_PIN_7
#define SW_RADAR_GPIO_Port GPIOA
#define SW_LIGHT_Pin GPIO_PIN_0
#define SW_LIGHT_GPIO_Port GPIOB
#define PWR_SENSOR_Pin GPIO_PIN_2
#define PWR_SENSOR_GPIO_Port GPIOB
#define PWR_RADAR_Pin GPIO_PIN_10
#define PWR_RADAR_GPIO_Port GPIOA
#define RADAR_Pin GPIO_PIN_11
#define RADAR_GPIO_Port GPIOA
#define RADAR_EXTI_IRQn EXTI15_10_IRQn
#define PWR_RADIO_Pin GPIO_PIN_12
#define PWR_RADIO_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOB
#define LED_W1_Pin GPIO_PIN_5
#define LED_W1_GPIO_Port GPIOB
#define LED_W2_Pin GPIO_PIN_6
#define LED_W2_GPIO_Port GPIOB
#define LED_W3_Pin GPIO_PIN_7
#define LED_W3_GPIO_Port GPIOB
#define LED_W4_Pin GPIO_PIN_8
#define LED_W4_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_9
#define BTN_GPIO_Port GPIOB
#define BTN_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
