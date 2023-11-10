/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define EN_0_Pin GPIO_PIN_0
#define EN_0_GPIO_Port GPIOC
#define EN_1_Pin GPIO_PIN_1
#define EN_1_GPIO_Port GPIOC
#define EN_2_Pin GPIO_PIN_2
#define EN_2_GPIO_Port GPIOC
#define EN_3_Pin GPIO_PIN_3
#define EN_3_GPIO_Port GPIOC
#define ADC_DRDYB_Pin GPIO_PIN_0
#define ADC_DRDYB_GPIO_Port GPIOA
#define ENC_BTN_Pin GPIO_PIN_1
#define ENC_BTN_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_2
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOA
#define ON_OFF_BTN_Pin GPIO_PIN_4
#define ON_OFF_BTN_GPIO_Port GPIOA
#define ADC_CSB_Pin GPIO_PIN_4
#define ADC_CSB_GPIO_Port GPIOC
#define EN_4_Pin GPIO_PIN_5
#define EN_4_GPIO_Port GPIOC
#define I2C_MUX_SEL0_Pin GPIO_PIN_0
#define I2C_MUX_SEL0_GPIO_Port GPIOB
#define I2C_MUX_SEL1_Pin GPIO_PIN_1
#define I2C_MUX_SEL1_GPIO_Port GPIOB
#define I2C_MUX_SEL2_Pin GPIO_PIN_2
#define I2C_MUX_SEL2_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_11
#define LCD_EN_GPIO_Port GPIOB
#define LCD_DB4_Pin GPIO_PIN_12
#define LCD_DB4_GPIO_Port GPIOB
#define LCD_DB5_Pin GPIO_PIN_13
#define LCD_DB5_GPIO_Port GPIOB
#define LCD_DB6_Pin GPIO_PIN_14
#define LCD_DB6_GPIO_Port GPIOB
#define LCD_DB7_Pin GPIO_PIN_15
#define LCD_DB7_GPIO_Port GPIOB
#define EN_5_Pin GPIO_PIN_6
#define EN_5_GPIO_Port GPIOC
#define EN_6_Pin GPIO_PIN_7
#define EN_6_GPIO_Port GPIOC
#define EN_7_Pin GPIO_PIN_8
#define EN_7_GPIO_Port GPIOC
#define TRIG_Pin GPIO_PIN_8
#define TRIG_GPIO_Port GPIOA
#define ON_OFF_LED_Pin GPIO_PIN_10
#define ON_OFF_LED_GPIO_Port GPIOC
#define WARN_LED_Pin GPIO_PIN_11
#define WARN_LED_GPIO_Port GPIOC
#define CS_MUX_SEL_0_Pin GPIO_PIN_3
#define CS_MUX_SEL_0_GPIO_Port GPIOB
#define CS_MUX_SEL_1_Pin GPIO_PIN_4
#define CS_MUX_SEL_1_GPIO_Port GPIOB
#define CS_MUX_SEL_2_Pin GPIO_PIN_5
#define CS_MUX_SEL_2_GPIO_Port GPIOB
#define LCD_BACKLIGHT_Pin GPIO_PIN_8
#define LCD_BACKLIGHT_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_9
#define LCD_RW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
