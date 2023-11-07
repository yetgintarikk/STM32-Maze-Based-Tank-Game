/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stm3210c_eval.h"
#include "stm3210c_eval_lcd.h"
#include "stm3210c_eval_io.h"
#include "stm3210c_eval_ts.h"

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
uint8_t CheckForUserInput(void);
uint16_t Calibration_GetX(uint16_t x);
uint16_t Calibration_GetY(uint16_t y);
uint8_t IsCalibrationDone(void);
void Touchscreen_Calibration(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_9
#define BUTTON_1_GPIO_Port GPIOD
#define BUTTON_2_Pin GPIO_PIN_10
#define BUTTON_2_GPIO_Port GPIOD
#define BUTTON_3_Pin GPIO_PIN_11
#define BUTTON_3_GPIO_Port GPIOD
#define BUTTON_4_Pin GPIO_PIN_12
#define BUTTON_4_GPIO_Port GPIOD
#define BUTTON_5_Pin GPIO_PIN_13
#define BUTTON_5_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
