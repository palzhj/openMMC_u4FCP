/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define AMC_MODE_Pin GPIO_PIN_13
#define AMC_MODE_GPIO_Port GPIOC
#define FPGA_PROG_B_Pin GPIO_PIN_14
#define FPGA_PROG_B_GPIO_Port GPIOC
#define FPGA_DONE_Pin GPIO_PIN_15
#define FPGA_DONE_GPIO_Port GPIOC
#define AMC_MODULE_HANDLE_Pin GPIO_PIN_0
#define AMC_MODULE_HANDLE_GPIO_Port GPIOA
#define FPGA_RST_B_Pin GPIO_PIN_1
#define FPGA_RST_B_GPIO_Port GPIOA
#define RTM_RES_Pin GPIO_PIN_2
#define RTM_RES_GPIO_Port GPIOA
#define RTM_INT_B_Pin GPIO_PIN_3
#define RTM_INT_B_GPIO_Port GPIOA
#define CLK0_INT_B_Pin GPIO_PIN_4
#define CLK0_INT_B_GPIO_Port GPIOA
#define CLK1_INT_B_Pin GPIO_PIN_5
#define CLK1_INT_B_GPIO_Port GPIOA
#define CLK0_LOL_B_Pin GPIO_PIN_6
#define CLK0_LOL_B_GPIO_Port GPIOA
#define CL1K_LOL_B_Pin GPIO_PIN_7
#define CL1K_LOL_B_GPIO_Port GPIOA
#define AMC_LED_GREEN_Pin GPIO_PIN_0
#define AMC_LED_GREEN_GPIO_Port GPIOB
#define FMC0_PRSNT_B_Pin GPIO_PIN_12
#define FMC0_PRSNT_B_GPIO_Port GPIOB
#define FMC1_PRSNT_B_Pin GPIO_PIN_13
#define FMC1_PRSNT_B_GPIO_Port GPIOB
#define FMC0_CLK_DIR_Pin GPIO_PIN_14
#define FMC0_CLK_DIR_GPIO_Port GPIOB
#define FMC1_CLK_DIR_Pin GPIO_PIN_15
#define FMC1_CLK_DIR_GPIO_Port GPIOB
#define RTM_PS_B_Pin GPIO_PIN_8
#define RTM_PS_B_GPIO_Port GPIOA
#define AMC_GA2_Pin GPIO_PIN_11
#define AMC_GA2_GPIO_Port GPIOA
#define AMC_GA1_Pin GPIO_PIN_12
#define AMC_GA1_GPIO_Port GPIOA
#define AMC_GA0_Pin GPIO_PIN_15
#define AMC_GA0_GPIO_Port GPIOA
#define AMC_GA_Pin GPIO_PIN_3
#define AMC_GA_GPIO_Port GPIOB
#define AMC_LED_BLUE_Pin GPIO_PIN_4
#define AMC_LED_BLUE_GPIO_Port GPIOB
#define AMC_LED_RED_Pin GPIO_PIN_5
#define AMC_LED_RED_GPIO_Port GPIOB
#define PMBUS_ALERT_B_Pin GPIO_PIN_8
#define PMBUS_ALERT_B_GPIO_Port GPIOB
#define PMBUS_CTRL_Pin GPIO_PIN_9
#define PMBUS_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
