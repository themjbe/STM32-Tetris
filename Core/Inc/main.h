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

#define row_mask 0x1F

#define PRESCALE 0

#define BITS_PER_PIXEL 24
#define BITS_PER_CHANNEL 8

#define WIDTH 64
#define HEIGHT 64
#define SCAN_RATE 32 // this is a 1/32 display

#define BRIGHTNESS 10 // this can be from 1 to 10

#define PIXEL(f, x, y) f[y * 64 + x]

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern volatile uint8_t bit;
extern volatile uint8_t row;
extern volatile uint8_t busyFlag;
extern volatile uint32_t frame_count;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R1_Pin GPIO_PIN_0
#define R1_GPIO_Port GPIOC
#define G1_Pin GPIO_PIN_1
#define G1_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_2
#define B1_GPIO_Port GPIOC
#define R2_Pin GPIO_PIN_3
#define R2_GPIO_Port GPIOC
#define G2_Pin GPIO_PIN_4
#define G2_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_5
#define B2_GPIO_Port GPIOC
#define SELA_Pin GPIO_PIN_0
#define SELA_GPIO_Port GPIOB
#define SELB_Pin GPIO_PIN_1
#define SELB_GPIO_Port GPIOB
#define SELC_Pin GPIO_PIN_2
#define SELC_GPIO_Port GPIOB
#define LATCH_Pin GPIO_PIN_6
#define LATCH_GPIO_Port GPIOC
#define OE_Pin GPIO_PIN_7
#define OE_GPIO_Port GPIOC
#define CLOCK_Pin GPIO_PIN_8
#define CLOCK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SELD_Pin GPIO_PIN_3
#define SELD_GPIO_Port GPIOB
#define SELE_Pin GPIO_PIN_4
#define SELE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
