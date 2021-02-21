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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BoardA_handle.h"
#include "traitement.h"
#include "robot_configuration.h"
#include "pilotes.h"
#include "canon.h"

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
#define DEBUG_TX_Pin GPIO_PIN_1
#define DEBUG_TX_GPIO_Port GPIOE
#define DEBUG_RX_Pin GPIO_PIN_0
#define DEBUG_RX_GPIO_Port GPIOE
#define RefereeSystem_Tx_Pin GPIO_PIN_14
#define RefereeSystem_Tx_GPIO_Port GPIOG
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define ReceiverRC_RX_Pin GPIO_PIN_7
#define ReceiverRC_RX_GPIO_Port GPIOB
#define ReceiverRC_TX_Pin GPIO_PIN_6
#define ReceiverRC_TX_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define RefereeSystem_Rx_Pin GPIO_PIN_9
#define RefereeSystem_Rx_GPIO_Port GPIOG
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define BOARD_POWER1_CTRL_Pin GPIO_PIN_2
#define BOARD_POWER1_CTRL_GPIO_Port GPIOH
#define BOARD_POWER2_CTRL_Pin GPIO_PIN_3
#define BOARD_POWER2_CTRL_GPIO_Port GPIOH
#define BOARD_POWER3_CTRL_Pin GPIO_PIN_4
#define BOARD_POWER3_CTRL_GPIO_Port GPIOH
#define BOARD_LED_A_Pin GPIO_PIN_8
#define BOARD_LED_A_GPIO_Port GPIOG
#define BOARD_POWER4_CTRL_Pin GPIO_PIN_5
#define BOARD_POWER4_CTRL_GPIO_Port GPIOH
#define BOARD_LED_B_Pin GPIO_PIN_7
#define BOARD_LED_B_GPIO_Port GPIOG
#define BOARD_LED_C_Pin GPIO_PIN_6
#define BOARD_LED_C_GPIO_Port GPIOG
#define BOARD_LED_D_Pin GPIO_PIN_5
#define BOARD_LED_D_GPIO_Port GPIOG
#define BOARD_LED_E_Pin GPIO_PIN_4
#define BOARD_LED_E_GPIO_Port GPIOG
#define BOARD_LED_F_Pin GPIO_PIN_3
#define BOARD_LED_F_GPIO_Port GPIOG
#define BOARD_LED_G_Pin GPIO_PIN_2
#define BOARD_LED_G_GPIO_Port GPIOG
#define BOARD_LED_H_Pin GPIO_PIN_1
#define BOARD_LED_H_GPIO_Port GPIOG
#define OLED_Analog_Pin GPIO_PIN_6
#define OLED_Analog_GPIO_Port GPIOA
#define JETSON_TX_Pin GPIO_PIN_8
#define JETSON_TX_GPIO_Port GPIOE
#define BOARD_LED_RED_Pin GPIO_PIN_11
#define BOARD_LED_RED_GPIO_Port GPIOE
#define OLED_MOSI_Pin GPIO_PIN_7
#define OLED_MOSI_GPIO_Port GPIOA
#define BOARD_LED_GREEN_Pin GPIO_PIN_14
#define BOARD_LED_GREEN_GPIO_Port GPIOF
#define JETSON_RX_Pin GPIO_PIN_7
#define JETSON_RX_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
