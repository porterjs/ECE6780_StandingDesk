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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum states { IDLE, MOTOR_UP };
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// --- UART READ DEFINITIONS --- //
// GENERAL
#define NONE				      0x00

#define NEWLINE           0x0A
#define HMI_HEARTBEAT     0xFF

// BASIC CONTROLS
#define DESK_STOP         0x10
#define DESK_RAISE        0x11
#define DESK_LOWER        0x12
#define DESK_HOME         0x13
#define DESK_GOTO_SP1     0x14
#define DESK_GOTO_SP2     0x15


// ADVANCED CONTROLS 
#define MTR1_RAISE        0x20
#define MTR1_LOWER        0x21
#define MTR2_RAISE        0x22
#define MTR2_LOWER        0x23

// MOTOR CONFIG
#define TURTLE_SPEED_SP   0x30
#define RABBIT_SPEED_SP   0x31
#define RABBIT_SPEED_EN   0x32

// ADVANCED CONFIG 
#define SET_SPEED_SCALE   0x40
#define SET_GAIN_P        0x41
#define SET_GAIN_I        0x42
#define SET_GAIN_D        0x43
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void sendChar(char c);

void sendStr(char* s);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
int current_state = IDLE;
int req_start_index = 0;
int req_end_index = 0;
int req_pending = 0;
char buf[32];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
