/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void USART1_IRQHandler(void) {
	
	static int buf_index = 0;
	static char buf[16];
	
	char c = USART1->RDR;
	
	if (c == '\n') {
		
		if (!strncmp(buf, "<3", 2)) {
			// Heartbeat
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		} else if (!strncmp(buf, "UP", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Manual raise desk
		} else if (!strncmp(buf, "DN", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Manual lower desk
		} else if (!strncmp(buf, "P1", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Go to standing position
		} else if (!strncmp(buf, "P2", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Go to sitting position
		} else if (!strncmp(buf, "HM", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Start calibration cycle (turtle speed)
		} else if (!strncmp(buf, "SP1=", 4)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Standing height in inches
			uint8_t height = atoi(buf+4);
		} else if (!strncmp(buf, "SP2=", 4)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Sitting height in inches
			uint8_t height = atoi(buf+4);
		} else if (!strncmp(buf, "GN=", 3)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Sitting height in inches
			uint8_t gains = atoi(buf+3);
		} else if (!strncmp(buf, "ST", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Stop
		} else if (!strncmp(buf, "LU", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Left leg raise
		} else if (!strncmp(buf, "LD", 2)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			// Left leg lower
		} else if (!strncmp(buf, "RU", 2)) {
			// Right leg raise
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		} else if (!strncmp(buf, "RD", 2)) {
			// Right leg lower
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		} else if (!strncmp(buf, "RM", 2)) {
			// Rabbit mode
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		} else if (!strncmp(buf, "TM", 2)) {
			// Turtle mode
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		} else if (!strncmp(buf, "RS=", 3)) {
			// Set rabbit speed
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			uint8_t rabbit_speed = atoi(buf+3);
		} else if (!strncmp(buf, "TS=", 3)) {
			// Set turtle speed
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			uint8_t turtle_speed = atoi(buf+3);
		}
		
		memset(buf, 0, sizeof(buf));
		buf_index = 0;
	}
	else {
		buf[buf_index] = c;
		buf_index++;
	}
	// Set RXNE Interrrupt bit
	USART1->CR1 |= 0x1 << 5;
}
/* USER CODE END 1 */

