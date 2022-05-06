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
int ticks_left = 1000;

// Motor Ramp Delays (SysTick Delay)
int RampDelay1 = 0;
int RampDelay2 = 0;

int Prox1 = 0;
int Prox2 = 0;

int UpdateDelay = 500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern int req_start_index;
extern int req_end_index;
extern char buf[32];
extern int req_pending;

extern int current_state;
extern void MTR1_Stop();
extern void MTR2_Stop();
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
	ticks_left --;
	
	if (ticks_left == 0) {
		current_state = IDLE;
		MTR1_Stop();
		MTR2_Stop();
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

	
	// Ramp Delays
	if (RampDelay1 > 0) {RampDelay1 --;}
	if (RampDelay2 > 0) {RampDelay2 --;}
  /* USER CODE END SysTick_IRQn 1 */
	
	if (UpdateDelay > 0) {UpdateDelay --;}
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void USART1_IRQHandler(void) {
	
	char c = USART1->RDR;
	
	if (c == (char)HMI_HEARTBEAT) {
		ticks_left = 1000;
	} else {
		
		if (c == (char)NEWLINE) {
			req_pending++;
		}
		
		buf[req_end_index] = c;
		req_end_index++;
		
		if (req_end_index == 32) {
			req_end_index = 0;
		}
		
	}	
	
	// Set RXNE Interrrupt bit
	USART1->CR1 |= 0x1 << 5;
}

void LimitSwitch1_Update()
{
	// Check Prox Sensor 1
		if ((GPIOC->IDR & GPIO_IDR_0) == GPIO_IDR_0)
		{
			Prox1 = 1;
			GPIOC->ODR |= GPIO_ODR_6;		
		}
		else 
		{
			Prox1 = 0;
			TIM3->CNT = 0x7FFF;
			GPIOC->ODR &= ~GPIO_ODR_6;
		}
	}

void LimitSwitch2_Update()
{
	// Check Prox Sensor 2
	if ((GPIOC->IDR & GPIO_IDR_3) == GPIO_IDR_3)
	{
		Prox2 = 1;
		GPIOC->ODR |= GPIO_ODR_7;
	}
	else 
	{
		Prox2 = 0;
		TIM2->CNT = 0x7FFF;
		GPIOC->ODR &= ~GPIO_ODR_7;
	}
}
	
// Writing the EXTI Interrupt Handler (Limit Switches
void EXTI0_1_IRQHandler(){
	
	LimitSwitch1_Update();
	
			
	// Acknowledge Interrupt Complete
	EXTI->PR |= (1 << 0);
}

void EXTI2_3_IRQHandler(){
	
	LimitSwitch2_Update();
	
	
	// Acknowledge Interrupt Complete
	EXTI->PR |= (1 << 3);
}
		
/* USER CODE END 1 */

