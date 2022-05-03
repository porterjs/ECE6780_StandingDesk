/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Define Global Variables */


#define uncalibrated_mask = 0x00;
#define ftc_mask					= 0x01;	// Failed-to-calibrate
#define overtravel_mask		= 0x02;
#define reversepol_mask		= 0x03;
#define overcurrent_mask	= 0x04;	// Future expansion


int8_t MTR1_SR = 0;	// Motor 1 Status Register
int8_t MTR2_SR = 0; // Motor 2 Status Register

int8_t LS1_SR = 0;
int8_t LS2_SR = 0;

int8_t AlarmGroup1 = 0;

	
void RCC_init()
{
	// Enable Peripheral RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGCOMPEN;
	// RCC->APB1ENR 	|= RCC_APB1ENR_TIM2EN;
	// RCC->APB1ENR 	|= RCC_APB1ENR_TIM3EN;
	// RCC->APB1ENR 	|= RCC_APB1ENR_USART3EN;
}

void LED_init()
{///
	/*
	PC6: Red
	PC7: Blue
	PC8: Orange
	PC9: Green
	*/
	
	// Configure LED Outputs
	GPIOC->MODER 		|=  (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);					// Set Output Mode
	GPIOC->MODER 		&= ~(GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);					// ...
	GPIOC->OTYPER 	&= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);											// Output Push-Pull
	GPIOC->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9);	// Low Speed 
	GPIOC->ODR 			&= ~(GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9);																							// Off
}///

void LimitSwitch_init()
{///
	/* 
	PA0: Prox Sensor 1 (N.C.)
	PA1: Prox Sensor 2 (N.C.)
	*/
	
	// Configure GPIO Inputs
	GPIOA->MODER 		&= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);					// Set Input Mode
	GPIOA->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1);	// Reset Output Speed
	GPIOA->PUPDR 		|=  (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1);			// Set Pull-down resistor
	GPIOA->PUPDR 		&= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0);			// ...
	
	// EXT1 Configuration
	EXTI->IMR  |= EXTI_IMR_IM0 | EXTI_IMR_IM1;			// Enable Interrupt Masks
	EXTI->RTSR |= EXTI_RTSR_RT0 | EXTI_RTSR_RT1;		// Enable Rising Edge Trigger
	EXTI->FTSR |= EXTI_FTSR_FT0 | EXTI_FTSR_FT1;		// Enable Falling Edge Trigger
	
	// Set EXTI0 Multiplexer for PA0
	SYSCFG->EXTICR[0] &= (uint16_t) ~SYSCFG_EXTICR1_EXTI0_PA;
	
	// Enable and Set Priority fo the EXTI Interrupt
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn,2);
	
	// Check Current State
	
}///


void Buzzer_init()
{///
	// PA8: Buzzer Output
	
	// Configure Buzzer Output
	GPIOA->MODER 		|= GPIO_MODER_MODER8_0;			// Output Mode
	GPIOA->MODER 		&= ~GPIO_MODER_MODER8_1;		// ...
	GPIOA->OTYPER 	&= ~GPIO_OTYPER_OT_8;				// Push-pull type
	GPIOA->OSPEEDR 	&= ~GPIO_OSPEEDR_OSPEEDR8;	// Low Speed
	GPIOA->PUPDR 		|= GPIO_PUPDR_PUPDR8_1;			// Pull-down
	GPIOA->PUPDR 		&= ~GPIO_PUPDR_PUPDR8_0;		// ...
	GPIOA->ODR 			&= ~GPIO_ODR_8;							// Off
}///

void Blink()
{///
	// Blink Green LED
	GPIOC->ODR ^= GPIO_ODR_9;
	HAL_Delay(1000);
}///	




void MTR1_init()
{
	/*
	PA11: Motor 1 FWD
	PA12: Motor 1 REV
	PB8: Motor 1 Enable
	*/
	
	
	
	
	// Configure PWM
	GPIOB->MODER |= GPIO_MODER_MODER8_1;		// AF Mode
	GPIOB->MODER &= ~GPIO_MODER_MODER8_0;	// ...
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_8;	// Push Pull Type
	
	// Set AF2 TIM16
	GPIOB->AFR[1] &= 0xFFFFFFF0;	// Clear PB8
	GPIOB->AFR[1] |= (1 << 1);		
	
	// Set up PWM timers
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	
	// Clear Timer Registers
	TIM16->CR1 = 0;	
	TIM16->CCMR1 = 0;
	TIM16->CCER = 0;
	
	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM16->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE); 
	TIM16->CCER |= TIM_CCER_CC1E;	// Enable Capture-compare channel 1 
	TIM16->PSC = 1;	// Set Prescale for 24Mhz
	TIM16->ARR = 1200;	// PWM at 40kHz
	TIM16->CCR1 = 0;		// Initialize PWM at 0% Duty Cycle
	TIM16->CR1 |= TIM_CR1_CEN; // Enable Timer
}

void MTR2_init()
{
	/*
	PB0: Motor 2 FWD
	PB1: Motor 2 REV
	PB9: Motor 2 Enable
	*/
	
	// Configure PB0/PB1 GPIO as Outputs
	GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1);
	
	// Configure PB9 for PWM
	GPIOB->MODER |= GPIO_MODER_MODER9_1;		// AF Mode
	GPIOB->MODER &= ~GPIO_MODER_MODER9_0;	// ...
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_9;	// Push Pull Type
	
	// Set AF2 TIM17
	GPIOB->AFR[1] &= 0xFFFFFF0F;	// Clear PB9
	GPIOB->AFR[1] |= (1 << 5);		
	
	// Set up PWM timers
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	
	// Clear Timer Registers
	TIM17->CR1 = 0;
	TIM17->CCMR1 = 0;
	TIM17->CCER = 0;
	
	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM17->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIM17->CCER |= TIM_CCER_CC1E; // Enable Capture-compare channel 1 
	TIM17->PSC = 1; 	// Set Prescale for 24Mhz
	TIM17->ARR = 1200;	// PWM at 40kHz	
	TIM17->CCR1 = 0;	// Initialize PWM at 0% Duty Cycle
	TIM17->CR1 |= TIM_CR1_CEN; // Enable Timer
}

void MTR1_SetDuty(uint8_t duty)
{
	if (duty <= 100)
	{
		TIM16->CCR1 = ((uint32_t)duty*TIM16->ARR)/100;	// Use linear transform to produce CCR1 value
	}
}

void MTR2_SetDuty(uint8_t duty)
{
	if (duty <= 100)
	{
		TIM17->CCR1 = ((uint32_t)duty*TIM17->ARR)/100;	// Use linear transform to produce CCR1 value
	}
}

void SpeedTest()
{
	static uint8_t myduty = 0;
	
	myduty = myduty + 10;
	if (myduty > 100) {myduty = 0;}
	
	MTR1_SetDuty(myduty);	
	
	HAL_Delay(1000);
}	


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	/* Enable Peripheral RCC */
	RCC_init();
	
	/* Enable LEDs */
	LED_init();
	
	/* Enable Limit Switches */
	LimitSwitch_init();
	
	/* Enable Buzzer */
	Buzzer_init();
	
	/* Enable Motors */
	MTR1_init();
	MTR2_init();
	
	
	
	// Configure PA11/PA12 GPIO as Outputs
//	GPIOA->MODER 		|=  (GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0);
//	GPIOA->MODER 		&= ~(GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
//	GPIOA->OTYPER 	&= ~(GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12);
//	GPIOA->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR11 | GPIO_OSPEEDR_OSPEEDR12);
//	GPIOA->PUPDR 		&= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
//	
	// Configure Buzzer Output
	GPIOA->MODER 		|= GPIO_MODER_MODER11_0;			// Output Mode
	GPIOA->MODER 		&= ~GPIO_MODER_MODER11_1;		// ...
	GPIOA->OTYPER 	&= ~GPIO_OTYPER_OT_11;				// Push-pull type
	GPIOA->OSPEEDR 	&= ~GPIO_OSPEEDR_OSPEEDR11;	// Low Speed
	GPIOA->PUPDR 		|= GPIO_PUPDR_PUPDR11_1;			// Pull-down
	GPIOA->PUPDR 		&= ~GPIO_PUPDR_PUPDR11_0;		// ...
	GPIOA->ODR 			|= GPIO_ODR_11;							// Off
	
	// Motor Forward
	//GPIOA->ODR |= GPIO_ODR_11;
	
  while (1)
  {
			
		// Code Running Indicator
		Blink();
		
		
		GPIOB->ODR |= GPIO_ODR_0;
		//HAL_Delay(20);
		
		//uint8_t duty = 80;
		
		//MTR1_SetDuty(duty);	
		//MTR2_SetDuty(duty);
		// Test Motor Speed
		//SpeedTest();
		
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		
		GPIOC->ODR |= GPIO_ODR_9;
		HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

