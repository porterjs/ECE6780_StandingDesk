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


// Function to transmit a single character over USART3
	void UART_write(char c)
	{
		// Check and wait on the USART status flag that indicates the transmit register is empty.
		while((USART3->ISR & USART_ISR_TC) == 0){}
			
		// Write the character into the transmit data register
		USART3->TDR = c;
	}
	
	// Function to transmit a string of up to 64 characters over USART3
	void UART_writes(char s[64])
	{		
		for(int i = 0; i < 64; i++)
		{
			if (s[i] == 0)
			{
				return;
			} else 
			{
				UART_write(s[i]);
			}
		}		
	}

	
	// Global Variables
	char rx_char = 0;			// Global Character Received
	char rx_flag = 0;		// Data Received Flag
	
	// USART3 Interrupt Function
	void USART3_4_IRQHandler()
	{
		rx_char = (uint8_t)(USART3->RDR);
		rx_flag = 1;
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

	// Enable Peripheral Clocks
	RCC->AHBENR 	|= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR 	|= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR 	|= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGCOMPEN;
	//RCC->APB1ENR 	|= RCC_APB1ENR_TIM2EN;
	//RCC->APB1ENR 	|= RCC_APB1ENR_TIM3EN;
	RCC->APB1ENR 	|= RCC_APB1ENR_USART3EN;
	
	// Reset GPIOB Registers
	GPIOB->MODER 		&= ~(0xFFFFFFFF);
	GPIOB->OTYPER 	&= ~(0xFFFFFFFF);
	GPIOB->OSPEEDR 	&= ~(0xFFFFFFFF);
	GPIOB->PUPDR 		&= ~(0xFFFFFFFF);
	GPIOB->AFR[0] 	&= ~(0xFFFFFFFF);
	GPIOB->AFR[1] 	&= ~(0xFFFFFFFF);
	
	// Reset GPIOC Registers
	GPIOC->MODER 		&= ~(0xFFFFFFFF);
	GPIOC->OTYPER 	&= ~(0xFFFFFFFF);
	GPIOC->OSPEEDR 	&= ~(0xFFFFFFFF);
	GPIOC->PUPDR 		&= ~(0xFFFFFFFF);
	GPIOC->AFR[0] 	&= ~(0xFFFFFFFF);
	GPIOC->AFR[1] 	&= ~(0xFFFFFFFF);
	GPIOB->AFR[0] 	&= ~(0xFFFFFFFF);
	
	// Configure GPIO for USART3
	// TX = PB10
	// RX = PB11
	GPIOB->MODER 		|= (0x00A00000);	// Alternate Function
	GPIOB->AFR[1] 	|= (0x00004400);	// AF4
	
	// Configure USART3
	USART3->BRR = 69;	// 115200 Baud rate
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
	USART3->CR1 	|= USART_CR1_RXNEIE;
	
	/* TTL/USB Color Code
	 BLK = GND
	 BRN = CTS: Clear to Send (Handshake)
	 RED = VCC
	 ORG = TXD
	 YEL = RXD
	 GRN = Request to Send (Notify)
	*/
	
	// Configure Orange LED  PC9
	GPIOC->MODER 	|= (1 << 16);
	
	// Configure Green LED  PC8
	GPIOC->MODER 	|= (1 << 18);

	// Configure Red LED  PC6
	GPIOC->MODER 	|= (1 << 12); 
	
	// Configure Blue LED  PC7
	GPIOC->MODER 	|= (1 << 14);  
	
	// Enable and Set Priority fo the USART3 Interrupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);
	
  while (1)
  {
		if (rx_flag) // Receive Message Ready
		{
			switch(rx_char)
			{
				case 'R':
					GPIOC->ODR ^= GPIO_ODR_6;  // Toggle red LED
					break;
				case 'B':
					GPIOC->ODR ^= GPIO_ODR_7;		// Toggle blue LED
					break;
				case 'U':
					break;
				case 'D':
					
					break;
			}
			
		}
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

