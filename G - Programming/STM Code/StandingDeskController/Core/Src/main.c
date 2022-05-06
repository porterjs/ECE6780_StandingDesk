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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
int read_buffer_int() {
	char int_buf[16];
	int end_index = req_start_index;
	
	int i = 0;
	while (buf[end_index] != NEWLINE && i < 15) { // If that second condition is true, something is wrong, I just don't know how to handle it yet.
		int_buf[i] = buf[end_index];
		end_index++;
		if (end_index == 32) {
			end_index = 0;
		}
	}
	
	return atoi(int_buf);
}

/* USER CODE BEGIN 0 */
int read_buffer() {
	int command = buf[req_start_index];
	
	if (req_start_index == 32) {
			req_start_index = 0;
	}
	req_pending--;
	
	switch (command) {
		case NEWLINE:
			return NONE; // Must return here otherwise we keep reading into nothingness...
			break;
		case DESK_STOP:
			break;
		case DESK_RAISE:
			break;
		case DESK_LOWER:
			break;
		case DESK_HOME:
			break;
		case DESK_GOTO_SP1:
			break;
		case DESK_GOTO_SP2:
			break;
		case MTR1_RAISE:
			break;
		case MTR1_LOWER:
			break;
		case MTR2_RAISE:
			break;
		case MTR2_LOWER:
			break;
		case TURTLE_SPEED_SP:
			// turtle_speed = read_buffer_int();
			break;
		case RABBIT_SPEED_SP:
			// rabbit_speed = read_buffer_int();
			break;
		case RABBIT_SPEED_EN:
			// rabbit_speed_en = read_buffer_int();
			break;
		case SET_SPEED_SCALE:
			// speed_scale = read_buffer_int();
			break;
		case SET_GAIN_P:
			// gain_p = read_buffer_int();
			break;
		case SET_GAIN_I:
			// gain_i = read_buffer_int();
			break;
		case SET_GAIN_D:
			// gain_d = read_buffer_int();
			break;
		default:
			command = NONE; // We got a garbage character
			break;
	}
	
	while (buf[req_start_index] != NEWLINE) {
		req_start_index++;
		if (req_start_index == 32) {
			req_start_index = 0;
		}
	}
	
	req_start_index++; // Increment one more time to go past the newline character.
	if (req_start_index == 32) {
			req_start_index = 0;
	}
	
	return command;
}
/* USER CODE END 0 */

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

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // LEDs for testing
	GPIO_InitTypeDef initStrLED = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStrLED);
	
	// Init UART TX/RX Pins
	GPIO_InitTypeDef initStrUART = {GPIO_PIN_9 | GPIO_PIN_10, GPIO_MODE_AF_PP, NULL, NULL};
	HAL_GPIO_Init(GPIOA, &initStrUART);
	
	// Set AFR to alternate function AF4
	GPIOA->AFR[1] |= (0x1 << 4 | 0x1 << 8);
	
	// Set Baud rate to 115200
	USART1->BRR = 69;
	
	// Enable RX, TX, and USART Module
	USART1->CR1 |= (0x1 << 3) | (0x1 << 2) | 0x1;
	
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	// Set RXNE Interrrupt bit
	USART1->CR1 |= 0x1 << 5;
	
  /* USER CODE END 2 */

  // Initialize PWM
	 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIO
	pwm_init();
	pwm_setDutyCycle(25);
	
	// Initialize External Interrupts
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1;	// PA0 Pull-down 
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
	//EXTI->IMR |= EXTI_IMR_IM0 | EXTI_IMR_IM1;	// Enable Interrupt Mask for EXT0-1
	//EXTI->RTSR |= EXTI_RTSR_RT0 | EXTI_RTSR_RT1; // Enable Rising Edge Trigger for EXT0-1
	//EXTI->FTSR |= EXTI_FTSR_FT0 | EXTI_FTSR_FT1;  // Enable Falling Edge Trigger for EXT0-1
	//SYSCFG->EXTICR[0] &= ~SYSCFG_EXT 
	EXTI->IMR = 0x0001; /* (3) */
	EXTI->RTSR = 0x0001; /* (4) */
	EXTI->FTSR = 0x0001; /* (5) */

	NVIC_EnableIRQ(EXTI0_1_IRQn);	// Enable Interrupt Handler
	NVIC_SetPriority(EXTI0_1_IRQn,0);	// Set Interrupt Priority
	
  /* USER CODE BEGIN WHILE */
  while (1)
  {
<<<<<<< HEAD
    /* USER CODE END WHILE */
		// HAL_Delay(250);
		// sendStr("Doing other tasks...\n\r");
		
		GPIOC->ODR |= GPIO_ODR_8;
=======
		int command = NONE;
    if (req_pending > 0) {
			command = read_buffer();
		}
		
		/*	
			UNCALIBRATED
			HOMING
			IDLE
			DESK_LOWERING
			DESK_RAISING
			TARGET_POSITIONING
			LEGL_LOWERING
			LEGL_RAISING
			LEGR_LOWERING
			LEGR_RAISING
		*/
		switch (current_state) {

			case UNCALIBRATED:
				if (command == DESK_HOME) {
					current_state = HOMING;
					// ENABLE WDTMTR
				}
				break;
			case HOMING:
				if (1) { // IF Calibration is done - do we need to make a calculation or set a variable on an interrupt?
					current_state = IDLE;
					// Reset Outputs
					// DISABLE WDTMTR
				} else if (0) { // WDTMTR_DN
					// FTH
				}
				break;
			case IDLE:
				switch (command) {
					case DESK_HOME:
						current_state = HOMING;
						// ENABLE WDTMTR
						break;
					case DESK_RAISE:
						current_state = DESK_RAISING;
					case DESK_LOWER:
						current_state = DESK_LOWERING;
						break;
					case DESK_GOTO_SP1:
						// Set target
						current_state = TARGET_POSITIONING;
						// ENABLE WDTMTR
						break;
					case DESK_GOTO_SP2:
						// Set target
						current_state = TARGET_POSITIONING;
						// ENABLE WDTMTR
						break;
					case MTR1_RAISE:
						current_state = LEGL_RAISING;
						break;
					case MTR1_LOWER:
						current_state = LEGL_LOWERING;
						break;
					case MTR2_RAISE:
						current_state = LEGR_RAISING;
						break;
					case MTR2_LOWER:
						current_state = LEGR_LOWERING;
						break;
					default:
						break;
				}
			case DESK_LOWERING:
			case DESK_RAISING:
			case LEGL_LOWERING:
			case LEGL_RAISING:
			case LEGR_LOWERING:
			case LEGR_RAISING:
				if (command == DESK_STOP) {
					current_state = IDLE;
				}
				break;
			case TARGET_POSITIONING:
				if (1) { // Done Positioning
					current_state = IDLE;
					// DISABLE WDTMTR
				}
			default:
				// How did we get here? (No seriously, if we are here, something is wrong)
				break;
		}
>>>>>>> 9bcb278f2076c571b8d267309b29cd5859ffdc25
  }
  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */
void sendChar(char c) {
	while (!((USART1->ISR & USART_ISR_TC) == USART_ISR_TC));
	
	USART1->TDR = c;
}

void sendStr(char* s) {	
	int i = 0;
	while (s[i]) {
		while (!((USART1->ISR & USART_ISR_TC) == USART_ISR_TC));
		USART1->TDR = s[i];
		i++;
	}
}

void pwm_init(void) {
    
    // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
  //  TIM14->ARR = 1200;                      // PWM at 20kHz							<--- creates high pitch humming
		TIM14->ARR = 1200;                      // PWM at 40kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle
    
    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

/* USER CODE END 4 */

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

