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

#include "string.h"
#include "stdlib.h"
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Define Global Variables */
#define uncalibrated_mask = 0x00;
#define ftc_mask					= 0x01;	// Failed-to-calibrate
#define overtravel_mask		= 0x02;
#define reversepol_mask		= 0x03;
#define overcurrent_mask	= 0x04;	// Future expansion

uint8_t MTR1_SR = 0;	// Motor 1 Status Register
uint8_t MTR2_SR = 0; // Motor 2 Status Register

#define running = 0x00;		// True if encoder is changing 
#define forward = 0x01;		// True if encoder incrementing
#define reverse = 0x02;		// True if encoder decrementing

uint8_t LS1_SR = 0;
uint8_t LS2_SR = 0;

uint8_t AlarmGroup1 = 0;
uint8_t test;
//
extern int Prox1;
extern int Prox2;

// Ramp Delay (SysTick)
extern int RampDelay1;
extern int RampDelay2;

extern int UpdateDelay;

// Motor Variables
uint8_t MTR1_CurrentSpeed = 0;
uint8_t MTR2_CurrentSpeed = 0;
uint8_t TurtleSpeed = 50;
uint8_t RabbitSpeed = 90;

const uint8_t TICKS_PER_INCH = 228;
const uint8_t TICKS_PER_MM = 9;

int SP1 = 2280;
int SP2 = 570;
int Overtravel_SP = 712;

// 8in = 1140

int Overtravel = 0;

// State Variables
int current_state = IDLE;

// UART Variables
int req_start_index = 0;
int req_end_index = 0;
int req_pending = 0;
char buf[32];
	
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
	PC0: Prox Sensor 1 (N.C.)
	PC1: Prox Sensor 2 (N.C.)
	*/
	
	// Configure GPIO Inputs
	GPIOC->MODER 		&= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER3);					// Set Input Mode
	GPIOC->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR3);	// Reset Output Speed
	GPIOC->PUPDR 		|=  (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR3_1);			// Set Pull-down resistor
	GPIOC->PUPDR 		&= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR3_0);			// ...
	
	// EXT1 Configuration
	EXTI->IMR  |= EXTI_IMR_IM0 | EXTI_IMR_IM3;			// Enable Interrupt Masks
	EXTI->RTSR |= EXTI_RTSR_RT0 | EXTI_RTSR_RT3;		// Enable Rising Edge Trigger
	EXTI->FTSR |= EXTI_FTSR_FT0 | EXTI_FTSR_FT3;		// Enable Falling Edge Trigger
	
	// Set EXTI0 Multiplexer for PC0
	SYSCFG->EXTICR[0] &= 0xFFFF0FF0;
	SYSCFG->EXTICR[0] |= (1 << 1) | (1 << 13);
	
	// Enable and Set Priority fo the EXTI Interrupt
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn,2);
	NVIC_SetPriority(EXTI2_3_IRQn,2);
	
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

void Buzzer_On()
{
	GPIOA->ODR 			|= GPIO_ODR_8;
}

void Buzzer_Off()
{
	GPIOA->ODR 			&= ~GPIO_ODR_8;
}

void Blink()
{///
	// Blink Green LED
	GPIOC->ODR ^= GPIO_ODR_9;
	HAL_Delay(1000);
}///	




void MTR1_init()
{
	/*
	PB2: Motor 1 FWD
	PB3: Motor 1 REV
	PB8: Motor 1 Enable
	*/
	
	
		// Configure PB2/PB3 GPIO as Outputs
	GPIOB->MODER 		|=  (GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0);
	GPIOB->MODER 		&= ~(GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
	GPIOB->OTYPER 	&= ~(GPIO_OTYPER_OT_2 | GPIO_OTYPER_OT_3);
	GPIOB->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR2 | GPIO_OSPEEDR_OSPEEDR3);
	GPIOB->PUPDR 		&= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
	GPIOB->ODR 			&= ~(GPIO_ODR_2 | GPIO_ODR_3);							// Off
	
	// Configure PWM
	GPIOA->MODER 	|= GPIO_MODER_MODER4_1;		// AF Mode
	GPIOA->MODER 	&= ~GPIO_MODER_MODER4_0;	// ...
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4;	// Push Pull Type
	
	// Set AF4 TIM14
	GPIOA->AFR[0] &= 0xFFF0FFFF;	// Clear PA4
	GPIOA->AFR[0] |= (1 << 18);		
	
	// Set up PWM timers
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	
	// Clear Timer Registers
	TIM14->CR1 		= 0;	
	TIM14->CCMR1 	= 0;
	TIM14->CCER 	= 0;
	
	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM14->CCMR1 	|= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE); 
	TIM14->CCER 	|= TIM_CCER_CC1E;	// Enable Capture-compare channel 1 
	TIM14->PSC 		 = 1;			// Set Prescale for 24Mhz
	TIM14->ARR 		 = 1200;	// PWM at 40kHz
	TIM14->CCR1 	 = 0;			// Initialize PWM at 0% Duty Cycle
	TIM14->CR1 		|= TIM_CR1_CEN; // Enable Timer
}

void MTR2_init()
{
	/*
	PB7: Motor 2 FWD
	PB8: Motor 2 REV
	PB9: Motor 2 Enable
	*/
	
	// Configure PB7/PB8 GPIO as Outputs
	GPIOB->MODER 		|= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0;
	GPIOB->MODER 		&= ~(GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1);
	GPIOB->OTYPER 	&= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8);
	GPIOB->OSPEEDR 	&= ~(GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR8);
	GPIOB->PUPDR 		&= ~(GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8);
	
	// Configure PB9 for PWM
	GPIOB->MODER 	|= GPIO_MODER_MODER9_1;		// AF Mode
	GPIOB->MODER 	&= ~GPIO_MODER_MODER9_0;	// ...
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_9;			// Push Pull Type
	
	// Set AF2 TIM17
	GPIOB->AFR[1] &= 0xFFFFFF0F;	// Clear PB9
	GPIOB->AFR[1] |= (1 << 5);		
	
	// Set up PWM timers
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	
	// Clear Timer Registers
	TIM17->CR1 		= 0;
	TIM17->CCMR1 	= 0;
	TIM17->CCER 	= 0;
	
	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM17->CCMR1 	|= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIM17->CCER 	|= TIM_CCER_CC1E; // Enable Capture-compare channel 1 
	TIM17->PSC 		 = 1; 	// Set Prescale for 24Mhz
	TIM17->ARR 		 = 1200;	// PWM at 40kHz	
	TIM17->CCR1 	 = 0;	// Initialize PWM at 0% Duty Cycle
	TIM17->BDTR 	|= TIM_BDTR_MOE;
	TIM17->CR1 		|= TIM_CR1_CEN; // Enable Timer
}

void MTR1_SetDuty(uint8_t duty)
{
	if (duty <= 100)
	{
		TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;	// Use linear transform to produce CCR1 value
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


void encoder_init()
{
	/*
	PB4: MTR1 Encoder A
	PB5: MTR1 Encoder B
	*/


    GPIOB->MODER 	&= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER 	|= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] &= 0xFF00FFFF;
		GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 	= 0;
    TIM3->CCER 		= 0;
    TIM3->SMCR 		= 0;
    TIM3->CR1 		= 0;

    TIM3->CCMR1 	|= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR 		|= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR 		 = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT 		 = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    TIM3->CR1 		|= TIM_CR1_CEN;                               // Enable timer

	/*
	PA0: MTR1 Encoder A
	PA1: MTR1 Encoder B
	*/


    GPIOA->MODER 	&= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0);
    GPIOA->MODER 	|=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOA->AFR[0] &=  0xFFFFFF00;
		GPIOA->AFR[0] |=  ((1 << 1) | (1 << 5));

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CCMR1 	= 0;
    TIM2->CCER		= 0;
    TIM2->SMCR		= 0;
    TIM2->CR1 		= 0;

    TIM2->CCMR1 	|= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM2->SMCR		|= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM2->ARR 		 = 0xFFFF;                                   // Set ARR to top of timer (longest possible period)
    TIM2->CNT 		 = 0x7FFF;                                   // Bias at midpoint to allow for negative rotation
    //TIM2->BDTR		|= TIM_BDTR_MOE;
		TIM2->CR1 		|= TIM_CR1_CEN;                              // Enable timer

}


void UART_init()
{
	__HAL_RCC_USART1_CLK_ENABLE();
	
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
}


///////////////////////////


int read_buffer_int() {
    char int_buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int end_index = req_start_index;

    int i = 0;
    while (buf[end_index] != NEWLINE && i < 15) { // If that second condition is true, something is wrong, I just don't know how to handle it yet.
        int_buf[i] = buf[end_index];
        i++;
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
	
	req_start_index ++;
	
	if (req_start_index == 32) {
			req_start_index = 0;
	}
	req_pending--;
	
	switch (command) {
		case NEWLINE:
			return NONE; // Must return here otherwise we keep reading into nothingness...
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


void MTR1_FWD()
{
	GPIOB->ODR &= ~GPIO_ODR_2;	// MTR1 REV
	GPIOB->ODR |=  GPIO_ODR_3;	// ...
	
	// TODO: Set MTR2 Forward Direction Status
}

void MTR1_REV()
{
	GPIOB->ODR |=  GPIO_ODR_2;	// MTR1 REV
	GPIOB->ODR &= ~GPIO_ODR_3;	// ...
	
	// TODO: Set MTR1 Reverse Direction Status
}

void MTR2_FWD()
{
	GPIOB->ODR |=  GPIO_ODR_7;	// MTR2 FWD
	GPIOB->ODR &= ~GPIO_ODR_8;	// ...
	
	// TODO: Set MTR2 Forward Direction Status
}

void MTR2_REV()
{
	GPIOB->ODR &= ~GPIO_ODR_7;	// MTR2 REV
	GPIOB->ODR |=  GPIO_ODR_8;	// ...
	
	// TODO: Set MTR2 Reverse Direction Status
}

void MTR1_Stop()
{
	// Disable Motor
	MTR1_SetDuty(0);
	MTR1_CurrentSpeed = 0;
	
	// Open Relays
	GPIOB->ODR &= ~GPIO_ODR_2;	
	GPIOB->ODR &= ~GPIO_ODR_3;	
	
	// TODO: Reset MTR1_RUNNING Status
}

void MTR2_Stop()
{
	// Disable Motor
	MTR2_SetDuty(0);
	MTR2_CurrentSpeed = 0;
	
	// Open Relays
	GPIOB->ODR &= ~GPIO_ODR_7;	
	GPIOB->ODR &= ~GPIO_ODR_8;	
	
	// TODO: Reset MTR2_RUNNING Status
}

void MTR1_RampSpeed(int8_t speed)
{
	
	
	
	if (MTR1_CurrentSpeed < speed)
	{
			if (RampDelay1 == 0)
			{
				MTR1_CurrentSpeed ++;
				MTR1_SetDuty(MTR1_CurrentSpeed);
				RampDelay1 = 10;
			}
	}
}

void MTR2_RampSpeed(int8_t speed)
{
	if (MTR2_CurrentSpeed < speed)
	{
		if (RampDelay2 == 0)
		{
			MTR2_CurrentSpeed ++;
			MTR2_SetDuty(MTR2_CurrentSpeed);
			RampDelay2 = 10;
		}
	}
}





extern void LimitSwitch1_Update(void);
extern void LimitSwitch2_Update(void);

///////////////////////////////////


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
	
	/* Enable Encoders */
	encoder_init();
	
	/* Configure USART1 */
	UART_init();

	// Motor Forward
	
	
	TIM14->CCR1 	 = 0;
	TIM17->CCR1	 	 = 0;
  GPIOB->ODR &= ~GPIO_ODR_2;	// MTR1 Off
	GPIOB->ODR &= ~GPIO_ODR_3;	// ...
	GPIOB->ODR &= ~GPIO_ODR_7;	// MTR2 Off
	GPIOB->ODR &= ~GPIO_ODR_8;	// ...

	LimitSwitch1_Update();
	LimitSwitch2_Update();
		
		
//	uint32_t h1 = 0;
//	char from_int[30];
	
  while (1)
  {
			
		// Code Running Indicator
//		Blink();
				

		

		
//		if (TIM2->CNT > 0x7FFF) {GPIOC->ODR |=GPIO_ODR_8;} else {GPIOC->ODR &= ~GPIO_ODR_8;}
//		if (TIM2->CNT == 0x7FFF) {GPIOC->ODR |=GPIO_ODR_7;} else {GPIOC->ODR &= ~GPIO_ODR_7;}
//		
//		if (TIM3->CNT > 0x7FFF) {GPIOC->ODR |=GPIO_ODR_8;} else {GPIOC->ODR &= ~GPIO_ODR_8;}
//		if (TIM3->CNT == 0x7FFF) {GPIOC->ODR |=GPIO_ODR_7;} else {GPIOC->ODR &= ~GPIO_ODR_7;}
		
	//	HAL_Delay(1000);

		
	//	SpeedTest();
		
		//uint8_t duty = 80;
		
		//MTR1_SetDuty(duty);	
		//MTR2_SetDuty(duty);
		// Test Motor Speed
		//SpeedTest();
		
		
		
		//////////////////////////////////
		
		/* USER CODE END WHILE */
		// HAL_Delay(250);
		// sendStr("Doing other tasks...\n\r");
		
		

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
		
		//if (TIM3->CNT > 0x7FFF + Overtravel_SP) {GPIOC->ODR |= GPIO_ODR_8;} else {GPIOC->ODR &= ~GPIO_ODR_8;}
		//if (TIM3->CNT == 0x7FFF) {GPIOC->ODR |= GPIO_ODR_9;} else {GPIOC->ODR &= ~GPIO_ODR_9;}
		
		// Detect Overtravel
		if ((TIM3->CNT > 0x7FFF + Overtravel_SP) || (TIM2->CNT > 0x7FFF + Overtravel_SP)) {Overtravel = 1;} else {Overtravel = 0;}
		if (Overtravel == 1) {Buzzer_On();} else {Buzzer_Off();}
			
		// Update HMI Data
//		if (UpdateDelay == 0)
//		{
//			// h1 = TIM3->CNT;
//			//if (h1 > 100) {h1 = 100;} 
//			
//			sprintf(from_int,"%d",h1);
//			sendStr("h1.val=");
//			sendStr(from_int);
//			sendChar(0xFF);
//			sendChar(0xFF);
//			sendChar(0xFF);
//			
//			UpdateDelay = 500;
//		}
		
			  // 
		switch (current_state) {

			case UNCALIBRATED:
				if (command == DESK_HOME) {
					current_state = HOMING;
					// ENABLE WDTMTR
				}
				break;
			case HOMING:
				// Ramp up to Speed
				if (Prox1 == 1)
				{MTR1_RampSpeed(TurtleSpeed);}
				else
				{MTR1_SetDuty(0);}
				
				if (Prox2 == 1)
				{MTR2_RampSpeed(TurtleSpeed);}
				else
				{MTR2_SetDuty(0);}
				
				if ((Prox1 == 0) && (Prox2 == 0)) { // IF Calibration is done - do we need to make a calculation or set a variable on an interrupt?
					current_state = IDLE;
					// Reset Outputs
					MTR1_Stop();
					MTR2_Stop();
					
					// DISABLE WDTMTR
				} else if (0) { // WDTMTR_DN
					// FTH
				}
				break;
			case IDLE:
				
				switch (command) {
					case DESK_HOME:
						current_state = HOMING;
						MTR1_REV();
						MTR2_REV();
						// ENABLE WDTMTR
						break;
					case DESK_RAISE:
						//if (Overtravel == 0) 
						//{
							current_state = DESK_RAISING;
							MTR1_FWD();
							MTR2_FWD();
						//} 
						
						break;
					case DESK_LOWER:
						current_state = DESK_LOWERING;
						MTR1_REV();
						MTR2_REV();
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
							MTR1_FWD();
						break;
					case MTR1_LOWER:
						if (Prox1 == 1)
						{
							current_state = LEGL_LOWERING;
							MTR1_REV();
						}
						break;
					case MTR2_RAISE:
						current_state = LEGR_RAISING;
						MTR2_FWD();
						break;
					case MTR2_LOWER:
						if (Prox2 ==1)
						{
							current_state = LEGR_LOWERING;
							MTR2_REV();
						}
						break;
					default:
						break;
				}
				break;
			case DESK_LOWERING:
				// Ramp up to Speed
				if (Prox1 == 1)
				{MTR1_RampSpeed(RabbitSpeed);}
				else
				{MTR1_SetDuty(0);}
				
				if (Prox2 == 1)
				{MTR2_RampSpeed(RabbitSpeed);}
				else
				{MTR2_SetDuty(0);}
			
				if (command == DESK_STOP) 
				{
					current_state = IDLE;
					MTR1_Stop();
					MTR2_Stop();
				}
				break;
			case DESK_RAISING:
				// Ramp up to Speed
				MTR1_RampSpeed(RabbitSpeed);
				MTR2_RampSpeed(RabbitSpeed);
			
				if ((command == DESK_STOP))// || (Overtravel == 1)) 
				{
					current_state = IDLE;
					MTR1_Stop();
					MTR2_Stop();
				}
				break;
			case LEGL_LOWERING:
				if (Prox1 == 1)
				{MTR1_RampSpeed(TurtleSpeed);}
				else
				{MTR1_SetDuty(0);}
				
				if (command == DESK_STOP) 
				{
					current_state = IDLE;
					MTR1_Stop();
				}
				break;
			case LEGL_RAISING:
				MTR1_RampSpeed(TurtleSpeed);
				
				if (command == DESK_STOP) 
				{
					current_state = IDLE;
					MTR1_Stop();
				}
				break;
			case LEGR_LOWERING:
				if (Prox2 == 1)
				{MTR2_RampSpeed(TurtleSpeed);}
				else
				{MTR2_SetDuty(0);}
				
				if (command == DESK_STOP) 
				{
					current_state = IDLE;
					MTR2_Stop();
				}
				break;
			case LEGR_RAISING:
				{MTR2_RampSpeed(TurtleSpeed);}
				
				if (command == DESK_STOP) {
					current_state = IDLE;
					MTR2_Stop();
				}
				break;
			case TARGET_POSITIONING:
				if (1) { // Done Positioning
					current_state = IDLE;
					// DISABLE WDTMTR
				}
				break;
			default:
				// How did we get here? (No seriously, if we are here, something is wrong)
				break;
		}
  }
}




///////////////////////////////////////////////////////////////////////////////////////////////////






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

