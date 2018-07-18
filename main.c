#include <stdint.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx_hal.h"


#define  __setbit(___reg, ___bit)      	((___reg) |= (1U << (___bit)))
#define  __clearbit(___reg, ___bit)    	((___reg) &= (~(1U << (___bit))))
#define  __togglebit(___reg, ___bit)   	((___reg) ^= (1U << (___bit)))
#define  __getbit(___reg, ___bit)      	(((___reg) & (1U << (___bit))) >> (___bit))


QueueHandle_t qHandle;

// string literals
const char *t1_Msg = "Message received From Button ISR...";

// Task-1
void vRxTask(void *pvParams);
// Take-Check
void checkTask(void *pvParams);

// Configure pin for User-Btn
static void configInputPin(void);

// Configure pin select for interrupt
static void configInterrupt0(void);

// Configure clocks for LEDs
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


int main(void) 
{
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	
	// btn
	configInputPin();

	// EXTI0 interrupt
	configInterrupt0();

	// check if task queue is working
	xTaskCreate(checkTask, "checkTask", 50, NULL, 1, NULL);
	
	qHandle = xQueueCreate(2, sizeof(char *));

	if(qHandle != NULL) 
	{    
		xTaskCreate(vRxTask, "RxTask", 200, NULL, 2, NULL);
		vTaskStartScheduler();      
	} 
	else {    
		printf ("Failed to create Queue! :-(\n");
		while (1);
	}
}


/*
		Configuration for Input pins and
		NVIC interrupt
*/

static void configInputPin(void) 
{
	// enable clock to GPIOA
	__setbit(RCC->AHB1ENR, 0);
				
	// configure PA.0 as digital input
	__clearbit(GPIOA->MODER, 0);
	__clearbit(GPIOA->MODER, 1);
		
	// GPIOs output type: Push-Pull
	__clearbit(GPIOA->OTYPER, 0);
		
	// GPIO speed: Medium
	__setbit(GPIOA->OSPEEDR, 0);
	__clearbit(GPIOA->OSPEEDR, 1);
		
	// Initial Level: Logic High --> No Pull-up/down
	__clearbit(GPIOA->PUPDR, 0);
	__setbit(GPIOA->PUPDR, 1);  
}

static void configInterrupt0(void) 
{
	// enable clock to syscfg for pin selection.
	__setbit(RCC->APB2ENR, 14);
		
	// connect PA.0 to EXTI.0
	__clearbit(SYSCFG->EXTICR[0],0);
	__clearbit(SYSCFG->EXTICR[0],1);
	__clearbit(SYSCFG->EXTICR[0],2);
	__clearbit(SYSCFG->EXTICR[0],3);
		
	// Enable Interrupt on EXTI0 line
	__setbit(EXTI->IMR, 0);
		
	// trigger interrupt on rising edge
	__setbit(EXTI->RTSR, 0);

	NVIC_SetPriority(EXTI0_IRQn, 6U);

	// enable IRQn.6 to accept interrupt
	NVIC_EnableIRQ(EXTI0_IRQn);
}


/*
	Define RTOS queue'd tasks
*/

void vRxTask(void *pvParams) 
{
	volatile unsigned int i = 0;
	char * msgPtr;
	int rxStatus = 0;

	for(;;) 
	{
		rxStatus = xQueueReceive(qHandle, &msgPtr, 500);

		if(0 == rxStatus) 
		{
			printf ("Awaiting Message...\n");

			/*Configure GPIO pin Output Level */
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
		} 
		else {
			printf ("Rx Msg: %s\n", msgPtr);   

			/*Configure GPIO pin Output Level */
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
			
			// delay before resetting tasks
			for(i; i < 3999990; i++);
			NVIC_SystemReset();
		}
	}
}

void checkTask(void *pvParams) 
{
	volatile unsigned int i = 0;
	
	for(;;) 
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		for(i; i < 100000; i++);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		for(i; i < 100000; i++);
	}
}


/*
	Setup Clock and GPIO ports
*/

void SystemClock_Config(void) 
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	// Configure the main internal regulator output voltage
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	// Initializes the CPU, AHB and APB busses clocks 
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	// Initializes the CPU, AHB and APB busses clocks 
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	// Configure the Systick interrupt time
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	// Configure the Systick
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// SysTick_IRQn interrupt configuration 
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStruct;

	// GPIO Ports Clock Enable 
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// Configure GPIO pins : PD12 PD13 PD14 PD15
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


/*
	Interrupt Handlers
*/

// Exit Handlr
void EXTI0_IRQHandler(void) 
{
	int txStatus = 0;
	BaseType_t xHigherPriorityTaskWoken;

	// Clear the pending interrupt
	__setbit (EXTI->PR, 0);

	printf("Button Pressed, Sending message to Queue!\n");

	txStatus = xQueueSendToBackFromISR(qHandle, &t1_Msg, &xHigherPriorityTaskWoken);

	if(0 == txStatus) 
	{   
		printf("Sending failed Task-1!\n");
		// reset MCU if task 1 failed
		NVIC_SystemReset();
	} 
	else {
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}


