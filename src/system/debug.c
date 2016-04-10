/**
  ******************************************************************************
  * @file    debug.c
  * @author  Dinow
  * @version V0.0.1
  * @date    2015-11-15
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "assert.h"
#include "type.h"
#include "debug.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define DEBUG_BUAD_RATE			115200

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t xPrintMutex;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize USART2 pins and parameters
 */
void DEBUG_Init(void)
{
	xPrintMutex = xSemaphoreCreateMutex();

	assert(xPrintMutex != NULL);

#if DEBUG_PORT == DEBUG_PORT_USART2
	// Enable clock of GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Configure pins PA2 and PA3 for USART2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// Enable clock of USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// Configure USART2
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = DEBUG_BUAD_RATE;
	USART_Init(USART2, &USART_InitStructure);

	// Configure USART2 interrupt
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART2_IRQn);

	USART_Cmd(USART2, ENABLE);

#elif DEBUG_PORT == DEBUG_PORT_USART3
	// Enable clock of GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Configure pins PC10 and PC11 for USART2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);

	// Enable clock of USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// Configure USART2
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = DEBUG_BUAD_RATE;
	USART_Init(USART3, &USART_InitStructure);

	// Configure USART2 interrupt
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);

	USART_Cmd(USART3, ENABLE);

#endif

}

/**
 * @brief Send data to debug port
 * @param Data
 */
void DEBUG_SendData(uint16_t Data)
{
	if (Data == '\r')	return;

#if DEBUG_PORT == DEBUG_PORT_USART2
	if (Data == '\n')
	{
		USART_SendData(USART2, '\r');
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}
	USART_SendData(USART2, Data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
#elif DEBUG_PORT == DEBUG_PORT_USART3
	if (Data == '\n')
	{
		USART_SendData(USART3, '\r');
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
	USART_SendData(USART3, Data);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
#endif
}
/**
 *
 * @param file
 * @param line
 * @param func
 * @param expression
 */
void __assert_func(const char *file, int line, const char *func, const char *expression)
{
	taskDISABLE_INTERRUPTS();
	while (1)
	{
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
