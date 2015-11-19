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
#include "debug.h"
#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TEST_USART_RX_ECHO		0
#define DEBUG_BUAD_RATE			115200
#define DEBUG_PORT				DEBUG_PORT_USART2	///<  DEBUG_PORT could be set either to DEBUG_PORT_USART2 or DEBUG_PORT_USART3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize USART2 pins and parameters
 */
void DEBUG_Init(void)
{
#if DEBUG_PORT == DEBUG_PORT_USART2
	// Enable clock of GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Configure pins PA2 and PA3 for USART2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
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

void DEBUG_SendData(uint16_t Data)
{
#if DEBUG_PORT == DEBUG_PORT_USART2
	USART_SendData(USART2, Data);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
#elif DEBUG_PORT == DEBUG_PORT_USART3
	USART_SendData(USART3, Data);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
#endif
}


#if DEBUG_PORT == DEBUG_PORT_USART2
/**
 * @brief  USART2 interrupt handler
 */
void USART2_IRQHandler(void)
{
	while (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
#if TEST_USART_RX_ECHO
		char data = USART_ReceiveData(USART2);
		USART_SendData(USART2, data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
#else
		USART_ReceiveData(USART2);
#endif
	}
}

#elif DEBUG_PORT == DEBUG_PORT_USART3
/**
 * @brief  USART3 interrupt handler
 */
void USART3_IRQHandler(void)
{
	while (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
#if TEST_USART_RX_ECHO
		char data = USART_ReceiveData(USART3);
		USART_SendData(USART3, data);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
#else
		USART_ReceiveData(USART3);
#endif
	}
}
#endif
