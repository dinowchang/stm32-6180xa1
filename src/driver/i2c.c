/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Dinow
  * @version V0.0.1
  * @date    2015-12-03
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "assert.h"
#include "type.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"
#include "semphr.h"
#include "driver/i2c.h"
#include "util.h"

/* Private define ------------------------------------------------------------*/
#define I2C_MAX_TRANSFER_LENGTH					20
#define I2C_TIMEOUT								(20*portTICK_PERIOD_MS)

#define I2C_TRANSFER_OK							0
#define I2C_PARAMERET_ERROR						-1
#define I2C_PARAMERET_TIMEOUT					-2
#define I2C_ACK_FAIL							-3


/* Private typedef -----------------------------------------------------------*/
typedef struct {
	volatile uint8_t direction;					// Transmitter / Receiver
	uint8_t slaveAddr;							// 7 bits address only
	uint16_t regAddr; 							// 8 bits or 16 bits register address of slave device
	uint8_t regAddrLen;							// type of register address
	uint8_t data[I2C_MAX_TRANSFER_LENGTH];		// data buffer
	volatile uint8_t dataLen;					// Length of data
	volatile uint8_t *dataPtr;					// transfer position
	volatile uint8_t status;
	SemaphoreHandle_t lock;
	SemaphoreHandle_t finishSemaphore;
} i2cPacket_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static i2cPacket_t i2c1Handler;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void I2C1_EV_IRQHandler(void)
{
	if (i2c1Handler.direction == I2C_Direction_Transmitter)
	{
		if(I2C_GetITStatus(I2C1, I2C_IT_SB)== SET)
		{
			I2C_Send7bitAddress(I2C1, (i2c1Handler.slaveAddr << 1), I2C_Direction_Transmitter);
		}
		else if(I2C_GetITStatus(I2C1, I2C_IT_ADDR)== SET)
		{
			if (i2c1Handler.dataLen == 1)
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE); // wait BTF
			else
				I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE); // wait TXE

			// reading SR1 register followed reading SR2 to clear ADDR register
			(void) (I2C1->SR1);
			(void) (I2C1->SR2);
			I2C_SendData(I2C1, i2c1Handler.dataPtr[0]);
			i2c1Handler.dataPtr++;
			i2c1Handler.dataLen--;
		}
		else if((I2C_GetITStatus(I2C1, I2C_IT_TXE)== SET)&&(I2C_GetITStatus(I2C1, I2C_IT_BTF) == RESET))
		{
			if (i2c1Handler.dataLen == 1)
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE); // wait BTF

			I2C_SendData(I2C1, i2c1Handler.dataPtr[0]);
			i2c1Handler.dataPtr++;
			i2c1Handler.dataLen--;
		}
		else if (I2C_GetITStatus(I2C1, I2C_IT_BTF) == SET)
		{
			if (i2c1Handler.dataLen == 0)
			{
				I2C_GenerateSTOP(I2C1, ENABLE);
				I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
				xSemaphoreGiveFromISR(i2c1Handler.finishSemaphore, NULL);
			}
			else
			{
				I2C_SendData(I2C1, i2c1Handler.dataPtr[0]);
				i2c1Handler.dataPtr++;
				i2c1Handler.dataLen--;
			}
		}
	}
	else //i2c1Handler.direction == I2C_Direction_Receiver
	{
		if(I2C_GetITStatus(I2C1, I2C_IT_SB)== SET)
		{
			I2C_Send7bitAddress(I2C1, (i2c1Handler.slaveAddr << 1), I2C_Direction_Receiver);
			I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE); // wait RXNE
		}
		else if(I2C_GetITStatus(I2C1, I2C_IT_ADDR)== SET)
		{
			if (i2c1Handler.dataLen == 1)
				I2C_AcknowledgeConfig(I2C1, DISABLE);

			// reading SR1 register followed reading SR2 to clear ADDR register
			(void) (I2C1->SR1);
			(void) (I2C1->SR2);

			if (i2c1Handler.dataLen == 1)
			{
				// interrupt is enabled at I2C_IT_SB. wait RXNE
				I2C_GenerateSTOP(I2C1, ENABLE);
			}
		}
		else if ((I2C_GetITStatus(I2C1, I2C_IT_RXNE) == SET) && (I2C_GetITStatus(I2C1, I2C_IT_BTF) == RESET))
		{
			i2c1Handler.dataPtr[0] = I2C_ReceiveData(I2C1);
			i2c1Handler.dataPtr++;
			i2c1Handler.dataLen--;

			if (i2c1Handler.dataLen == 0x01)
			{
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);
			}

			if (i2c1Handler.dataLen == 0x00)
			{
				I2C_ITConfig(I2C1, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				xSemaphoreGiveFromISR(i2c1Handler.finishSemaphore, NULL);
			}
		}
	}

	if( I2C_GetITStatus(I2C1, I2C_IT_AF) )
	{
		I2C_GenerateSTOP(I2C1, ENABLE);
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		I2C_ITConfig(I2C1, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
		i2c1Handler.status = I2C_ACK_FAIL;
		xSemaphoreGiveFromISR(i2c1Handler.finishSemaphore, NULL);
	}

}

/**
 * @brief Write data to I2C1
 * @param slaveAddr		support 7 bit only
 * @param regAddr		device internal register address
 * @param regAddrLen	8 bits or 16 bits, depend on device
 * @param data
 * @param length
 * @return status
 */
int32_t I2C1_Write(uint8_t slaveAddr, uint16_t regAddr, uint8_t regAddrLen, uint8_t *data, uint8_t length)
{
	xSemaphoreTake(i2c1Handler.lock, portMAX_DELAY );

	if ((regAddrLen + length) > I2C_MAX_TRANSFER_LENGTH) return I2C_PARAMERET_ERROR;

	if( regAddrLen == 2 )
	{
		i2c1Handler.data[0] = (regAddr >> 8);
		i2c1Handler.data[1] = (regAddr & 0xff);
	}
	else if( regAddrLen == 1 )
	{
		i2c1Handler.data[0] = (regAddr & 0xff);
	}
	else
	{
		xSemaphoreGive(i2c1Handler.lock);
		return I2C_PARAMERET_ERROR;
	}

	memcpy( &i2c1Handler.data[regAddrLen], data, length);
	i2c1Handler.dataLen = regAddrLen + length;
	i2c1Handler.dataPtr = i2c1Handler.data;
	i2c1Handler.slaveAddr = slaveAddr;
	i2c1Handler.direction = I2C_Direction_Transmitter;
	i2c1Handler.status = I2C_TRANSFER_OK;
    /* While the bus is busy */
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);	// START condition

	if ( xSemaphoreTake(i2c1Handler.finishSemaphore, I2C_TIMEOUT) != pdTRUE)
		i2c1Handler.status = I2C_PARAMERET_TIMEOUT;

	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);

	xSemaphoreGive(i2c1Handler.lock);

    return i2c1Handler.status;

}

/**
 * @brief Read data from I2C1
 * @param slaveAddr		support 7 bit only
 * @param regAddr		device internal register address
 * @param regAddrLen	8 bits or 16 bits, depend on device
 * @param data
 * @param length
 * @return status
 */
int32_t I2C1_Read(uint8_t slaveAddr, uint16_t regAddr, uint8_t regAddrLen, uint8_t *data, uint8_t length)
{
	xSemaphoreTake(i2c1Handler.lock, portMAX_DELAY );

	if (regAddrLen > I2C_MAX_TRANSFER_LENGTH || length > I2C_MAX_TRANSFER_LENGTH ) return I2C_PARAMERET_ERROR;

	if( regAddrLen == 2 )
	{
		i2c1Handler.data[0] = (regAddr >> 8);
		i2c1Handler.data[1] = (regAddr & 0xff);
	}
	else if( regAddrLen == 1 )
	{
		i2c1Handler.data[0] = (regAddr & 0xff);
	}
	else
	{
		xSemaphoreGive(i2c1Handler.lock);
		return I2C_PARAMERET_ERROR;
	}


	// Send register address first
	i2c1Handler.dataLen = regAddrLen;
	i2c1Handler.dataPtr = i2c1Handler.data;
	i2c1Handler.slaveAddr = slaveAddr;
	i2c1Handler.direction = I2C_Direction_Transmitter;
	i2c1Handler.status = I2C_TRANSFER_OK;

    /* While the bus is busy */
    //while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);	// START condition

	if ( xSemaphoreTake(i2c1Handler.finishSemaphore, I2C_TIMEOUT) != pdTRUE)
	{
		i2c1Handler.status = I2C_PARAMERET_TIMEOUT;
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
		xSemaphoreGive(i2c1Handler.lock);
		return i2c1Handler.status;
	}
	else if ( i2c1Handler.status != I2C_TRANSFER_OK )
	{
	    xSemaphoreGive(i2c1Handler.lock);
		return i2c1Handler.status;
    }

	i2c1Handler.dataLen = length;
	i2c1Handler.dataPtr = i2c1Handler.data;
	i2c1Handler.direction = I2C_Direction_Receiver;
	i2c1Handler.status = I2C_TRANSFER_OK;

	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);	// START condition

	if ( xSemaphoreTake(i2c1Handler.finishSemaphore, I2C_TIMEOUT) != pdTRUE)
	{
		i2c1Handler.status = I2C_PARAMERET_TIMEOUT;
	}
	else if ( i2c1Handler.status != I2C_TRANSFER_OK )
	{
	    xSemaphoreGive(i2c1Handler.lock);
		return i2c1Handler.status;
    }
	else
	{
		memcpy(data, i2c1Handler.data, length);
	}

	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);

	/* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C1, ENABLE);

	xSemaphoreGive(i2c1Handler.lock);

	return i2c1Handler.status;

}

/**
 * @brief CLI I2C1 Read function
 * @param pcWriteBuffer
 * @param xWriteBufferLen
 * @param pcCommandString
 * @return
 */
static BaseType_t I2C1_ReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *parameterPtr;
	int32_t paramterLen;

	int32_t ret;
	uint32_t slaveAddr, regAddr, data = 0;
	uint16_t regAddrLen, dataLen;

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		slaveAddr = HexToInt((char *) parameterPtr, paramterLen);
	}
	else
	{
		slaveAddr = DecToInt((char *) parameterPtr, paramterLen);
	}

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 2, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		regAddr = HexToInt((char *) parameterPtr, paramterLen);
		regAddrLen = (paramterLen - 2) / 2;
	}
	else
	{
		regAddr = DecToInt((char *) parameterPtr, paramterLen);
		regAddrLen = (regAddr >= 256) ? 2 : 1;
	}

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 3, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		dataLen = HexToInt((char *) parameterPtr, paramterLen);
	}
	else
	{
		dataLen = DecToInt((char *) parameterPtr, paramterLen);
	}

	dataLen = (dataLen > 4) ? 4 : dataLen;
	dataLen = (dataLen < 1) ? 1 : dataLen;

	ret = I2C1_Read(slaveAddr, regAddr, regAddrLen, (uint8_t *)(&data), dataLen);

	if( ret == I2C_TRANSFER_OK )
		sprintf(pcWriteBuffer, "Device:0x%04lx Reg:0x%04lx = %08lx\n", slaveAddr, regAddr, data);
	else
	{
		I2C_ClearFlag(I2C1, I2C_FLAG_AF);
		sprintf(pcWriteBuffer, "I2C1 Error %ld!!!!\n", ret);
	}



	return pdFALSE;
}

static const CLI_Command_Definition_t m_I2c1Read =
{
	"i2cr", /* The command string to type. */
	"i2cr [slave address] [register address] [data length]\n\tRead data from I2C1. Max data length is 4 bytes.\n",
	I2C1_ReadCommand, /* The function to run. */
	3 /* No parameters are expected. */
};

/**
 * @brief CLI I2C1 Write function
 * @param pcWriteBuffer
 * @param xWriteBufferLen
 * @param pcCommandString
 * @return
 */
static BaseType_t I2C1_WriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *parameterPtr;
	int32_t paramterLen;

	int32_t ret;
	uint32_t slaveAddr, regAddr, data;
	uint16_t regAddrLen, dataLen;

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		slaveAddr = HexToInt((char *) parameterPtr, paramterLen);
	}
	else
	{
		slaveAddr = DecToInt((char *) parameterPtr, paramterLen);
	}

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 2, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		regAddr = HexToInt((char *) parameterPtr, paramterLen);
		regAddrLen = (paramterLen - 2) / 2;
	}
	else
	{
		regAddr = DecToInt((char *) parameterPtr, paramterLen);
		regAddrLen = (regAddr >= 256) ? 2 : 1;
	}

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 3, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		data = HexToInt((char *) parameterPtr, paramterLen);
		dataLen = (paramterLen - 2) / 2;
		dataLen = (dataLen > 4) ? 4 : dataLen;
		dataLen = (dataLen < 1) ? 1 : dataLen;
	}
	else
	{
		data = DecToInt((char *) parameterPtr, paramterLen);

		if ( data <= 0x000000ff) dataLen = 1;
		else if (( data <= 0x0000ffff)) dataLen = 2;
		else if (( data <= 0x00ffffff)) dataLen = 3;
		else dataLen = 4;
	}

	ret = I2C1_Write(slaveAddr, regAddr, regAddrLen, (uint8_t *)(&data), regAddrLen);

	if( ret == I2C_TRANSFER_OK )
		sprintf(pcWriteBuffer, "Device:0x%02lx Reg:0x%04lx = %08lx\n", slaveAddr, regAddr, data);
	else
	{
		I2C_ClearFlag(I2C1, I2C_FLAG_AF);
		sprintf(pcWriteBuffer, "I2C1 Error %ld!!!!\n", ret);
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t m_I2c1Write =
{
	"i2cw", /* The command string to type. */
	"i2cw [slave address] [register address] [data]\n\tWrite data to I2C1. Max data length is 4 bytes.\n",
	I2C1_WriteCommand, /* The function to run. */
	3 /* No parameters are expected. */
};

/**
 * @brief  Initialize I2C1 pins and parameters
 */
void I2C1_Init(void)
{
	i2c1Handler.lock = xSemaphoreCreateMutex();
	i2c1Handler.finishSemaphore = xSemaphoreCreateBinary();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Configure pins PB8 and PB9 for I2C1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	FreeRTOS_CLIRegisterCommand( &m_I2c1Read );
	FreeRTOS_CLIRegisterCommand( &m_I2c1Write );
}
