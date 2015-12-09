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

#define I2C_TRANSFER_OK							0
#define I2C_PARAMERET_ERROR						-1

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint8_t direction;							// Transmitter / Receiver
	uint8_t slaveAddr;							// 7 bits address only
	uint16_t regAddr; 							// 8 bits or 16 bits register address of slave device
	uint8_t regAddrLen;							// type of register address
	uint8_t data[I2C_MAX_TRANSFER_LENGTH];		// data buffer
	uint8_t dataLen;							// Length of data
	uint8_t *dataPtr;							// transfer position
	uint8_t status;
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
			I2C_Send7bitAddress(I2C1, i2c1Handler.slaveAddr, I2C_Direction_Transmitter);
		}
		else if(I2C_GetITStatus(I2C1, I2C_IT_ADDR)== SET)
		{
			if (i2c1Handler.dataLen == 1)
			{
				// wait BTF
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
			}
			else
			{
				// wait TXE
				I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
			}
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
			{
				// wait BTF
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
			}
			I2C_SendData(I2C1, i2c1Handler.dataPtr[0]);
			i2c1Handler.dataPtr++;
			i2c1Handler.dataLen--;
		}
		else if (I2C_GetITStatus(I2C1, I2C_IT_BTF) == SET)
		{
			if (i2c1Handler.dataLen == 0)
			{
				I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);
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
    // ENTR_CRT_SECTION();
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
		return I2C_PARAMERET_ERROR;
	}

	memcpy( &i2c1Handler.data[regAddrLen], data, length);
	i2c1Handler.dataLen = regAddrLen + length;
	i2c1Handler.dataPtr = i2c1Handler.data;
	i2c1Handler.slaveAddr = slaveAddr;
	i2c1Handler.direction = I2C_Direction_Transmitter;

    /* While the bus is busy */
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);	// START condition

	xSemaphoreTake(i2c1Handler.finishSemaphore, 10);

	xSemaphoreGive(i2c1Handler.lock);

    return I2C_TRANSFER_OK;

}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
void I2C1_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
    // ENTR_CRT_SECTION();

	I2C_ITConfig(I2C1, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    /* While the bus is busy */
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send slave address for write */
    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(I2C1, ENABLE);

    /* Send the register address */
	I2C_SendData(I2C1, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2C1, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(I2C1, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU6050 */
            *pBuffer = I2C_ReceiveData(I2C1);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    // EXT_CRT_SECTION();
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
	uint32_t slaveAddr, regAddr;

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
	}
	else
	{
		regAddr = DecToInt((char *) parameterPtr, paramterLen);
	}

	uint8_t data[2];
	I2C1_BufferRead(slaveAddr * 2, data, regAddr, 2);

	sprintf(pcWriteBuffer, "Device:0x%04lx Reg:0x%04lx = %02x %02x\n", slaveAddr, regAddr, data[0], data[1]);
	return pdFALSE;
}

static const CLI_Command_Definition_t m_I2c1Read =
{
	"i2cr", /* The command string to type. */
	"i2cr [slave address] [register address]\n\tRead data from I2C1\n",
	I2C1_ReadCommand, /* The function to run. */
	2 /* No parameters are expected. */
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
	uint32_t slaveAddr, regAddr;
	uint16_t data;

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
	}
	else
	{
		regAddr = DecToInt((char *) parameterPtr, paramterLen);
	}

	parameterPtr = FreeRTOS_CLIGetParameter(pcCommandString, 3, &paramterLen);
	if (paramterLen > 3 && parameterPtr[0] == '0' && parameterPtr[1] == 'x')
	{
		data = HexToInt((char *) parameterPtr, paramterLen);
	}
	else
	{
		data = DecToInt((char *) parameterPtr, paramterLen);
	}

	I2C1_Write(slaveAddr * 2, regAddr, 1, (uint8_t *)(&data), 2);

	sprintf(pcWriteBuffer, "Device:0x%02lx Reg:0x%04lx = %02x\n", slaveAddr, regAddr, data);
	return pdFALSE;
}

static const CLI_Command_Definition_t m_I2c1Write =
{
	"i2cw", /* The command string to type. */
	"i2cw [slave address] [register address] [data]\n\tWrite data to I2C1\n",
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
