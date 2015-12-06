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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
void I2C1_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint16_t readAddr, uint16_t NumByteToRead)
{
    // ENTR_CRT_SECTION();

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

    /* Send the index[8:15] */
	I2C_SendData(I2C1, (u8) (readAddr >> 8));

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the index[0:7] */
	I2C_SendData(I2C1, (u8) (readAddr & 0xff));

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
 * @brief Displays a table showing the state of each FreeRTOS task
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

	uint8_t readBuf[16];
	I2C1_BufferRead(slaveAddr * 2, readBuf, regAddr, 1);

	sprintf(pcWriteBuffer, "Device:0x%04lx Reg:0x%04lx = %02x\n", slaveAddr, regAddr, readBuf[0]);
	return pdFALSE;
}

static const CLI_Command_Definition_t m_I2c1Read =
{
	"i2cr", /* The command string to type. */
	"\r\ni2cr [slave address] [register address]\r\n Read data from I2C1\r\n",
	I2C1_ReadCommand, /* The function to run. */
	2 /* No parameters are expected. */
};

/**
 * @brief  Initialize I2C1 pins and parameters
 */
void I2C1_Init(void)
{

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




	FreeRTOS_CLIRegisterCommand( &m_I2c1Read );
}
