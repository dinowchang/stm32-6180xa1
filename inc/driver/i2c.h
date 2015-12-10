/**
  ******************************************************************************
  * @file    i2c.h
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

#ifndef __INC_DRIVER_I2C_H_
#define __INC_DRIVER_I2C_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void I2C1_Init(void);
int32_t I2C1_Read(uint8_t slaveAddr, uint16_t regAddr, uint8_t regAddrLen, uint8_t *data, uint8_t length);
int32_t I2C1_Write(uint8_t slaveAddr, uint16_t regAddr, uint8_t regAddrLen, uint8_t *data, uint8_t length);

#endif /* __INC_DRIVER_I2C_H_ */
