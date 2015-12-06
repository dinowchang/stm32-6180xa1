/**
  ******************************************************************************
  * @file    util.c
  * @author  Dinow
  * @version V0.0.1
  * @date    2015-12-06
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
#include "type.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 * @brief convert hex string to integer
 * @param hexadecimal string
 * @param length of string
 * @return integer value
 */
uint32_t HexToInt(char *str, uint16_t len)
{
	uint32_t ret = 0;
	while(len > 0)
	{
		if( '0' <= *str && *str <= '9')
		{
			ret = ret * 16 + *str - '0';
		}
		else if ( 'a' <= *str && *str <= 'f')
		{
			ret = ret * 16 + *str - 'a' + 10;
		}
		else if ( 'A' <= *str && *str <= 'F')
		{
			ret = ret * 16 + *str - 'A' + 10;
		}
		str++;
		len--;
	}
	return ret;
}

/**
 * @brief convert dec string to integer
 * @param decimal string
 * @param length of string
 * @return integer value
 */
uint32_t DecToInt(char *str, uint16_t len)
{
	uint32_t ret = 0;
	while(len > 0)
	{
		if( '0' <= *str && *str <= '9')
		{
			ret = ret * 10 + *str - '0';
		}
		str++;
		len--;
	}
	return ret;
}
