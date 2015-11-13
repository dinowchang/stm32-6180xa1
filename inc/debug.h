/**
  ******************************************************************************
  * @file    debug.h
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

#ifndef __DEBUG_H_
#define __DEBUG_H_

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#ifdef DEBUG
#define DEBUG_printf(CONDITION, args...)	do									\
									{											\
										if(CONDITION)	printf(args);			\
									}while(0)
#else
#define DEBUG_printf(CONDITION, args...)	((void)0)
#endif

/* Exported functions ------------------------------------------------------- */
void DEBUG_Init(void);

#endif /* __DEBUG_H_ */
