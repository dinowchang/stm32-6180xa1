/**
  ******************************************************************************
  * @file    command.c
  * @author  Dinow
  * @version V0.0.1
  * @date    2015-11-26
  * @brief   command console
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "config.h"
#include "assert.h"
#include "type.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "command.h"
#include "debug.h"
#include "debug.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COMM_TASK_PRIORITY					( tskIDLE_PRIORITY + 2UL )
#define COMM_TASK_STACK						( 2048/4 ) 							// 2048 bytes
#define COMM_QUEUE_LENGTH					16
#define MAX_INPUT_SIZE						64

#define DBG_COMM							TRUE

#define CHAR_ASCII_DEL						( 0x7F ) 							// DEL acts as a backspace.

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
QueueHandle_t xCommandQueue;
static const char * const pcWelcomeMessage = "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n\r\n>";
static const char * const pcEndOfOutputMessage = "\r\n[Press ENTER to execute the previous command again]\r\n>";


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Command console task
 * @param pvParameters
 */
static void COMM_Task( void *pvParameters )
{
	char data;
	uint8_t inputIndex = 0;
	static char inputString[MAX_INPUT_SIZE], lastInputString[MAX_INPUT_SIZE];
	char *outputString;
	BaseType_t ret;

	outputString = FreeRTOS_CLIGetOutputBuffer();
	DEBUG_printf(DBG_COMM, "%s", pcWelcomeMessage);
	fflush(stdout);

	while (1)
	{
		xQueueReceive(xCommandQueue, &data, portMAX_DELAY);

		if (data == '\r')
		{
			DEBUG_printf(DBG_COMM, "\n");

			// See if the command is empty, indicating that the last command is to be executed again.
			if (inputIndex == 0)
			{
				strcpy(inputString, lastInputString);
			}

			do
			{
				// Get the next output string from the command interpreter.
				ret = FreeRTOS_CLIProcessCommand(inputString, outputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);

				// Write the generated string to the UART.
				DEBUG_printf(DBG_COMM, "%s\n", outputString);

			} while (ret != pdFALSE);

			strcpy(lastInputString, inputString);
			inputIndex = 0;
			memset(inputString, 0x00, MAX_INPUT_SIZE);

			DEBUG_printf(DBG_COMM, "%s", pcEndOfOutputMessage);
			fflush(stdout);

		}
		else if ((data == '\b') || (data == CHAR_ASCII_DEL))
		{
			// Backspace was pressed.  Erase the last character in the string - if any
			if (inputIndex > 0)
			{
				inputIndex--;
				inputString[inputIndex] = '\0';
				DEBUG_printf(DBG_COMM, "\b \b");
				fflush(stdout);
			}
		}
		else if ((data >= ' ') && (data <= '~'))
		{
			if (inputIndex < MAX_INPUT_SIZE)
			{
				inputString[inputIndex] = data;
				inputIndex++;
				DEBUG_printf(DBG_COMM, "%c", data);
				fflush(stdout);
			}
		}
		else
		{
			// Ignore other character
		}

	}
}


/**
 * @brief Initialize and create command task
 */
void COMM_Init(void)
{

	xCommandQueue = xQueueCreate(COMM_QUEUE_LENGTH, sizeof(char));

	//assert(xCommandQueue != NULL);

	// Create that task that handles the console itself.
	xTaskCreate( 	COMM_Task,					/* The task that implements the command console. */
					"COMM",						/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					COMM_TASK_STACK,			/* The size of the stack allocated to the task. */
					NULL,						/* The parameter is not used, so NULL is passed. */
					COMM_TASK_PRIORITY,			/* The priority allocated to the task. */
					NULL );						/* A handle is not required, so just pass NULL. */
}
