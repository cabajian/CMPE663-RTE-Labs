/*
 * override_cmd.h
 *
 *  Created on: Sep 27, 2021
 *      Author: abaji
 */

#ifndef INC_OVERRIDE_CMD_H_
#define INC_OVERRIDE_CMD_H_

#include "stm32l4xx_hal.h"
#include "common.h"

// Maximum command length.
#define ORCMD_MAX_LEN		(NUM_CONC_RECIPES)

// Override Command Statuses.
typedef enum
{
  INCOMPLETE,
  ESCAPE,
  COMPLETE
} ORCMD_Status;

// Override Command Functions.
typedef enum
{
  NOOP			= 0x00,
  PAUSE			= 0x01,
  CONTINUE    	= 0x02,
  MOVER			= 0x03,
  MOVEL			= 0x04,
  BEGIN			= 0x05
} ORCMD_Function;

ORCMD_Function* ORCMD_Get_Functions(UART_HandleTypeDef *huart);
void ORCMD_Poll(UART_HandleTypeDef *huart);
ORCMD_Function* ORCMD_Process();

#endif /* INC_OVERRIDE_CMD_H_ */
