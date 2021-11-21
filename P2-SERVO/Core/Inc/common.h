/*
 * uart.h
 *
 *  Created on: Sep 27, 2021
 *      Author: abaji
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"

#define NUM_CONC_RECIPES		(2)

// USART2 printing macros.
#define UART_TIMEOUT_MS			(100)

void PWM_Set_Duty(TIM_TypeDef *htim, uint32_t channel, float duty_percent);
void print(char *str);
void println(char *str);

#endif /* INC_COMMON_H_ */
