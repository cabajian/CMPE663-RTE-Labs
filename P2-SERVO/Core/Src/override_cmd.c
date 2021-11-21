/*
 * override_cmd.c
 *
 *  Created on: Sep 27, 2021
 *      Author: abaji
 */

// Includes.
#include "override_cmd.h"

// Static ORCMD-specific variables.
static ORCMD_Status status = INCOMPLETE;
static uint8_t idx = 0;
static uint8_t cmd[ORCMD_MAX_LEN] = {0};
static ORCMD_Function functions[ORCMD_MAX_LEN] = {NOOP};

/*
 * Handler to poll specified UART for a command, process completed commands,
 * and return a pointer to the decoded command functions.
 */
ORCMD_Function* ORCMD_Get_Functions(UART_HandleTypeDef *huart) {
	ORCMD_Poll(huart);
	return ORCMD_Process(huart);
}

/*
 * Non-blocking poll of the specified UART module for an incoming command character.
 */
void ORCMD_Poll(UART_HandleTypeDef *huart) {
	// Non-blocking UART poll.
	if (huart->Instance->ISR & USART_ISR_RXNE) {
		// Fetch the characters.
		uint8_t c = (uint8_t)huart->Instance->RDR;
		// <CR> is the escape/complete character.
		if (c == '\r') {
			status = (idx < (ORCMD_MAX_LEN-1)) ? ESCAPE : COMPLETE;
		// 'x' is an escape character.
		} else if (c == 'X' || c == 'x') {
			status = ESCAPE;
			HAL_UART_Transmit(huart, &c, 1, UART_TIMEOUT_MS);
		// Add other characters to the command buffer.
		} else if (idx < ORCMD_MAX_LEN) {
			cmd[idx++] = c;
			HAL_UART_Transmit(huart, &c, 1, UART_TIMEOUT_MS);
		}
	}
}

/*
 * Process a command if completed or escaped.
 */
ORCMD_Function* ORCMD_Process() {
	// Set default functions to NOOP.
	for (int i = 0; i < ORCMD_MAX_LEN; i++) {
		functions[i] = NOOP;
	}
	// Only process functions when complete.
	if (status == COMPLETE) {
		idx = 0;
		print("\r\n> ");
		status = INCOMPLETE;
		// Process each command and assign to a function.
		for (int i = 0; i < ORCMD_MAX_LEN; i++) {
			switch (cmd[i]) {
				case 'P':
				case 'p':
					functions[i] = PAUSE;
					break;
				case 'C':
				case 'c':
					functions[i] = CONTINUE;
					break;
				case 'R':
				case 'r':
					functions[i] = MOVER;
					break;
				case 'L':
				case 'l':
					functions[i] = MOVEL;
					break;
				case 'B':
				case 'b':
					functions[i] = BEGIN;
					break;
				default:
					functions[i] = NOOP;
					break;
			}
		}
	} else {
		// Update status if escape code entered.
		if (status == ESCAPE) {
			idx = 0;
			print("\r\n> ");
			status = INCOMPLETE;
		}
	}
	return functions;
}
