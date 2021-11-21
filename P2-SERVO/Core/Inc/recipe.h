/*
 * recipe.h
 *
 *  Created on: Sep 27, 2021
 *      Author: abaji
 */

#ifndef INC_RECIPE_H_
#define INC_RECIPE_H_

#include "common.h"
#include "servo.h"
#include "MFS.h"

typedef enum {
	MOV = 0x20,
	WAIT = 0x40,
	LOOP = 0x80,
	END_LOOP = 0xA0,
	RECIPE_END = 0x00
} recipeCommand;

// Possible statuses of a recipe's operation.
typedef enum {
	PAUSED,
	RUNNING,
	COMMAND_ERROR,
	NESTED_ERROR
} recipeStatus;

// A recipe's state contains its instruction set, status, servo state, and supporting variables.
typedef struct {
	const unsigned char *recipe;// Pointer to recipe instruction set
	uint8_t id;					// Recipe ID used for status printouts.
	uint8_t index;              // Current recipe instruction index
	uint8_t stall_count;   		// Execution stall counter (for WAIT and MOV commands)
	uint8_t loop_start;   		// Index at the start of a loop
	uint8_t loop_count;   		// Remaining loop iteration count
	servoState *servo_state;	// Pointer to state of servo
	recipeStatus status;		// Status of this recipe state
} recipeState;

void Recipe_Init(recipeState *recipe_state, const unsigned char *recipe, uint8_t id, servoState *servo_state);
void Recipe_Restart(recipeState *recipe_state);
void Recipe_Update_Status(recipeState *recipe_state, recipeStatus status);
void Recipe_Process_Next(recipeState *recipe_state);
void Recipe_Process_CMD(recipeState *recipe_state, recipeCommand cmd, uint8_t param);

#endif /* INC_RECIPE_H_ */
