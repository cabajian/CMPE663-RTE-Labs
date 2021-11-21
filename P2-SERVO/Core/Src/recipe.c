/*
 * recipe.c
 *
 *  Created on: Sep 27, 2021
 *      Author: abaji
 */

#include "recipe.h"

char buff[256];

/* Initializes a recipe structure's elements. */
void Recipe_Init(recipeState *recipe_state, const unsigned char *recipe, uint8_t id, servoState *servo_state) {
	// Assign default elements.
	recipe_state->recipe = recipe;
	recipe_state->id = id;
	recipe_state->index = 0;
	recipe_state->stall_count = 0;
	recipe_state->loop_start = 0;
	recipe_state->loop_count = 0;
	recipe_state->servo_state = servo_state;
	// Set default state as paused.
	Recipe_Update_Status(recipe_state, PAUSED);
}

/* Restarts a recipe and processes the first instruction immediately. */
void Recipe_Restart(recipeState *recipe_state) {
	Recipe_Update_Status(recipe_state, PAUSED);
	// Reset all indices and counters.
	recipe_state->index = 0;
	recipe_state->stall_count = 0;
	recipe_state->loop_start = 0;
	recipe_state->loop_count = 0;
	// Process first instructions.
	Recipe_Update_Status(recipe_state, RUNNING);
	Recipe_Process_Next(recipe_state);
}

/* Update the status of the recipe and provide printouts for error states. */
void Recipe_Update_Status(recipeState *recipe_state, recipeStatus status) {
	// Provide printout when in error state and trying to continue. */
	if ((recipe_state->status == COMMAND_ERROR || recipe_state->status == NESTED_ERROR) && status == RUNNING) {
		sprintf(buff, "Cannot continue recipe %d in error state. Please restart recipe.\r\n> ", recipe_state->id);
		print(buff);
	} else {
		// If new status is indicative of an error, provide printout.
		if (status == COMMAND_ERROR) {
			sprintf(buff, "RECIPE ERROR: invalid command found on instruction #%d of recipe %d\r\n> ", recipe_state->index+1, recipe_state->id);
			print(buff);
		} else if (status == NESTED_ERROR) {
			sprintf(buff, "RECIPE ERROR: nested loop found on instruction #%d of recipe %d\r\n> ", recipe_state->index+1, recipe_state->id);
			print(buff);
		}
		// Update the status field.
		recipe_state->status = status;
	}
	// Update status LEDs on MFS board.
	if (recipe_state->id == 1) {
		// Recipe 1 gets LEDs 3+4 for error statuses.
		MFS_set_led(1, 0);
		MFS_set_led(3, 0);
		MFS_set_led(4, 0);
		switch (recipe_state->status) {
			case RUNNING:
				MFS_set_led(1, 1);
				break;
			case COMMAND_ERROR:
				MFS_set_led(3, 1);
				break;
			case NESTED_ERROR:
				MFS_set_led(4, 1);
				break;
			default:
				break;
		}
	} else {
		// Set LED2 when recipe2 is running, otherwise it's off,
		if (recipe_state->status == RUNNING) {
			MFS_set_led(2, 1);
		} else {
			MFS_set_led(2, 0);
		}
	}
}

/* Process the next instruction of the given recipe */
void Recipe_Process_Next(recipeState *recipe_state) {
	// Only process commands when running.
	if (recipe_state->status == RUNNING) {
		// Execute if no stall cycles remain.
		if (recipe_state->stall_count == 0) {
			// Decode the instruction and process the opcode/args.
			int idx = recipe_state->index;
			recipeCommand cmd = recipe_state->recipe[idx] & ~0x1F;
			uint8_t param = recipe_state->recipe[idx] & ~0xE0;
			Recipe_Process_CMD(recipe_state, cmd, param);
		} else {
			// Stall this cycle (noop) and update stall count.
			recipe_state->stall_count--;
		}
	}
}

/* Processes an individual recipe command. */
void Recipe_Process_CMD(recipeState *recipe_state, recipeCommand cmd, uint8_t param) {
	// Switch based on the opcode.
	switch (cmd) {
		case MOV:
			// Move the servo to the designated position if it's within the servo bounds.
			if (param < 0 || param > 5) {
				Recipe_Update_Status(recipe_state, COMMAND_ERROR);
			} else {
				Servo_Move(recipe_state->servo_state, param);
				recipe_state->index++;
			}
			break;
		case WAIT:
			// Stall this cycle and update the stall count according to the parameters.
			recipe_state->stall_count = param;
			recipe_state->index++;
			break;
		case LOOP:
			// Nested loops prohibited.
			if (recipe_state->loop_start > 0) {
				Recipe_Update_Status(recipe_state, NESTED_ERROR);
			} else {
				// Update the start of the loop index and loop count.
				recipe_state->loop_start = recipe_state->index + 1;
				recipe_state->loop_count = param;
				recipe_state->index++;
			}
			break;
		case END_LOOP:
			// If loop iterations remain, reset program counter (index) back to the start of teh loop.
			if (recipe_state->loop_count > 0) {
				recipe_state->index = recipe_state->loop_start;
				recipe_state->loop_count--;
			} else {
				recipe_state->index++;
				recipe_state->loop_start = 0;
			}
			break;
		case RECIPE_END:
			// Pause the recipe at the end.
			sprintf(buff, "End of recipe %d reached.\r\n> ", recipe_state->id);
			print(buff);
			Recipe_Update_Status(recipe_state, PAUSED);
			break;
		default:
			// Invalid instruction.
			Recipe_Update_Status(recipe_state, COMMAND_ERROR);
	}
}
