/*
 * recipes.h
 *
 *  Created on: Oct 2, 2021
 *      Author: abaji
 */

#ifndef INC_RECIPES_H_
#define INC_RECIPES_H_

// Sample recipe provided in exercise handout.
const unsigned char RECIPE_A[] = {
		MOV  | 0,
		MOV  | 5,
		MOV  | 0,
		MOV  | 3,
		LOOP | 0,
		MOV  | 1,
		MOV  | 4,
		END_LOOP,
		MOV  | 0,
		MOV  | 2,
		WAIT | 0,
		MOV  | 3,
		WAIT | 0,
		MOV	 | 2,
		MOV  | 3,
		WAIT | 31,
		WAIT | 31,
		WAIT | 31,
		MOV	 | 4,
		RECIPE_END
};

// Iterates through all 6 servo positions.
const unsigned char RECIPE_B[] = {
		MOV  | 0,
		WAIT | 31,
		MOV  | 1,
		WAIT | 31,
		MOV  | 2,
		WAIT | 31,
		MOV  | 3,
		WAIT | 31,
		MOV  | 4,
		WAIT | 31,
		MOV  | 5,
		WAIT | 31,
		RECIPE_END,
		MOV  | 3 // shouldn't execute
};

// Nested loop.
const unsigned char RECIPE_C[] = {
		MOV  | 0,
		LOOP | 31,
		LOOP | 10,
		END_LOOP,
		END_LOOP,
		RECIPE_END
};

// Out-of-range parameter.
const unsigned char RECIPE_D[] = {
		MOV  | 10,
		RECIPE_END
};

// Command error (since 32 overflows and forms "011" opcode)
const unsigned char RECIPE_E[] = {
		WAIT | 32,
		RECIPE_END
};

#endif /* INC_RECIPES_H_ */
