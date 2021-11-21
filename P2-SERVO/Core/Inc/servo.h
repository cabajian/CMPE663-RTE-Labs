/*
 * servo.h
 *
 *  Created on: Sep 29, 2021
 *      Author: abaji
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "common.h"

// Macro to convert the servo position [0,5] to a duty cycle.
#define SERVO_POS_TO_DUTY(pos)	(10.0-1.8*pos*(5.0/6.0))

// Possible statuses of a servo's activity.
typedef enum {
	STEADY,
	MOVING,
	UNKNOWN
} servoStatus;

// A servo's state is described by its status and position.
typedef struct {
	TIM_TypeDef *htim;
	uint32_t channel;
	int position;
	servoStatus status;
} servoState;

void Servo_Init(servoState *servo, TIM_TypeDef *htim, uint32_t channel);
void Servo_Move(servoState *servo, int position);

#endif /* INC_SERVO_H_ */
