/*
 * servo.c
 *
 *  Created on: Sep 29, 2021
 *      Author: abaji
 */

#include "servo.h"

/* Initalizes a servo structure by assigning values to its elements. */
void Servo_Init(servoState *servo, TIM_TypeDef *htim, uint32_t channel) {
  servo->htim = htim;
  servo->channel = channel;
  servo->position = 0;
  servo->status = STEADY;
  Servo_Move(servo, 0);
}

/* Moves the servo to the appropriate duty cycle and updates the position. */
void Servo_Move(servoState *servo, int position) {
	PWM_Set_Duty(servo->htim, servo->channel, SERVO_POS_TO_DUTY(position));
	servo->position = position;
}
