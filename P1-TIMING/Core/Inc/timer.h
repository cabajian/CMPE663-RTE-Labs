/**
  ******************************************************************************
  * @file           : timer.h
  * @brief          : Timer header file.
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32l476xx.h"

#define IC_TIMEOUT (60000000) // timeout in us (60s)

void TIM2_Init(void);
void TIM_Init_IC(TIM_TypeDef* TIMx);
uint32_t TIM_Capture_Input(TIM_TypeDef* TIMx);

#endif /* INC_TIMER_H_ */
