/**
  ******************************************************************************
  * @file           : timer.c
  * @brief          : Timer initialization and ISR routines.
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"

/* Global variables ----------------------------------------------------------*/
_Bool ovfl = 0; // Overflow flag.

/**
  * TIM2 Timer initialization.
  */
void TIM2_Init(void) {
  // Enable TIM2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  // Load prescaler. Default clock freq is 16MHz, or 62.5ns period.
  // A prescaler of 16 yields a 1us period per count.
  TIM2->PSC = (uint16_t) 15;
  // Trigger event to update prescaler.
  TIM2->EGR |= TIM_EGR_UG;
  //
  TIM2->DIER |= TIM_DIER_UIE;
  //
  NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * Configure TIMx for input capture mode. See Fig. 309 in reference manual for block diagram.
  */
void TIM_Init_IC(TIM_TypeDef* TIMx) {
  // Disable CH1 compare/capture mode so we can configure the capture mode register.
  // The capture/compare mode register's "selection" bits are read-only if CC1E is set. 
  TIMx->CCER &= ~TIM_CCER_CC1E;
  // Configure capture/compare CH1 as input w/ IC1 mapped on TI1.
  TIMx->CCMR1 = (TIMx->CCMR1 & 0xFFFFFFFC) | TIM_CCMR1_CC1S_0;
  // Make sure input capture 1 filter is cleared.
  TIMx->CCMR1 &= ~TIM_CCMR1_IC1F;
  // Reenable capture mode.
  TIMx->CCER |= TIM_CCER_CC1E;
}

/**
  * Waits for a single input capture and returns the counter value. Times out on
  * an overflow of the counter and returns 0xFFFFFFFF.
  */
uint32_t TIM_Capture_Input(TIM_TypeDef* TIMx) {
  ovfl = 0;
  TIMx->SR &= ~TIM_SR_CC1IF;
  // Wait for input capture interrupt
  while (!(TIMx->SR & TIM_SR_CC1IF)) {
    // If timer passes timeout value or overflows, exit.
    if (ovfl || (TIMx->CNT == IC_TIMEOUT)) {
        TIMx->SR &= ~TIM_SR_UIF;
        TIMx->CNT = 0;
        return 0xFFFFFFFF;
    }
  }
  // Fetch latched timer value. Reading this register resets the CC1IF interrupt bit.
  return TIMx->CCR1;
}

/*
 * TIM2 IRQ Handler.
 */
void TIM2_IRQHandler() {
  TIM2->SR = 0;
  ovfl = 1;
}
