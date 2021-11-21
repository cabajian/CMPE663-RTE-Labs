/**
  ******************************************************************************
  * @file           : clock.c
  * @brief          : Onboard clock initialization and supporting functions.
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l476xx.h"

/**
  * Clock initialization function.
  */
void Clock_Init(void) {
    RCC->CR |= ((uint32_t)RCC_CR_HSION);

    // wait until HSI is ready
    while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}

    // Select HSI as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;     // 01: HSI16 oscillator used as system clock

    // Wait till HSI is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 ) {;}
}
