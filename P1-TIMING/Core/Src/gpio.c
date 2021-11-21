/**
  ******************************************************************************
  * @file           : gpio.c
  * @brief          : GPIO initialization functions.
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

#include "gpio.h"

/*
 * GPIOA Initialization. Enables the clock and sets PA5 to alternate function 1.
 */
void GPIOA_Init() {
    // Enable the clock to GPIO Ports A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // enable clock for the User LED, UART
    // INIT TIM2_CH1 alternate function
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODE5) | GPIO_MODER_MODE5_1;  // set GPIOA pin5 to alt function mode
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL5) | GPIO_AFRL_AFSEL5_0; // select GPIOA pin5 to alt function 1
}
