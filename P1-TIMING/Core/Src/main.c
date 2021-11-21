/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ----------------------------------------------------------*/
uint8_t buffer[MAX_BUFF_SIZE];

/**
  * The application entry point.
  */
int main(void) {
  /* Define process variables */
  _Bool res = 0;
  int lower_limit = 950;
  _Bool rerun = 1;

  /* Initialize/configure peripherals */
  Clock_Init();         // Switch System Clock = 16 MHz
  USART2_Init(115200);	// initialize USART2
  GPIOA_Init();
  TIM2_Init();
  TIM_Init_IC(TIM2);

  /* Introduction and program initialization */
  USART_Write_Line(USART2, "CMPE-663 Project 1 by Chris Abajian (cxa6282@rit.edu)\r\n");
  // Run POST
  res = power_on_self_test();
  if (!res) {
    USART_Write_Line(USART2, "Halting, restart the program.\r\n");
    while (1) {} // halt
  }

  /* Infinite loop */
  while (rerun) {
    // Retrieve lower limit for timer period.
    lower_limit = set_timer_base(lower_limit+50) - 50;
    USART_Write_Line(USART2, "Running..");
    // Define buckets.
    uint16_t buckets[101] = {0};
    // Enable timer.
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
    // Align to the first rising edge.
    uint32_t val = TIM_Capture_Input(TIM2);
    // Begin capturing 1000 inter-arrival times. Note the timer continues to run
    // while we assign the last value to a bucket for accurate results.
    for (int i = 0; i < 1000; i++) {
      uint32_t prev_val = val;
      val = TIM_Capture_Input(TIM2);
      uint32_t delta = val - prev_val;
      if ((delta >= lower_limit) && (delta <= (lower_limit+100))) {
        buckets[delta - lower_limit] += 1;
      }
    }
    // Disable timer.
    TIM2->CR1 &= ~TIM_CR1_CEN;
    // Display the results.
    print_buckets(buckets, 101, lower_limit);
    USART_Write_Line(USART2, "Run again? ");
    rerun = yes_no_prompt();
  }
}

/**
  * POST to verify a correct environment and test conditions before running the program.
  */
_Bool power_on_self_test(void) {
  _Bool status = 0;
  USART_Write_Line(USART2, "Running POST..\r\n");
  /* Confirm the GPIO port is seeing pulses at least once in 100ms. */
  // Set preload to 100000 for 100ms until overflow.
  TIM2->ARR = 100000;
  // Loop
  while (1) {
    // Capture input.
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
    uint32_t val = TIM_Capture_Input(TIM2);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    if (val == 0xFFFFFFFF) {
      // Bad POST. Provide option to rerun.
      USART_Write_Line(USART2, "..No pulse detected within 100ms. Try POST again? ");
      _Bool rerun = yes_no_prompt();
      if (!rerun) {
        USART_Write_Line(USART2, "..POST failed\r\n");
        break;
      } else {
        USART_Write_Line(USART2, "..rerunning..\r\n");
      }
    } else {
      // Good POST.
      USART_Write_Line(USART2, "..POST completed\r\n");
      status = 1;
      break;
    }
  }
  // Return auto-reload to max.
  TIM2->ARR = 0xFFFFFFFF;
  return status;
}

/*
 * Prompts user for timer period.
 */
int set_timer_base(int curr_period) {
  char *c_ptr;
  int period = curr_period;
  // Prompt period range
  sprintf(buffer, "Current range: [%d,%d] us\r\n", period-50, period+50);
  USART_Write_Line(USART2, buffer);
  USART_Write_Line(USART2, "Do you want to change the period? ");
  _Bool modify = yes_no_prompt();
  if (modify) {
    // Update period
    sprintf(buffer, "..Enter a new period (%d to %d): ", MIN_PERIOD_US, MAX_PERIOD_US);
    USART_Write_Line(USART2, buffer);
    USART_Read_Line(USART2, buffer, MAX_BUFF_SIZE);
    period = (int) strtol(buffer, &c_ptr, 10);
    // Cap period to bounds.
    if (period == 0 && buffer[0] != '0') {
      sprintf(buffer, "....unknown input, retaining %d us\r\n", curr_period);
      USART_Write_Line(USART2, buffer);
      period = curr_period;
    } else if (period < MIN_PERIOD_US) {
      sprintf(buffer, "....period too small, binding to %d us\r\n", MIN_PERIOD_US);
      USART_Write_Line(USART2, buffer);
      period = 100;
    } else if (period > MAX_PERIOD_US) {
      sprintf(buffer, "....period too large, binding to %d us\r\n", MAX_PERIOD_US);
      USART_Write_Line(USART2, buffer);
      period = 10000;
    }
    sprintf(buffer, "..New limits: [%d to %d]\r\n", period-50, period+50);
    USART_Write_Line(USART2, buffer);
  }
  return period;
}

/*
 * Simple yes/no prompt in terminal. Return true if 'y' or 'Y'
 * are entered, false otherwise.
 */
_Bool yes_no_prompt(void) {
  USART_Write_Line(USART2, "(y/n) ");
  uint8_t c = USART_Read(USART2);
  USART_Write(USART2, &c, 1);
  USART_Write_Line(USART2, "\r\n");
  return (c == 'y' || c == 'Y');
}

/*
 * Prints a table of non-zero bucket occurrences.
 */
void print_buckets(uint16_t *bucket_ptr, int num_buckets, int lower_bound) {
  USART_Write_Line(USART2, "\r\nBUCKET | OCCURENCES\r\n");
  USART_Write_Line(USART2, "-------+-----------\r\n");
  for (int i = 0; i < num_buckets; i++) {
    uint16_t val = bucket_ptr[i];
    if (val) {
      sprintf(buffer, "%d\t | %d\r\n", i+lower_bound, val);
      USART_Write_Line(USART2, buffer);
    }
  }
  USART_Write_Line(USART2, "\n");
}
