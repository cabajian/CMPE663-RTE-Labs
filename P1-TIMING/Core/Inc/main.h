/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  * @details        : This file contains the common defines of the application.
  *
  * @author         : Chris Abajian
  * @date           : September 2021
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "clock.h"
#include "uart.h"
#include "timer.h"
#include <stdlib.h>

/* Defines -------------------------------------------------------------------*/
#define MAX_BUFF_SIZE (100)
#define MIN_PERIOD_US (100)
#define MAX_PERIOD_US (10000)

/* Prototypes ----------------------------------------------------------------*/
_Bool power_on_self_test(void);
int set_timer_base(int curr_period);
_Bool yes_no_prompt(void);
void print_buckets(uint16_t *bucket_ptr, int num_buckets, int lower_bound);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
