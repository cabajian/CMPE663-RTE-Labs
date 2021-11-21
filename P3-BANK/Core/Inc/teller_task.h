/*
 * teller_task.h
 *
 *  Created on: Sept 25, 2021
 *      Author: rickweil
 */

#ifndef INC_TELLER_TASK_H_
#define INC_TELLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define NUM_TELLERS 3	// Maximum 8

#define TELLER_TRANSACT_MIN		(30)	//seconds
#define TELLER_TRANSACT_MAX		(480)	//seconds
#define RAND_TELLER_TIME	(random_dist(TELLER_TRANSACT_MIN, TELLER_TRANSACT_MAX))

typedef struct {
	// some parameters to track
	int instance;
	int customers_served;
	uint32_t wait_start;
	int max_wait_time;
	uint32_t total_wait_time;
	int max_transaction_time;
	uint32_t total_transaction_time;
	TaskHandle_t handle;
} TELLER_t;

extern TELLER_t tellers[NUM_TELLERS];

int teller_task_init(int num);


#endif /* INC_TELLER_TASK_H_ */
