/*
 * teller_task.h
 *
 *  Created on: Sept 25, 2021
 *      Author: rickweil
 */

#ifndef INC_CUSTOMER_TASK_H_
#define INC_CUSTOMER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"

#define MAX_CUSTOMERS			(50)	// maximum number of customers in the queue

#define CUSTOMER_ARRIVAL_MIN	(60)	//seconds
#define CUSTOMER_ARRIVAL_MAX	(240)	//seconds
#define RAND_CUSTOMER_TIME		(random_dist(CUSTOMER_ARRIVAL_MIN, CUSTOMER_ARRIVAL_MAX))

typedef struct {
	uint32_t enter_time;	// time the customer entered the queue
} CUSTOMER_t;

typedef struct {
	int length;				// length of the customer queue
	int max_length;			// max length of the customer queue
	int max_wait_time;		// max time a customer waited in the queue
	int total_wait_time;	// total cumulative customer wait time
	int num_served;			// total number of customers served
	TaskHandle_t handle;
} CUSTOMER_QUEUE_t;

extern CUSTOMER_QUEUE_t customer_queue;

int customer_queue_init();
int customer_queue_pop();
int customer_queue_push(CUSTOMER_t *customer);


#endif /* INC_TELLER_TASK_H_ */
