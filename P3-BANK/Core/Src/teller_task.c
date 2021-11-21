/*
 * teller_task.c
 *
 *  Created on: Sept 25, 2021
 *      Author: rickweil
 */


#include "teller_task.h"
#include "customer_task.h"
#include "string.h"
#include "stdio.h"

TELLER_t tellers[NUM_TELLERS];

// private methods
static void teller_task(void *params);

/**
 * This public method creates a number of teller tasks.
 */
int teller_task_init(int num_tellers) {
	// Create each teller.
	for(int ii=0; ii<num_tellers; ii++) {
		TELLER_t *t = &tellers[ii];
		memset(t, 0, sizeof(TELLER_t));
		t->instance = ii;
		TaskHandle_t th;
		// Launch the teller task.
		BaseType_t err = xTaskCreate(teller_task, "TellerTask", 256, &tellers[ii], 12, &th);
		assert(err == pdPASS);
		// Store the handle to suspend/resume the task at a later time.
		t->handle = th;
	}
	// Success.
	return 0;
}

/**
 * RTOS task for an individual teller. A teller attempts to serve a customer
 * from the queue by popping the head of the queue and suspending this task
 * until a timer interrupt resumes the task.
 */
static void teller_task(void *params) {
	TELLER_t *t = (TELLER_t *)params;
	int instance = t->instance;	// which teller am I?
	t->wait_start = htim2.Instance->CNT;
	while(1) {
		// Serve the next customer if the queue isn't empty.
		int err = customer_queue_pop();
		if (!err) {
			// Update time teller was waiting.
			uint32_t wait_time = htim2.Instance->CNT - t->wait_start;
			wait_time = REAL_US_TO_SIM_SEC(wait_time);
			t->total_wait_time += wait_time;
			if (wait_time > t->max_wait_time) {
				t->max_wait_time = wait_time;
			}
			t->wait_start = 0;
			// Update teller instance as busy.
			osMutexAcquire(availTellersMutexHandle, portMAX_DELAY);
			availTellers &= ~((uint8_t)(0x01 << instance));
			osMutexRelease(availTellersMutexHandle);
			// Block task for teller transaction time.
			int transaction_time = RAND_TELLER_TIME;
			t->total_transaction_time += transaction_time;
			if (transaction_time > t->max_transaction_time) {
				t->max_transaction_time = transaction_time;
			}
			if (instance == 0) {
				htim2.Instance->CCR1 = htim2.Instance->CNT + SIM_SEC_TO_REAL_US(transaction_time);
			} else if (instance == 1) {
				htim2.Instance->CCR2 = htim2.Instance->CNT + SIM_SEC_TO_REAL_US(transaction_time);
			} else if (instance == 2) {
				htim2.Instance->CCR3 = htim2.Instance->CNT + SIM_SEC_TO_REAL_US(transaction_time);
			}
			vTaskSuspend(NULL);
			// Begin wait time.
			t->wait_start = htim2.Instance->CNT;
			// Update customers_served statistic.
			t->customers_served++;
			// Update teller as free.
			osMutexAcquire(availTellersMutexHandle, portMAX_DELAY);
			availTellers |= ((uint8_t)(0x01 << instance));
			osMutexRelease(availTellersMutexHandle);
		}
	}
}
