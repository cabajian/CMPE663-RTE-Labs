/*
 * customer_task.c
 *
 *  Created on: Oct 16, 2021
 *      Author: abaji
 */

#include "customer_task.h"
#include "string.h"
#include "stdio.h"

CUSTOMER_QUEUE_t customer_queue;

// private methods
static void customer_task(void *params);
static QueueHandle_t queue_handle;

/**
 * This public method creates an empty queue of customers. A customer task
 * is created to periodically add customers to the queue.
 */
int customer_queue_init() {
	// Create the RTOS queue.
	queue_handle = xQueueCreate(MAX_CUSTOMERS, sizeof(CUSTOMER_t));
	assert(queue_handle != NULL);
	// Set the queue metrics.
	customer_queue.length = 0;
	customer_queue.max_length = 0;
	customer_queue.max_wait_time = 0;
	customer_queue.total_wait_time = 0;
	customer_queue.num_served = 0;
	TaskHandle_t th;
	// Start customer task.
	BaseType_t err = xTaskCreate(customer_task, "CustomerTask", 128, NULL, 12, &th);
	assert(err == pdPASS);
	customer_queue.handle = th;
	// Success.
	return 0;
}

static void customer_task(void *params) {
	while(1) {
		// Wait for random time before queuing next customer.
		htim2.Instance->CCR4 = htim2.Instance->CNT + SIM_SEC_TO_REAL_US(RAND_CUSTOMER_TIME);
		vTaskSuspend(NULL);
		// Create a new customer.
		CUSTOMER_t c = {htim2.Instance->CNT};
		// Queue the customer.
		int err = customer_queue_push(&c);
		assert(!err);
	}
}

int customer_queue_push(CUSTOMER_t *customer) {
	if (customer_queue.length < MAX_CUSTOMERS) {
		// Add the customer to the queue.
		BaseType_t err = xQueueSendToBack(queue_handle, customer, portMAX_DELAY);
		assert(err = pdPASS);
		// Update the queue metrics.
		osMutexAcquire(customerQueueMutexHandle, portMAX_DELAY);
		int length = ++customer_queue.length;
		if (length > customer_queue.max_length) {
			customer_queue.max_length = length;
		}
		osMutexRelease(customerQueueMutexHandle);
		// Success.
		return 0;
	} else {
		// Failure-- queue is full.
		return 1;
	}
}

int customer_queue_pop() {
	if (customer_queue.length > 0) {
		// Remove the customer from the queue.
		CUSTOMER_t c;
		BaseType_t err = xQueueReceive(queue_handle, &c, portMAX_DELAY);
		assert(err = pdPASS);
		// Update the queue metrics.
		osMutexAcquire(customerQueueMutexHandle, portMAX_DELAY);
		customer_queue.length--;
		uint32_t wait_time = htim2.Instance->CNT - c.enter_time;
		wait_time = REAL_US_TO_SIM_SEC(wait_time);
		customer_queue.total_wait_time += wait_time;
		if (wait_time > customer_queue.max_wait_time) {
			customer_queue.max_wait_time = wait_time;
		}
		customer_queue.num_served++;
		osMutexRelease(customerQueueMutexHandle);
		// Success.
		return 0;
	} else {
		// Failure-- queue is empty.
		return 1;
	}
}
