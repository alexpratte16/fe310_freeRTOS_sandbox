#ifndef TEST_TASK_H
#define TEST_TASK_H


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* standard library includes*/
#include <stdint.h>

#include "spibus/spibus.h"

TaskHandle_t test_task_init(void);

#endif