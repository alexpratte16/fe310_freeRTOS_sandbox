#ifndef HEARTBEAT_H
#define HEARTBEAT_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Metal includes. */
#include <metal/gpio.h>

#define HEARTBEAT_PIN 0

TaskHandle_t heartbeat_init(struct metal_gpio* gpio_handle);

#endif