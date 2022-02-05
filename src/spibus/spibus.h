#ifndef SPIBUS_H
#define SPIBUS_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* standard library includes*/
#include <stdint.h>

int spibus_send(uint8_t* buffer, int len);
int spibus_receive(uint8_t* buffer, int len);
TaskHandle_t spibus_init(void);


#endif