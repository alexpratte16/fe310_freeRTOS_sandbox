/* header file include*/
#include "heartbeat.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>



/*defines*/
#define HEARTBEAT_DELAY 500
#define HEARTBEAT_TICK_COUNT_FOR_500MS			pdMS_TO_TICKS( HEARTBEAT_DELAY )
#define HEARTBEAT_TASK_PRIORITY tskIDLE_PRIORITY + 5

/*globals*/

static void heartbeat_task(void *pvParameters);

TaskHandle_t heartbeat_init(struct metal_gpio* gpio_handle){
    TaskHandle_t heartbeat_handler;
    xTaskCreate( heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, gpio_handle, HEARTBEAT_TASK_PRIORITY, &heartbeat_handler );
    return heartbeat_handler;
}

static void heartbeat_task(void *pvParameters){

	struct metal_gpio* gpio0_handle =  (struct metal_gpio*)pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	TickType_t xNextWakeTime = xTaskGetTickCount();

    while(1){
	    write( STDOUT_FILENO, "on\r\n", strlen( "on\r\n" ) );
        if(gpio0_handle != NULL){
            metal_gpio_set_pin(gpio0_handle, HEARTBEAT_PIN, 1);
        }
		vTaskDelayUntil( &xNextWakeTime, HEARTBEAT_TICK_COUNT_FOR_500MS );
        if(gpio0_handle != NULL){
            metal_gpio_set_pin(gpio0_handle, HEARTBEAT_PIN, 0);
        }
	    write( STDOUT_FILENO, "off\r\n", strlen( "off\r\n" ) );
        vTaskDelayUntil( &xNextWakeTime, HEARTBEAT_TICK_COUNT_FOR_500MS );
    }

}