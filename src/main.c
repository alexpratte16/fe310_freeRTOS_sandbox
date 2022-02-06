/* Copyright 2019 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */



/******************************************************************************
 *
 * main() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 1000 milliseconds, before sending the value 100 to the queue that
 * was created within main().  Once the value is sent, the task loops
 * back around to block for another 1000 milliseconds...and so on.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, writes 'Blink' to the UART
 * (the UART is used in place of the LED to allow easy execution in QEMU).  The
 * 'block time' parameter passed to the queue receive function specifies that
 * the task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 1000 milliseconds, the queue receive
 * task leaves the Blocked state every 1000 milliseconds, and therefore toggles
 * the LED every 1 second.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Freedom metal includes. */
#include <metal/machine.h>
#include <metal/machine/platform.h>

#include <metal/lock.h>
#include <metal/uart.h>
#include <metal/interrupt.h>
#include <metal/clock.h>
#include <metal/led.h>

/* Project includes */
#include "heartbeat/heartbeat.h"
#include "spibus/spibus.h"
#include "test_task/test_task.h"



/*-----------------------------------------------------------*/
/*
 * Functions:
 * 		- prvSetupHardware: Setup Hardware according CPU and Board.
 */
static void prvSetupHardware( void );


/*-----------------------------------------------------------*/

/* The queue used by both tasks. */

struct metal_gpio *gpio0;

#define LED_GPIO 5

#define LED_ERROR ((led0_red == NULL) || (led0_green == NULL) || (led0_blue == NULL))

/*-----------------------------------------------------------*/
int main( void )
{
	TaskHandle_t 	heartbeat_handle, spibus_handle, test_task_handle;
	const char * const pcMessage = "FreeRTOS Demo start\r\n";
	const char * const pcMessageEnd = "FreeRTOS Demo end\r\n";

	prvSetupHardware();
	write( STDOUT_FILENO, pcMessage, strlen( pcMessage ) );

	write( STDOUT_FILENO, "init heartbeat task\r\n", strlen( "init heartbeat task\r\n" ) );
	printf("printf test%d\r\n", 420);
	heartbeat_handle = heartbeat_init(gpio0);
	write( STDOUT_FILENO, "heartbeat task inited\r\n", strlen("heartbeat task inited\r\n") );

	printf("init spibus task\r\n");
	spibus_handle = spibus_init();
	printf("spibus task inited\r\n");

	printf("init test task task\r\n");
	test_task_handle = test_task_init();
	printf("test task task inited\r\n");

	write( STDOUT_FILENO, "starting scheduler\r\n", strlen( "starting scheduler\r\n" ) );
	vTaskStartScheduler();

	vTaskDelete( heartbeat_handle );
	vTaskDelete( spibus_handle );




}


static void prvSetupHardware( void )
{
	int ret;


	gpio0 = metal_gpio_get_device(0);
	ret = metal_gpio_enable_output(gpio0, HEARTBEAT_PIN);
	ret = metal_gpio_set_pin(gpio0, HEARTBEAT_PIN, 0);

}
/*-----------------------------------------------------------*/


void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();

	_exit(1);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();

	write( STDOUT_FILENO, "ERROR Stack overflow on func: ", 30 );
	write( STDOUT_FILENO, pcTaskName, strlen( pcTaskName ) );

	_exit(1);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* The tests in the full demo expect some interaction with interrupts. */
}
/*-----------------------------------------------------------*/

void vAssertCalled( void )
{
	taskDISABLE_INTERRUPTS();

	_exit(1);
}
