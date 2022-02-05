#include "test_task.h"

#define TEST_TASK_TASK_PRIORITY tskIDLE_PRIORITY + 2
#define TEST_TASK_DELAY 250
#define TEST_TASK_TICK_COUNT_FOR_250MS			pdMS_TO_TICKS( TEST_TASK_DELAY )


static void test_task_task(void *pvParameters);

TaskHandle_t test_task_init(void){
        TaskHandle_t test_task_task_handle = NULL;

        xTaskCreate(test_task_task, "test task", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_TASK_PRIORITY, &test_task_task_handle );
        return test_task_task_handle;
}

static void test_task_task(void *pvParameters){
    uint8_t to_send_spi[] = {1,2,3,4,5,6,7,8,9,10};
    TickType_t xNextWakeTime = xTaskGetTickCount();

    while(1){
        spibus_send(to_send_spi, sizeof(to_send_spi)/sizeof(uint8_t));
        vTaskDelayUntil( &xNextWakeTime, TEST_TASK_TICK_COUNT_FOR_250MS );
    }
}