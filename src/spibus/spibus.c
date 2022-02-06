
/* header include */
#include "spibus.h"

/* Metal includes. */
#include <metal/spi.h>
#include <metal/machine.h>

/* std lib include */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>


/* defines */
#define SPIBUS_TX_TASK_PRIORITY tskIDLE_PRIORITY + 2
#define SPIBUS_TX_DELAY 2
#define SPIBUS_TX_TICK_COUNT_FOR_2MS			pdMS_TO_TICKS( SPIBUS_TX_DELAY )
#define SPIBUS_TEST_BYTE 0xAA
#define SPIBUS_SPI_1 0x10024000 //I don't like hardcoding this but I will find another way eventually
#define SPIBUS_TX_MAX_QUEUE_LEN 64
#define SPIBUS_MAX_DEVICES 4

struct spibus_device_config spibus_device_configs[SPIBUS_MAX_DEVICES];

struct metal_spi* spi_handle;

static QueueHandle_t spibus_tx_queue_handle = NULL;


static void spibus_tx_task(void *pvParameters);

uint8_t spibus_init_hw(){
    int ret = 1;
    /* Get SPI 1 */
	spi_handle = metal_spi_get_device(1);

	/* Fallback to SPI 0 */
	if(spi_handle == NULL) {
		spi_handle = metal_spi_get_device(0);
        ret = 0;
	}

    /* this isn't good */
    if(spi_handle == NULL) {
		return -1;
	}

    /* Initialize the SPI device to 100_000 baud */
	metal_spi_init(spi_handle, 100000);

    return ret;
}


//consider diabling interrupts for contiguous transfers
//int spibus_send(uint8_t cs, uint8_t* buffer, int len){
int spibus_send(uint8_t* buffer, int len){
    int ret = 0;
    if(len > SPIBUS_TX_MAX_QUEUE_LEN){
        return -1;
    }
    //struct spibus_packet send = {cs, buffer[ibuff]};
    for(int ibuf = 0; ibuf < len; ibuf++){
        //ret = xQueueSend( spibus_tx_queue_handle, &send, 0U );
        ret = xQueueSend( spibus_tx_queue_handle, buffer+ibuf, 0U );
        if(ret != pdPASS){
            return ret;
        }
    }
    return 0;
}

int spibus_receive(uint8_t* buffer, int len){
    return 0;

}
TaskHandle_t spibus_init(void){
    TaskHandle_t spibus_task_handle = NULL;

    uint8_t spibus_dev_num = spibus_init_hw();
    if(spibus_dev_num  < 0 ){
        printf("spibus_init failed hw\r\n");
        return NULL;
    }
    int* spi_base_addr = (int*)SPIBUS_SPI_1; 
    printf("got spibus addr: 0x%X\r\n", (unsigned int)spi_base_addr);
    spibus_tx_queue_handle = xQueueCreate( SPIBUS_TX_MAX_QUEUE_LEN, sizeof(uint8_t));
    //spibus_tx_queue_handle = xQueueCreate( SPIBUS_TX_MAX_QUEUE_LEN, sizeof(struct spibus_packet));

    xTaskCreate(spibus_tx_task, "spibus_tx", configMINIMAL_STACK_SIZE, spi_base_addr, SPIBUS_TX_TASK_PRIORITY, &spibus_task_handle );

    return spibus_task_handle;
}
//this should be done while no transfers are happening AKA only at beginning 
spibus_load_device_config(uint8_t cs, int* base_addr){
    if(cs >= SPIBUS_MAX_DEVICES){
        return -1;
    }
    //disable interrupts

    //load values from spibus_device_configs[cs]

    //enable interrupts

    return 0;
}

int spibus_set_device_config(uint8_t cs, struct spibus_device_config config){
    if(cs >= SPIBUS_MAX_DEVICES){
        return -1;
    }
    spibus_device_configs[cs] = config;

    return 0;
}

static void spibus_tx_task(void *pvParameters){

    volatile int* spi_base_addr  = (volatile int*)pvParameters;
    uint8_t recv = 0; 
    //spibus_packet_t recv = 0; 

    spi_base_addr[0x1] =  spi_base_addr[0x1]; //adjust polarity and phase as necesary
    spi_base_addr[0x4] |= 0; //use CS 0
    spi_base_addr[0x6] |= 2; //set CS mode to HOLD
    spi_base_addr[0x10] = spi_base_addr[0x10]; //adjust frame format as necesary

    uint8_t last_cs = SPIBUS_MAX_DEVICES; //make sure we load a new config on first transfer

    while(1){
        

        //wait for something to show up on the queue
        xQueueReceive(spibus_tx_queue_handle, &recv, portMAX_DELAY );

        //if(last_cs != recv.cs){
        //    need to make sure FIFO is clear before loading new config
        //    figure that out!
        //    last_cs != recv.cs
        //    spibus_load_device_config(last_cs, spi_base_addr);
        //}
        

		//vTaskDelayUntil( &xNextWakeTime, SPIBUS_TX_TICK_COUNT_FOR_2MS );
        //make sure fifo isnt full, in rxdata reg 31st bit
        //pend until there is room
        //should put this on an iterrupt, have this task pend on flag from interrupt
        //

        while((spi_base_addr[0x12] >> 31)){
            ;
        }
        spi_base_addr[0x12] |= recv; //send something distinct
            
    }

}
