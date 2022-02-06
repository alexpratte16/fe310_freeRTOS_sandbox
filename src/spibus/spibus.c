
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
#define SPIBUS_TX_TASK_PRIORITY tskIDLE_PRIORITY + 7
#define SPIBUS_TX_DELAY 2
#define SPIBUS_TX_TICK_COUNT_FOR_2MS			pdMS_TO_TICKS( SPIBUS_TX_DELAY )
#define SPIBUS_TEST_BYTE 0xAA
#define SPIBUS_SPI_1 0x10024000 // I don't like hardcoding this but I will find another way eventually
#define SPIBUS_TX_MAX_QUEUE_LEN 64
#define SPIBUS_MAX_DEVICES 4

/* globals */
struct spibus_device_config spibus_device_configs[SPIBUS_MAX_DEVICES];
struct metal_spi* spi_handle; // is this needed?
QueueHandle_t spibus_tx_queue_handle = NULL;
QueueHandle_t spibus_rx_queue_handle[SPIBUS_MAX_DEVICES]; // rx queue for each device?

static void spibus_tx_task(void *pvParameters);

// init spi bus
// this function is essentially useless because I am resetting the configs 
// on new device transfer
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
	metal_spi_init(spi_handle, 1000000);

    return ret;
}


// consider diabling interrupts for contiguous transfers
// adds buffer items to send queue
// change return value to number of items not stored
int spibus_send(uint8_t cs, uint8_t* buffer, int len){
//int spibus_send(uint8_t* buffer, int len){
    int ret = 0;
    if(len > uxQueueSpacesAvailable(spibus_tx_queue_handle)){
        return -1;
    }
    for(int ibuf = 0; ibuf < len; ibuf++){
        struct spibus_packet send = {cs, buffer[ibuf]};
        ret = xQueueSend( spibus_tx_queue_handle, &send, 0U );
        //ret = xQueueSend( spibus_tx_queue_handle, buffer+ibuf, 0U );
        if(ret != pdPASS){
            return ret;
        }
    }
    return 0;
}
// look into queue of cs device fill buffer with as many as possible or len, whichever is shorter
// int spibus_receive(uint8_t cs, uint8_t* buffer, int len){
int spibus_receive(uint8_t* buffer, int len){
    return 0;

}

// initialize spibus_tx_task, return task handle
// add an interrupt for receive
TaskHandle_t spibus_init(void){
    TaskHandle_t spibus_task_handle = NULL;

    uint8_t spibus_dev_num = spibus_init_hw();
    if(spibus_dev_num  < 0 ){
        printf("spibus_init failed hw\r\n");
        return NULL;
    }
    int* spi_base_addr = (int*)SPIBUS_SPI_1; 
    printf("got spibus addr: 0x%X\r\n", (unsigned int)spi_base_addr);
    //spibus_tx_queue_handle = xQueueCreate( SPIBUS_TX_MAX_QUEUE_LEN, sizeof(uint8_t));
    spibus_tx_queue_handle = xQueueCreate( SPIBUS_TX_MAX_QUEUE_LEN, sizeof(struct spibus_packet));
    spi_base_addr[0x14] = 1; // set tx watermark to 1
    spi_base_addr[0x1C] = 1; // enable tx watermark interrupt


    xTaskCreate(spibus_tx_task, "spibus_tx", configMINIMAL_STACK_SIZE, spi_base_addr, SPIBUS_TX_TASK_PRIORITY, &spibus_task_handle );

    return spibus_task_handle;
}

// load cs device from spibus_device_configs[cs] into peripheral 
spibus_load_device_config(uint8_t cs, int* base_addr){
    if(cs >= SPIBUS_MAX_DEVICES){
        return -1;
    }
    // disable interrupts

    // load values from spibus_device_configs[cs]
    base_addr[0x01] = (spibus_device_configs[cs].clk_pha & 0x01) | ((spibus_device_configs[cs].clk_pol & 0x01) << 1); // set clock phase and polarity
    base_addr[0x04] = cs; // set cs
    base_addr[0x0A] = (spibus_device_configs[cs].cs_sck & 0xFF) | ((spibus_device_configs[cs].sck_cs & 0xFF) << 8); // set delay 0 register
    base_addr[0x0B] = (spibus_device_configs[cs].intercs & 0xFF) | ((spibus_device_configs[cs].interxfer & 0xFF) << 8); // set delay1 register



    // enable interrupts

    return 0;
}

//load config into spibus_device_configs[cs]
// this should be done while no transfers are happening AKA only at beginning 
int spibus_set_device_config(uint8_t cs, struct spibus_device_config config){
    if(cs >= SPIBUS_MAX_DEVICES){
        return -1;
    }
    spibus_device_configs[cs] = config;

    return 0;
}

//spibus tx task waits for womething to go onto the queue and then sends it if there is space
static void spibus_tx_task(void *pvParameters){

    volatile uint32_t* spi_base_addr  = (volatile uint32_t*)pvParameters;
    //uint8_t recv = 0; 
    struct spibus_packet recv; 

    spi_base_addr[0x1] =  spi_base_addr[0x1]; //adjust polarity and phase as necesary
    spi_base_addr[0x4] |= 0; //use CS 0
    spi_base_addr[0x6] |= 2; //set CS mode to HOLD
    spi_base_addr[0x10] = spi_base_addr[0x10]; //adjust frame format as necesary

    uint8_t last_cs = SPIBUS_MAX_DEVICES; //make sure we load a new config on first transfer

    while(1){
        

        //wait for something to show up on the queue
        xQueueReceive(spibus_tx_queue_handle, &recv, portMAX_DELAY );

        if(last_cs != recv.cs){
            //need to make sure FIFO is clear before loading new config
            ///figure that out!
            last_cs = recv.cs;
            //vTaskDelay( 1 ); //delay to let fifo clear, which is really not great
            // seems to shift the data out fast enough when the bus is at 1Mhz

            spibus_load_device_config(last_cs, spi_base_addr);

        }
        

        /*
            HOW DO I MAKE SURE THERE IS NOTHING IN THE TX FIFO?????
            think about limiting SPIBUS usage to just sending one at a time
            wait for interrupt with tx watermark=1 to post before snding one
        */

		//vTaskDelayUntil( &xNextWakeTime, SPIBUS_TX_TICK_COUNT_FOR_2MS );
        //make sure fifo isnt full, in rxdata reg 31st bit
        //pend until there is room
        //should put this on an iterrupt, have this task pend on flag from interrupt
        //

        while((spi_base_addr[0x12] >> 31)){
            ;
        }
        spi_base_addr[0x12] |= recv.data; //send something distinct
            
    }

}
