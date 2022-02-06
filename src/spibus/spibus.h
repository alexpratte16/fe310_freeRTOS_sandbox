#ifndef SPIBUS_H
#define SPIBUS_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* standard library includes*/
#include <stdint.h>

struct spibus_device_config{
    uint8_t clk_pol; //0x01 << 1
    uint8_t clk_pha; //0x01 
    uint8_t cs_sck; //0x0A
    uint8_t sck_cs; //0x0A << 16
    uint8_t intercs; //0x0B
    uint8_t interxfer; // 0x0B <<24
};

struct spibus_packet{
    uint8_t cs;
    uint8_t data;
};

int spibus_send(uint8_t cs, uint8_t* buffer, int len); //todo add chip selection to this and send spibus_packets down the queue to load different configs for different chips
int spibus_receive(uint8_t* buffer, int len); //4 seperate queues for each chip?
int spibus_set_device_config(uint8_t cs, struct spibus_device_config config);
TaskHandle_t spibus_init(void);




#endif