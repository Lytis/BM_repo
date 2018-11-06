#ifndef FIFO_H
#define FIFO_H


#include "stm32f7xx_hal.h"
#include "main.h"

#define USB_FIFO_PACKETS    30
#define USB_PACKET_SAMPLES  24
#define SUB_PACKETS         200

#define FIFO_QUEUES         4

#define MIC                 0


void put_samples(int16_t* rx_buffer);
uint8_t* get_packet(void);
void fifo_init(void);


#endif

