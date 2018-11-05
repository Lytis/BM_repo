#ifndef FIFO_H
#define FIFO_H


#include "stm32f7xx_hal.h"
#include "main.h"

#define USB_FIFO_PACKETS    30
#define USB_PACKET_SAMPLES  24


void put_sample(int16_t sample);
uint8_t* get_packet(void);
int packet_samples(void);


#endif

