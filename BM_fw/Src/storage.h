#ifndef STORAGE_H
#define STORAGE_H

#include "stm32f7xx_hal.h"
#include "main.h"

#define BUFFER_SIZE 9600        //7680
#define STORAGE_BUFFER_SIZE BUFFER_SIZE*4



#define HALF            2
#define FULL            1
#define EMPTY           0

//every packet has 7680 samples from 8 mics. 
// So there are 7680/8 = 960 samples per mic per packet
// at 24kHz sampling rate every packet holdes 960/24k = 40 milli seconds of recording time

//data rate per module = 7680samples*2bytes/40msecs = 384kByte/sec = 3.072Mbit/sec
//data rate for 4 modules = 4*384kByte/sec = 1.536Mbyte/sec = 12.288Mbit/sec
//if 32 bits are used with 48ksamples/sec then data rate should be 49.152Mbit/sec


//an alternative is to use 48kHz sampling rate and send the first data with 32 bit
//and the rest with 16 bit. This should end up close to 3.072MByte/sec or 24.576Mbit/sev

#endif


