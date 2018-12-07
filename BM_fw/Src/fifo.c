#include "fifo.h"
#include "usbd_audio.h"
#include "storage.h"
#include <math.h>
#include <stdio.h>

int Prd = 0, Swr = 0, Pwr = 0;

int16_t USB_fifo[FIFO_SIZE][FIFO_PACKET_SIZE];
int16_t zero_pad[FIFO_PACKET_SIZE];

int USB_fifo_use = 0;
double t = 0, dt = 4.16e-5, w = 2512, ramp = 0, dramp = 1e-4;


void put_samples(int16_t sample)
{
    
    USB_fifo [Pwr][Swr] = sample;

    if (USB_fifo_use == 1)
    {
    Swr++;
        if (Swr == FIFO_PACKET_SIZE)
        {
            Swr = 0;
            Pwr++;
            if(Pwr == FIFO_SIZE)
            {
                Pwr = 0;
            }
            if (Pwr == Prd)
            {
                Prd +=1;
            }
        }
    }

}




uint8_t* get_packet()
{
    static uint8_t* tempPtr = NULL;

    if (Prd != Pwr)
    {
        tempPtr = (uint8_t*)USB_fifo[(Prd)%(FIFO_SIZE-1)];
        Prd++;
        Prd %= FIFO_SIZE;
    }else
    {
        tempPtr = (uint8_t*)zero_pad;
    }

    return tempPtr;
}

void fifo_put(int16_t* buffer, int mic)
{
    int i;
    mic %=8;

    for (i=mic; i<BUFFER_SIZE/2; i+=8)
    {
        put_samples(buffer[i]);
    }
}



