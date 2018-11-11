#include "fifo.h"
#include "usbd_audio.h"

#include <math.h>
#include <stdio.h>

int Prd = 0, Swr = 0, Pwr = 0;

int16_t USB_fifo[FIFO_SIZE][FIFO_PACKET_SIZE];


double t = 0, dt = 4.16e-5, w = 2512, ramp = 0, dramp = 1e-4;


void put_samples(int16_t sample)
{
    
    /* t += dt;
    ramp +=dramp;
    if (ramp >= 1)
    {
        ramp = 0;
    }
    double S = ramp*32767*sin(w*t);
    sample = (int16_t)S; */
    USB_fifo [Pwr][Swr] = sample;

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




uint8_t* get_packet()
{
    static uint8_t* tempPtr = NULL;

    tempPtr = (uint8_t*)USB_fifo[(Prd + 60)%(FIFO_SIZE-1)];
    Prd++;
    Prd %= FIFO_SIZE;

    return tempPtr;
}





