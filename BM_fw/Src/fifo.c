#include "fifo.h"
#include "usbd_audio.h"

int rd = 0, wr = 0;
int samples = 0;

int16_t USB_fifo[FIFO_SIZE];
int pack = 0, ptr = 0;

int16_t zeroPad[24] = {0};



void put_samples(int16_t sample)
{
    /* 
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) hUsbDeviceFS->pClassData; */

    USB_fifo[wr] = sample;
    wr ++;
    wr %= FIFO_SIZE-1;
    samples ++;
    if (samples == FIFO_SIZE+1)
    {
        samples = FIFO_SIZE;
        rd ++;
        rd %= FIFO_SIZE-1;
    }

}




uint8_t* get_packet()
{
    uint8_t* tempPtr = NULL;


    if (samples >=24)
    {
        tempPtr = (uint8_t*)&USB_fifo[rd];
        rd += 24;
        samples -=24;
        rd %= FIFO_SIZE-1;
    }else
    {
        tempPtr = (uint8_t*)&zeroPad[0];
    }

    return tempPtr;
}





