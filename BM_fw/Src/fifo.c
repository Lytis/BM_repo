#include "fifo.h"
#include "stm32f7xx_hal.h"
#include "main.h"

int16_t USB_fifo[USB_FIFO_PACKETS][USB_PACKET_SAMPLES];
int16_t zero_pad[USB_PACKET_SAMPLES];

int packet_wr = 0, packet_rd = 0, sample_wr = 0, sample_rd = 0;



void put_sample(int16_t sample)
{

    USB_fifo[packet_wr][sample_wr] = sample;

    sample_wr++;
    if (sample_wr >= USB_PACKET_SAMPLES)
    {
        packet_wr ++;
        packet_wr %= (USB_FIFO_PACKETS-1);

        if (packet_wr == packet_rd)
        {
            packet_rd ++;
            packet_rd %= (USB_FIFO_PACKETS - 1);
        }
    }

}




uint8_t* get_packet()
{
    uint8_t* tempPtr = NULL;

    if (packet_rd != packet_wr)
    {
        tempPtr = (uint8_t *)USB_fifo[packet_rd];
        packet_rd ++;
        packet_rd %= (USB_FIFO_PACKETS - 1);
    }else
    {
        tempPtr = (uint8_t *)zero_pad;
    }

    return tempPtr;
}




int packet_samples()
{
    return USB_PACKET_SAMPLES * sizeof(USB_fifo[0]);
}







