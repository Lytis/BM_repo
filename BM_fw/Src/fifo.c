#include "fifo.h"
#include "usbd_audio.h"

int16_t USB_fifo[USB_FIFO_PACKETS][USB_PACKET_SAMPLES];
int16_t zero_pad[USB_PACKET_SAMPLES];

int packet_wr = 0, packet_rd = 0, sample_wr = 0, sample_rd = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

static USBD_AUDIO_HandleTypeDef    *fifo;

void fifo_init(void)
{
    fifo = (USBD_AUDIO_HandleTypeDef*)hUsbDeviceFS.pClassData;
}

void put_sample(int16_t sample)
{

    fifo->buffer[fifo->wr_ptr] = (uint8_t)(sample >>8);
    fifo->wr_ptr ++;
    fifo->wr_ptr %= (AUDIO_TOTAL_BUF_SIZE - 1);
    fifo->buffer[fifo->wr_ptr] = (uint8_t)(sample);
    fifo->wr_ptr ++;
    fifo->wr_ptr %= (AUDIO_TOTAL_BUF_SIZE - 1);

    if (fifo->wr_ptr == fifo->rd_ptr)
    {
        fifo->rd_ptr += 2;
        fifo->rd_ptr %= (AUDIO_TOTAL_BUF_SIZE - 1);
    }

}




uint8_t* get_packet()
{
    uint8_t* tempPtr = NULL;

    if (fifo->rd_ptr != fifo->wr_ptr)
    {
        tempPtr = (uint8_t *)&fifo->buffer[fifo->rd_ptr];
        packet_rd += 2;
        packet_rd %= (AUDIO_TOTAL_BUF_SIZE - 1);
    }else
    {
        tempPtr = (uint8_t *)zero_pad;
    }

    return tempPtr;
}





