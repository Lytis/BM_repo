#include "fifo.h"
#include "usbd_audio.h"

int16_t USB_fifo[FIFO_QUEUES][SUB_PACKETS][USB_PACKET_SAMPLES] = {0};
int16_t zero_pad[USB_PACKET_SAMPLES] = {0};

static int packet_wr = 0, packet_rd = 0, sample_wr = 0;

static int fifo_no = 0;
static int stored_queues = 0;
static int stored_packets[FIFO_QUEUES] = {0};
static int pt = 0;


void put_samples(int16_t* rx_buffer)
{

    int i, k, s = 0;

    for (i=0;i<=199;i++)
    {
        for (k=0;k<=23;k++)
        {
            USB_fifo[fifo_no][i][k] = rx_buffer[s];
            s++;
        }
        
    }
    stored_packets[fifo_no]++;

    stored_queues++;
    stored_queues %= FIFO_QUEUES-1;
    fifo_no ++;
    fifo_no %= FIFO_QUEUES-1;

}




uint8_t* get_packet()
{
    uint8_t* tempPtr = NULL;

/*     if (packet_wr != packet_rd)
    {
        tempPtr = (uint8_t *)USB_fifo[packet_rd];
        packet_rd ++;
        if (packet_rd == USB_FIFO_PACKETS)
            packet_rd = 0;
    }else
    {
        tempPtr = (uint8_t *)zero_pad;
    } */


    tempPtr = (uint8_t *)USB_fifo[1][pt];
    pt ++;
    pt %= SUB_PACKETS;

    return tempPtr;
}





