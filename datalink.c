#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER  2000
#define MAXSIZE 5//����ʹ���

struct FRAME {
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned int  padding;
};

static unsigned char ack_expected=0,/*���ʹ����±߽�*/  buffer[MAXSIZE + 1][PKT_LEN],nbuffered=0, next_frame_to_send=0/*���ʹ����ϱ߽�*/;//���ͷ�
static unsigned char frame_expected = 0;
static int phl_ready = 0;

static void inc(unsigned char* next_frame_to_send)
{
    *next_frame_to_send = (*next_frame_to_send + 1) % (MAXSIZE + 1);
}



static int between(unsigned char mid, unsigned char low, unsigned char high)
{
    if (low < high)
    {
        if ((mid >= low) && (mid < high))
            return 1;
        else
            return 0;
    }
    else if (low > high)
    {
        if ((mid >= low) || (mid < high))
            return 1;
        else
            return 0;
    }
    else
        return 0;
}
static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}



static void send_data_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = next_frame_to_send;
    s.ack = (MAXSIZE + frame_expected) % (MAXSIZE + 1);

    memcpy(s.data, buffer[next_frame_to_send], PKT_LEN);

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)s.data);

    put_frame((unsigned char*)&s, 3 + PKT_LEN);
    start_timer(next_frame_to_send, DATA_TIMER);

    inc(&next_frame_to_send);
}

static void send_ack_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_ACK;
    // s.ack = 1 - frame_expected;
    s.ack = (frame_expected + MAXSIZE) % (MAXSIZE + 1);

    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame((unsigned char*)&s, 2);
}

int main(int argc, char** argv)
{
    int event, arg;
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv); //protocol_init()��������վ��֮��� TCP ���ӣ������趨ʱ������Ĳο� 0 ��
    lprintf("Designed by Chen Shuyi, build: " __DATE__"  "__TIME__"\n");
    lprintf("DATA_TIMER: %d    MAXSIZE:%d\n",DATA_TIMER,MAXSIZE);

    disable_network_layer();

    for (;;) {
        event = wait_for_event(&arg);

        switch (event) {
        case NETWORK_LAYER_READY://������д����͵ķ���
            get_packet(buffer[next_frame_to_send]);//get_packet(p)���������鿽���� ָ�� p ָ���Ļ������У����ڷ��鳤�Ȳ��ȣ���������ֵΪ���鳤�ȡ�
            nbuffered++;
            send_data_frame();
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char*)&f, sizeof( f));//size Ϊ���ڴ�Ž���֡�Ļ����� buf �Ŀռ��С������ֵΪ �յ�֡��ʵ�ʳ��ȡ�
            if (len < 5 || crc32((unsigned char*)&f, len) != 0) {
                dbg_event("****** Receiver Error, Bad CRC Checksum\n");
                lprintf("********error f.seq: %d \n", f.seq);
                break;//У�������
            }


            if (f.kind == FRAME_DATA) {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
                if (f.seq == frame_expected) {
                    put_packet(f.data, len - 7);//��������������յ�����Ļ������׵�ַ�ͷ��鳤�ȡ�
                    frame_expected = (frame_expected + 1) % (MAXSIZE + 1);
                }              
            }
            while (between(f.ack, ack_expected, next_frame_to_send))
                 {
                     nbuffered--;
                     stop_timer(ack_expected);
                     ack_expected = (ack_expected + 1) % (MAXSIZE + 1);
                 }//�ۼ�ACK,�Ӵ�ACK
                break;

        case DATA_TIMEOUT:
            dbg_event("---- DATA %d timeout\n", arg);
            next_frame_to_send = ack_expected;
            for (int i = 1; i <= nbuffered; i++)
            {
                send_data_frame();
            }

            break;
        }

        if (nbuffered < MAXSIZE && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}
