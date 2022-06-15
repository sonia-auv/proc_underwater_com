#ifndef MODEM_DATA_H
#define MODEM_DATA_H

#pragma pack(push, 1)

typedef struct
{
    uint8_t AUV_ID :7;
    uint8_t rec_send : 1; // 0 = receive 1 = send
    uint8_t cmd;
    uint8_t data[6];
} Modem_M64_t;

#pragma pack(pop)

#endif //MODEM_DATA_H