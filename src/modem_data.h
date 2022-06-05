#ifndef MODEM_DATA_H
#define MODEM_DATA_H

#pragma pack(push, 1)

typedef struct
{
    uint8_t packetNumber;
    uint8_t packetId : 7;
    uint8_t endOfPacket : 1;
} Header_t;

typedef struct
{
    Header_t header;
    uint8_t AUV_ID;
    uint8_t cmd;
    uint8_t data[6];
} Modem_M64_t;

#pragma pack(pop)

#endif //MODEM_DATA_H