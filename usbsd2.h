#ifndef _USBSD2_H_
#define _USBSD2_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/boot.h>
#include "busby.h"
#include "misc.h"
#include "zd2card.h"

static constexpr uint32_t
    M2S_CSW_SIGNATURE = 0x53425355,
    M2S_CBW_SIGNATURE = 0x43425355;

typedef struct
{
    uint32_t Signature;
    uint32_t Tag;
    uint32_t DataTransferLength;
    uint8_t Flags;
    uint8_t LUN;
    uint8_t SCSICommandLength;
    uint8_t SCSICommandData[16];
}
__attribute__ ((packed)) MS_KommandBlockWrapper_t;

typedef struct
{
    uint32_t Signature;
    uint32_t Tag;
    uint32_t DataTransferResidue;
    uint8_t Status;
}
__attribute__ ((packed)) MS_KommandStatusWrapper_t;

class USBSD : public USB
{
private:
    Board _board;
    Sd2Card _sd;
    Endpoint _inpoint;
    Endpoint _outpoint;
    MS_KommandBlockWrapper_t cmdBlock;
    MS_KommandStatusWrapper_t cmdStatus = { .Signature = M2S_CSW_SIGNATURE };
    bool ReadInCommandBlock();
    void DataflashManager_ReadBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks);
    bool SCSI_Command_ReadWrite_10(const bool IsDataRead);
public:
    void ReturnCommandStatus();
    void MassStorage_Task();
    bool decodeSCSICmd();
    void Device_ProcessControlRequest();
    USBSD();
};

#endif



