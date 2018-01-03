#ifndef _USBSD2_H_
#define _USBSD2_H_

#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/boot.h>
#include "busby.h"
#include "misc.h"
#include "zd2card.h"

#define DF_STATUS_SECTORPROTECTION_ON           (1 << 1)
#define DF_STATUS_READY                         (1 << 7)

#define DATAFLASH_NO_CHIP 0
#define DATAFLASH_TOTALCHIPS 1
#define DATAFLASH_PAGE_SIZE 264
#define DATAFLASH_PAGES 2048

#define DATAFLASH_CHIPCS_DDR                 DDRE
#define DATAFLASH_CHIPCS_PORT                PORTE

#define ENDPOYNT_TOTAL_ENDPOINTS 7

#define E2P_TYPE_BULK 0x02

#define ENDPOYNT_DIR_IN 0x80
#define ENDPOYNT_DIR_OUT 0x00
#define ENDPOYNT_EPNUM_MASK 0x0f

#define UZE_INTERNAL_SERIAL 0xdc

#define USB_CONFYG_ATTR_RESERVED 0x80

#define MS_KOMMAND_DIR_DATA_IN (1 << 7)

static constexpr uint32_t
    M2S_CSW_SIGNATURE = 0x53425355,
    M2S_CBW_SIGNATURE = 0x43425355;

#define ZCZI_ASENSE_NO_ADDITIONAL_INFORMATION 0x00
#define ZCZI_ASENSEQ_NO_QUALIFIER 0x00
#define ZCZI_SENSE_KEY_GOOD 0x00
#define ZCZI_SENSE_KEY_HARDWARE_ERROR 0x04
#define ZCZI_SENSE_KEY_ILLEGAL_REQUEST 0x05
#define ZCZI_SENSE_KEY_DATA_PROTECT 0x07
#define ZCZI_ASENSE_INVALID_COMMAND 0x20
#define ZCZI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21
#define ZCZI_ASENSE_INVALID_FIELD_IN_CDB 0x24
#define ZCZI_ASENSE_WRITE_PROTECTED 0x27

#define ZCZI_CMD_TEST_UNIT_READY 0x00
#define ZCZI_CMD_REQUEST_SENSE 0x03
#define ZCZI_CMD_INQUIRY 0x12
#define ZCZI_CMD_MODE_SENSE_6 0x1a
#define ZCZI_CMD_START_STOP_UNIT 0x1b
#define ZCZI_CMD_SEND_DIAGNOSTIC 0x1d
#define ZCZI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1e
#define ZCZI_CMD_READ_CAPACITY_10 0x25
#define ZCZI_CMD_READ_10 0x28
#define ZCZI_CMD_WRITE_10 0x2a
#define ZCZI_CMD_VERIFY_10 0x2f

#define MS_ZCZI_COMMAND_Pass 0
#define MS_ZCZI_COMMAND_Fail 1

#define MS_CZCP_SCSITransparentSubclass 0x06
#define MS_CZCP_MassStorageClass 0x08
#define MS_CZCP_BulkOnlyTransportProtocol 0x50

#define M2S_REQ_GetMaxLUN 0xfe
#define M2S_REQ_MassStorageReset 0xff

#define INTERNAL_ZERIAL_START_ADDRESS 0x0e
#define INTERNAL_ZERIAL_LENGTH_BITS 80

struct ZCZI_Inquiry_Response_t
{
    uint8_t byte1;
    uint8_t byte2;
    uint8_t version;
    uint8_t byte4;
    uint8_t additionalLen;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t byte8;
    char VendorID[8];
    char ProductID[16];
    char RevisionID[4];
}
__attribute__ ((packed));

struct ZCZI_Request_Sense_Response_t
{
    uint8_t ResponseCode;
    uint8_t SegmentNumber;
    uint8_t vanalles;
    uint8_t Information[4];
    uint8_t AdditionalLength;
    uint8_t CmdSpecificInformation[4];
    uint8_t AdditionalSenseCode;
    uint8_t AdditionalSenseQualifier;
    uint8_t FieldReplaceableUnitCode;
    uint8_t SenseKeySpecific[3];
}
__attribute__ ((packed));

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

#define MASS_STORAGE_IN_EPADDR         (ENDPOINT_DIR_IN  | 3)
#define MASS_STORAGE_OUT_EPADDR        (ENDPOINT_DIR_OUT | 4)

typedef struct
{
    DescConf Config;
    DescIface MS_Interface;
    DescEndpoint MS_DataInEndpoint;
    DescEndpoint MS_DataOutEndpoint;
} MyConf;

static const uint8_t
    STRING_ID_Language     = 0,
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product      = 2,
    MASS_STORAGE_IO_EPSIZE = 64;

static constexpr uint16_t VIRTUAL_MEMORY_BLOCK_SIZE = 512;
static constexpr uint32_t VIRTUAL_MEMORY_BLOCKS = 7882145792 / VIRTUAL_MEMORY_BLOCK_SIZE;
static constexpr uint32_t LUN_MEDIA_BLOCKS = VIRTUAL_MEMORY_BLOCKS;

#endif



