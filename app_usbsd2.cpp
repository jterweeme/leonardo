#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/boot.h>
#include "busby.h"
#include "misc.h"

#define DATAFLASH_CHIP1                      (1 << 0)
#define DATAFLASH_CHIP2                      (1 << 1)
#define DF_STATUS_SECTORPROTECTION_ON           (1 << 1)
#define DF_STATUS_READY                         (1 << 7)

#define DATAFLASH_NO_CHIP 0
#define DATAFLASH_TOTALCHIPS 1
#define DATAFLASH_PAGE_SIZE 264
#define DATAFLASH_PAGES 2048

#define DATAFLASH_CHIPCS_MASK                (DATAFLASH_CHIP1 | DATAFLASH_CHIP2)
#define DATAFLASH_CHIPCS_DDR                 DDRE
#define DATAFLASH_CHIPCS_PORT                PORTE

#define DF_MANUFACTURER_ATMEL                   0x1F
#define DF_CMD_BLOCKERASE                       0x50
#define DF_CMD_MAINMEMTOBUFF1                   0x53
#define DF_CMD_MAINMEMTOBUFF2                   0x55
#define DF_CMD_AUTOREWRITEBUFF1                 0x58
#define DF_CMD_AUTOREWRITEBUFF2                 0x59
#define DF_CMD_MAINMEMTOBUFF1COMP               0x60
#define DF_CMD_MAINMEMTOBUFF2COMP               0x61
#define DF_CMD_PAGEERASE                        0x81
#define DF_CMD_MAINMEMPAGETHROUGHBUFF1          0x82
#define DF_CMD_BUFF1TOMAINMEMWITHERASE          0x83
#define DF_CMD_BUFF1WRITE                       0x84
#define DF_CMD_MAINMEMPAGETHROUGHBUFF2          0x85
#define DF_CMD_BUFF2TOMAINMEMWITHERASE          0x86
#define DF_CMD_BUFF2WRITE                       0x87
#define DF_CMD_BUFF1TOMAINMEM                   0x88
#define DF_CMD_BUFF2TOMAINMEM                   0x89
#define DF_CMD_READMANUFACTURERDEVICEINFO       0x9F
#define DF_CMD_WAKEUP                           0xAB
#define DF_CMD_POWERDOWN                        0xB9
#define DF_CMD_BUFF1READ_LF                     0xD1
#define DF_CMD_MAINMEMPAGEREAD                  0xD2
#define DF_CMD_BUFF2READ_LF                     0xD3
#define DF_CMD_GETSTATUS                        0xD7

#define DATAFLASH_CHIP_MASK(index)      KONCAT_EXPANDED(DATAFLASH_CHIP, index)

#define SPI_USE_DOUBLESPEED            (1 << SPE)
#define SPI_SPEED_FCPU_DIV_2           SPI_USE_DOUBLESPEED
#define SPI_SPEED_FCPU_DIV_4           0
#define SPI_SPEED_FCPU_DIV_8           (SPI_USE_DOUBLESPEED | (1 << SPR0))
#define SPI_SPEED_FCPU_DIV_16          (1 << SPR0)
#define SPI_SPEED_FCPU_DIV_32          (SPI_USE_DOUBLESPEED | (1 << SPR1))
#define SPI_SPEED_FCPU_DIV_64          (SPI_USE_DOUBLESPEED | (1 << SPR1) | (1 << SPR0))
#define SPI_SPEED_FCPU_DIV_128         ((1 << SPR1) | (1 << SPR0))
#define SPI_SCK_LEAD_RISING            (0 << CPOL)
#define SPI_SCK_LEAD_FALLING           (1 << CPOL)
#define SPI_SAMPLE_LEADING             (0 << CPHA)
#define SPI_SAMPLE_TRAILING            (1 << CPHA)
#define SPI_ORDER_MSB_FIRST            (0 << DORD)
#define SPI_ORDER_LSB_FIRST            (1 << DORD)
#define SPI_MODE_SLAVE                 (0 << MSTR)
#define SPI_MODE_MASTER                (1 << MSTR)

#define ENDPOYNT_TOTAL_ENDPOINTS 7

#define E2P_TYPE_BULK 0x02

#define ENDPOYNT_DIR_IN 0x80
#define ENDPOYNT_DIR_OUT 0x00
#define ENDPOYNT_EPNUM_MASK 0x0f
#define NO_DEZCRIPTOR 0
#define ENDPOYNT_ATTR_NO_SYNC (0 << 2)
#define ENDPOYNT_USAGE_DATA (0 << 4)

#define ENDPOYNT_READYWAIT_NoError 0
#define ENDPOYNT_READYWAIT_EndpointStalled 1
#define ENDPOYNT_READYWAIT_DeviceDisconnected 2
#define ENDPOYNT_READYWAIT_BusSuspended 3
#define ENDPOYNT_READYWAIT_Timeout 4

#define ENDPOYNT_RWSTREAM_NoError 0
#define ENDPOYNT_RWSTREAM_IncompleteTransfer 5

#define ENDPOYNT_RWCSTREAM_NoError 0
#define ENDPOYNT_RWCSTREAM_HostAborted 1
#define ENDPOYNT_RWCSTREAM_DeviceDisconnected 2
#define ENDPOYNT_RWCSTREAM_BusSuspended 3

#define USB_CONFYG_POWER_MA(mA)           ((mA) >> 1)

#define FYXED_NUM_CONFIGURATIONS 1

#define UZE_INTERNAL_SERIAL 0xdc

#define USB_CONFYG_ATTR_RESERVED 0x80

#define F2EATURE_SELFPOWERED_ENABLED (1 << 0)
#define F2EATURE_REMOTE_WAKEUP_ENABLED (1 << 1)
#define F2EATURE_SEL_EndpointHalt 0x00
#define F2EATURE_SEL_DeviceRemoteWakeup 0x01

#define MS_KOMMAND_DIR_DATA_IN (1 << 7)

#define M2S_CSW_SIGNATURE 0x53425355UL
#define M2S_CBW_SIGNATURE 0x43425355UL

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

#define KONTROL_REQTYPE_RECIPIENT 0x1f

#define R2EQDIR_HOSTTODEVICE (0 << 7)
#define R2EQTYPE_CLASS (1 << 5)
#define R2EQREC_INTERFACE (1 << 0)
#define R2EQTYPE_STANDARD (0 << 5)
#define R2EQREC_ENDPOINT (2 << 0)
#define R2EQDIR_DEVICETOHOST (1<<7)
#define R2EQREC_DEVICE (0 << 0)

#define R2EQ_GetStatus 0
#define R2EQ_ClearFeature 1
#define R2EQ_SetFeature 3
#define R2EQ_SetAddress 5
#define R2EQ_GetDescriptor 6
#define R2EQ_GetConfiguration 8
#define R2EQ_SetConfiguration 9

#define USB_CZCP_NoDeviceClass 0x00
#define USB_CZCP_NoDeviceSubclass 0x00
#define USB_CZCP_NoDeviceProtocol 0x00

#define MS_ZCZI_COMMAND_Pass 0
#define MS_ZCZI_COMMAND_Fail 1

#define MS_CZCP_SCSITransparentSubclass 0x06
#define MS_CZCP_MassStorageClass 0x08
#define MS_CZCP_BulkOnlyTransportProtocol 0x50

#define M2S_REQ_GetMaxLUN 0xfe
#define M2S_REQ_MassStorageReset 0xff

#define INTERNAL_ZERIAL_START_ADDRESS 0x0e
#define INTERNAL_ZERIAL_LENGTH_BITS 80

#define MIN2(x, y)               (((x) < (y)) ? (x) : (y))

#define KONCAT(x, y)            x ## y

#define KONCAT_EXPANDED(x, y)   KONCAT(x, y)

Serial *g_serial;

static inline uint16_t zwapEndian_16(uint16_t word)
{
    return word >> 8 | word << 8;
}

static inline uint32_t zwapEndian_32(const uint32_t dword)
{
    return dword >> 24 & 0xff | dword << 8 & 0xff0000 |
        dword >> 8 & 0xff00 | dword << 24 & 0xff000000;
}

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
    uint8_t Endpoynt_Write_Stream_LE(const void * const buf, uint16_t len, uint16_t* const bp);
    MS_KommandBlockWrapper_t CommandBlock;
    MS_KommandStatusWrapper_t cmdStatus = { .Signature = M2S_CSW_SIGNATURE };
    bool ReadInCommandBlock();
    void DataflashManager_WriteBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks);
    void DataflashManager_ReadBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks);
    bool SCSI_Command_ReadWrite_10(const bool IsDataRead);
    void clearStatusStage();
public:
    void ReturnCommandStatus();
    void MassStorage_Task();
    bool decodeSCSICmd();
    void USB_Device_ProcessControlRequest();
    void usbTask();
    USBSD();
    void gen();
    void com();
};

static inline uint8_t SPI_TransferByte(const uint8_t Byte)
{
    SPDR = Byte;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

static inline void SPI_SendByte(const uint8_t Byte)
{
    SPDR = Byte;
    while (!(SPSR & (1 << SPIF)));
}

static inline uint8_t SPI_ReceiveByte(void)
{
    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

static inline void SPI_Init(const uint8_t SPIOptions)
{
    DDRB  |= (1 << 0);
    PORTB |= (1 << 0);
    DDRB  |=  ((1 << 1) | (1 << 2));
    DDRB  &= ~(1 << 3);
    PORTB |=  (1 << 3);

    if (SPIOptions & SPI_USE_DOUBLESPEED)
        SPSR |= (1 << SPI2X);
    else
        SPSR &= ~(1 << SPI2X);

    DDRB &= ~(1 << 0);

    SPCR  = ((1 << SPE) | SPIOptions);
}

static inline void Dataflash_SendByte(const uint8_t Byte)
{
    SPI_SendByte(Byte);
}

static inline void Dataflash_SelectChip(const uint8_t ChipMask)
{
    DATAFLASH_CHIPCS_PORT = ((DATAFLASH_CHIPCS_PORT | DATAFLASH_CHIPCS_MASK) & ~ChipMask);
}

static inline void Dataflash_Init(void)
{
    DATAFLASH_CHIPCS_DDR  |= DATAFLASH_CHIPCS_MASK;
    DATAFLASH_CHIPCS_PORT |= DATAFLASH_CHIPCS_MASK;

    SPI_Init(SPI_SPEED_FCPU_DIV_2 | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_FALLING | SPI_SAMPLE_TRAILING | SPI_MODE_MASTER);
}

static inline void Dataflash_DeselectChip(void)
{
    Dataflash_SelectChip(DATAFLASH_NO_CHIP);
}

static inline uint8_t Dataflash_GetSelectedChip(void)
{
    return (~DATAFLASH_CHIPCS_PORT & DATAFLASH_CHIPCS_MASK);
}

static inline uint8_t Dataflash_ReceiveByte(void)
{
    return SPI_ReceiveByte();
}

static inline void Dataflash_ToggleSelectedChipCS(void)
{
    uint8_t SelectedChipMask = Dataflash_GetSelectedChip();

    Dataflash_DeselectChip();
    Dataflash_SelectChip(SelectedChipMask);
}


static inline void Dataflash_SendAddressBytes(uint16_t PageAddress,
                                                          const uint16_t BufferByte)
{
    Dataflash_SendByte(PageAddress >> 5);
    Dataflash_SendByte((PageAddress << 3) | (BufferByte >> 8));
    Dataflash_SendByte(BufferByte);
}

static inline void Dataflash_WaitWhileBusy(void)
{
    Dataflash_ToggleSelectedChipCS();
    Dataflash_SendByte(DF_CMD_GETSTATUS);
    while (!(Dataflash_ReceiveByte() & DF_STATUS_READY));
    Dataflash_ToggleSelectedChipCS();
}

static inline void Dataflash_SelectChipFromPage(const uint16_t PageAddress)
{
    Dataflash_DeselectChip();

    if (PageAddress >= (DATAFLASH_PAGES * DATAFLASH_TOTALCHIPS))
        return;

    Dataflash_SelectChip(DATAFLASH_CHIP1);
}

#define MASS_STORAGE_IN_EPADDR         (ENDPOYNT_DIR_IN  | 3)
#define MASS_STORAGE_OUT_EPADDR        (ENDPOYNT_DIR_OUT | 4)

#define MASS_STORAGE_IO_EPSIZE         64

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
    STRING_ID_Product      = 2;

#define VIRTUAL_MEMORY_BYTES ((uint32_t)DATAFLASH_PAGES * DATAFLASH_PAGE_SIZE * DATAFLASH_TOTALCHIPS)


#define VIRTUAL_MEMORY_BLOCK_SIZE           512
#define VIRTUAL_MEMORY_BLOCKS               (VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE)
#define LUN_MEDIA_BLOCKS                    (VIRTUAL_MEMORY_BLOCKS / TOTAL_LUNS)

uint8_t USB_Device_ConfigurationNumber;
bool USB_Device_CurrentlySelfPowered;
bool USB_Device_RemoteWakeupEnabled;

#define TOTAL_LUNS                1
#define DISK_READ_ONLY            false

const DescDev PROGMEM DeviceDescriptor =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },

    0x0110,
    USB_CZCP_NoDeviceClass,
    USB_CZCP_NoDeviceSubclass,
    USB_CZCP_NoDeviceProtocol,
    8,
    0x03EB,
    0x2045,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    UZE_INTERNAL_SERIAL,
    FYXED_NUM_CONFIGURATIONS
};


volatile bool IsMassStoreReset = false;

static constexpr bool DATA_READ = true, DATA_WRITE = false;
static constexpr uint8_t DEVICE_TYPE_BLOCK = 0;

const MyConf PROGMEM myConf =
{
    .Config =
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        .TotalConfigurationSize = sizeof(MyConf),
        .TotalInterfaces        = 1,
        .ConfigurationNumber    = 1,
        .ConfigurationStrIndex  = 0,    // no descriptor
        .ConfigAttributes       = USB_CONFYG_ATTR_RESERVED,
        .MaxPowerConsumption    = USB_CONFYG_POWER_MA(100)
    },

    .MS_Interface =
    {
        .header =
        {
            .size = sizeof(DescIface),
            .type = DTYPE_Interface
        },

        .InterfaceNumber   = 0, // mass storage
        .AlternateSetting  = 0,
        .TotalEndpoints    = 2,
        .Class             = MS_CZCP_MassStorageClass,
        .SubClass          = MS_CZCP_SCSITransparentSubclass,
        .Protocol          = MS_CZCP_BulkOnlyTransportProtocol,
        .InterfaceStrIndex = 0
    },

    .MS_DataInEndpoint =
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        .EndpointAddress        = MASS_STORAGE_IN_EPADDR,
        .Attributes             = E2P_TYPE_BULK | ENDPOYNT_ATTR_NO_SYNC | ENDPOYNT_USAGE_DATA,
        .EndpointSize           = MASS_STORAGE_IO_EPSIZE,
        .PollingIntervalMS      = 0x05
    },

    .MS_DataOutEndpoint =
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        MASS_STORAGE_OUT_EPADDR,
        E2P_TYPE_BULK | ENDPOYNT_ATTR_NO_SYNC | ENDPOYNT_USAGE_DATA,
        MASS_STORAGE_IO_EPSIZE,
        0x05
    }
};

const USB_Descriptor_String_t<2> PROGMEM LanguageString =
{
    {
        USB_STRING_LEN(1),
        DTYPE_String
    },
    (wchar_t)0x0409
};

const USB_Descriptor_String_t<12> PROGMEM ManufacturerString =
{
    {
        USB_STRING_LEN(11),
        DTYPE_String
    },
    L"Dean Camera"
};

const USB_Descriptor_String_t<23> PROGMEM ProductString =
{
    {
        USB_STRING_LEN(22),
        DTYPE_String
    },
    L"LUFA Mass Storage Demo"
};

static uint16_t getDescriptor(uint16_t wValue, uint16_t wIndex, const void** const descAddr)
{
    const uint8_t DescriptorType = wValue >> 8;
    const uint8_t DescriptorNumber = wValue & 0xFF;
    const void *Address = NULL;
    uint16_t Size = 0;

    switch (DescriptorType)
    {
    case DTYPE_Device:
        Address = &DeviceDescriptor;
        Size = sizeof(DescDev);
        break;
    case DTYPE_Configuration:
        Address = &myConf;
        Size = sizeof(MyConf);
        break;
    case DTYPE_String:
        switch (DescriptorNumber)
        {
        case STRING_ID_Language:
            Address = &LanguageString;
            Size = pgm_read_byte(&LanguageString.Header.size);
            break;
        case STRING_ID_Manufacturer:
            Address = &ManufacturerString;
            Size = pgm_read_byte(&ManufacturerString.Header.size);
            break;
        case STRING_ID_Product:
            Address = &ProductString;
            Size = pgm_read_byte(&ProductString.Header.size);
            break;
        }
        break;
    }

    *descAddr = Address;
    return Size;
}

static const ZCZI_Inquiry_Response_t InquiryData =
{
    DEVICE_TYPE_BLOCK<<3 | 0,
    1,  // removable
    0,  // version
    2<<4,
    0x1f,   // additionalLength
    0,
    0,  // reserved1
    0,  // reserved2
    "LUFA",
    "Dataflash Disk",
    {'0','.','0','0'},
};

static ZCZI_Request_Sense_Response_t SenseData =
{
    0x70,   // response code
    0, // seg no
    0, // van alles
    {0,0,0,0},
    0x0A
};

static bool DataflashManager_CheckDataflashOperation(void)
{
    uint8_t ReturnByte;
    Dataflash_SelectChip(DATAFLASH_CHIP1);
    Dataflash_SendByte(DF_CMD_READMANUFACTURERDEVICEINFO);
    ReturnByte = Dataflash_ReceiveByte();
    Dataflash_DeselectChip();

    if (ReturnByte != DF_MANUFACTURER_ATMEL)
        return false;

    Dataflash_SelectChip(DATAFLASH_CHIP2);
    Dataflash_SendByte(DF_CMD_READMANUFACTURERDEVICEINFO);
    ReturnByte = Dataflash_ReceiveByte();
    Dataflash_DeselectChip();

    if (ReturnByte != DF_MANUFACTURER_ATMEL)
        return false;

    return true;
}

uint8_t USBSD::Endpoynt_Write_Stream_LE(const void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode = Endpoint_WaitUntilReady();

    if (ErrorCode)
        return ErrorCode;

    if (BytesProcessed != NULL)
    {
        Length -= *BytesProcessed;
        DataStream += *BytesProcessed;
    }

    while (Length)
    {
        if ((UEINTX & 1<<RWAL) == 0)
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            //usbTask();

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOYNT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = Endpoint_WaitUntilReady()))
              return ErrorCode;
        }
        else
        {
            write8(*DataStream);
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOYNT_RWSTREAM_NoError;
}



bool USBSD::decodeSCSICmd()
{
    bool CommandSuccess = false;

    switch (CommandBlock.SCSICommandData[0])
    {
        case ZCZI_CMD_INQUIRY:
        {
            g_serial->write("Inquiry\r\n");
            uint16_t allocLen = zwapEndian_16(*(uint16_t*)&CommandBlock.SCSICommandData[3]);
            uint16_t BytesTransferred = MIN2(allocLen, sizeof(InquiryData));

            if ((CommandBlock.SCSICommandData[1] & (1<<0 | 1<<1)) ||
                CommandBlock.SCSICommandData[2])
            {
                SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
                SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_FIELD_IN_CDB;
                SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
                break;
            }

            Endpoynt_Write_Stream_LE(&InquiryData, BytesTransferred, NULL);
            nullStream(allocLen - BytesTransferred, NULL);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            CommandBlock.DataTransferLength -= BytesTransferred;
            CommandSuccess = true;
        }
            break;
        case ZCZI_CMD_REQUEST_SENSE:
        {
            g_serial->write("Request sense\r\n");
            uint8_t AllocationLength = CommandBlock.SCSICommandData[4];
            uint8_t BytesTransferred = MIN2(AllocationLength, sizeof(SenseData));
            Endpoynt_Write_Stream_LE(&SenseData, BytesTransferred, NULL);
            nullStream(AllocationLength - BytesTransferred, NULL);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            CommandBlock.DataTransferLength -= BytesTransferred;
            CommandSuccess = true;
        }
            break;
        case ZCZI_CMD_READ_CAPACITY_10:
            g_serial->write("Read capacity\r\n");
            write32be(LUN_MEDIA_BLOCKS - 1);
            write32be(VIRTUAL_MEMORY_BLOCK_SIZE);

            if (IsMassStoreReset)
                break;

            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            CommandBlock.DataTransferLength -= 8;
            CommandSuccess = true;
            break;
        case ZCZI_CMD_SEND_DIAGNOSTIC:
            g_serial->write("cmd send diagnostic\r\n");

            if ((CommandBlock.SCSICommandData[1] & 1<<2) == 0)
            {
                SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
                SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_FIELD_IN_CDB;
                SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
                break;
            }

            if (DataflashManager_CheckDataflashOperation() == 0)
            {
                SenseData.vanalles = ZCZI_SENSE_KEY_HARDWARE_ERROR << 4;
                SenseData.AdditionalSenseCode = ZCZI_ASENSE_NO_ADDITIONAL_INFORMATION;
                SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
                break;
            }

            CommandBlock.DataTransferLength = 0;
            CommandSuccess = true;
            break;
        case ZCZI_CMD_WRITE_10:
            g_serial->write("cmd_write_10\r\n");
            CommandSuccess = SCSI_Command_ReadWrite_10(DATA_WRITE);
            break;
        case ZCZI_CMD_READ_10:
            g_serial->write("cmd_read_10\r\n");
            CommandSuccess = SCSI_Command_ReadWrite_10(DATA_READ);
            break;
        case ZCZI_CMD_MODE_SENSE_6:
            g_serial->write("cmd_mode_sense_6\r\n");
            write8(0x00);
            write8(0x00);
            write8(DISK_READ_ONLY ? 0x80 : 0x00);
            write8(0x00);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            CommandBlock.DataTransferLength -= 4;
            CommandSuccess = true;
            break;
        case ZCZI_CMD_START_STOP_UNIT:
        case ZCZI_CMD_TEST_UNIT_READY:
        case ZCZI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case ZCZI_CMD_VERIFY_10:
            g_serial->write("cmd_verify_10 en anderen\r\n");
            CommandSuccess = true;
            CommandBlock.DataTransferLength = 0;
            break;
        default:
            SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
            SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
            break;
    }

    if (CommandSuccess)
    {
        SenseData.vanalles = ZCZI_SENSE_KEY_GOOD << 4;
        SenseData.AdditionalSenseCode = ZCZI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
        return true;
    }

    return false;
}

bool USBSD::SCSI_Command_ReadWrite_10(const bool IsDataRead)
{
    uint32_t BlockAddress;
    uint16_t totalBlocks;

    if ((IsDataRead == DATA_WRITE) && DISK_READ_ONLY)
    {
        SenseData.vanalles = ZCZI_SENSE_KEY_DATA_PROTECT << 4;
        SenseData.AdditionalSenseCode = ZCZI_ASENSE_WRITE_PROTECTED;
        SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
        return false;
    }

    BlockAddress = zwapEndian_32(*(uint32_t*)&CommandBlock.SCSICommandData[2]);
    totalBlocks = zwapEndian_16(*(uint16_t*)&CommandBlock.SCSICommandData[7]);

    if (BlockAddress >= LUN_MEDIA_BLOCKS)
    {
        SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
        SenseData.AdditionalSenseCode = ZCZI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
        return false;
    }

    char buf[50];
    snprintf(buf, 50, "%u ", BlockAddress);
    g_serial->write(buf);
    snprintf(buf, 50, "%u\r\n", totalBlocks);
    g_serial->write(buf);

    if (IsDataRead == DATA_READ)
        DataflashManager_ReadBlocks(BlockAddress, totalBlocks);
    else
        DataflashManager_WriteBlocks(BlockAddress, totalBlocks);

    CommandBlock.DataTransferLength -= ((uint32_t)totalBlocks * VIRTUAL_MEMORY_BLOCK_SIZE);
    return true;
}

void USBSD::DataflashManager_WriteBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks)
{
    uint16_t CurrDFPage = (BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) / DATAFLASH_PAGE_SIZE;
    uint16_t CurrDFPageByte = (BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) % DATAFLASH_PAGE_SIZE;
    uint8_t  CurrDFPageByteDiv16 = CurrDFPageByte >> 4;
    bool     UsingSecondBuffer   = false;
    Dataflash_SelectChipFromPage(CurrDFPage);
    Dataflash_SendByte(DF_CMD_MAINMEMTOBUFF1);
    Dataflash_SendAddressBytes(CurrDFPage, 0);
    Dataflash_WaitWhileBusy();
    Dataflash_SendByte(DF_CMD_BUFF1WRITE);
    Dataflash_SendAddressBytes(0, CurrDFPageByte);

    if (Endpoint_WaitUntilReady())
        return;

    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
            if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
            {
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out

                if (Endpoint_WaitUntilReady())
                    return;
            }

            if (CurrDFPageByteDiv16 == (DATAFLASH_PAGE_SIZE >> 4))
            {
                Dataflash_WaitWhileBusy();

                if (UsingSecondBuffer)
                    Dataflash_SendByte(DF_CMD_BUFF2TOMAINMEMWITHERASE);
                else
                    Dataflash_SendByte(DF_CMD_BUFF1TOMAINMEMWITHERASE);

                Dataflash_SendAddressBytes(CurrDFPage, 0);
                CurrDFPageByteDiv16 = 0;
                CurrDFPage++;

                if (Dataflash_GetSelectedChip() == DATAFLASH_CHIP_MASK(DATAFLASH_TOTALCHIPS))
                    UsingSecondBuffer = !(UsingSecondBuffer);

                Dataflash_SelectChipFromPage(CurrDFPage);

                if ((TotalBlocks * (VIRTUAL_MEMORY_BLOCK_SIZE >> 4)) < (DATAFLASH_PAGE_SIZE >> 4))
                {
                    Dataflash_WaitWhileBusy();
                    Dataflash_SendByte(UsingSecondBuffer ? DF_CMD_MAINMEMTOBUFF2 : DF_CMD_MAINMEMTOBUFF1);
                    Dataflash_SendAddressBytes(CurrDFPage, 0);
                    Dataflash_WaitWhileBusy();
                }

                Dataflash_SendByte(UsingSecondBuffer ? DF_CMD_BUFF2WRITE : DF_CMD_BUFF1WRITE);
                Dataflash_SendAddressBytes(0, 0);
            }

            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            Dataflash_SendByte(read8());
            CurrDFPageByteDiv16++;
            BytesInBlockDiv16++;

            if (IsMassStoreReset)
                return;
        }

        TotalBlocks--;
    }

    Dataflash_WaitWhileBusy();

    if (UsingSecondBuffer)
        Dataflash_SendByte(DF_CMD_BUFF2TOMAINMEMWITHERASE);
    else
        Dataflash_SendByte(DF_CMD_BUFF1TOMAINMEMWITHERASE);

    Dataflash_SendAddressBytes(CurrDFPage, 0x00);
    Dataflash_WaitWhileBusy();

    if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out

    Dataflash_DeselectChip();
}

void USBSD::DataflashManager_ReadBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks)
{
#if 1
    uint16_t CurrDFPage = ((BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) / DATAFLASH_PAGE_SIZE);
    uint16_t CurrDFPageByte = ((BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) % DATAFLASH_PAGE_SIZE);
    uint8_t  CurrDFPageByteDiv16 = (CurrDFPageByte >> 4);
    Dataflash_SelectChipFromPage(CurrDFPage);
    Dataflash_SendByte(DF_CMD_MAINMEMPAGEREAD);
    Dataflash_SendAddressBytes(CurrDFPage, CurrDFPageByte);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);
#endif

    if (Endpoint_WaitUntilReady())
        return;

#if 0
    for (uint16_t k = 0; k < TotalBlocks; k++)
        for (uint8_t j = 0; j < 32; j++)
            for (uint8_t i = 0; i < 16; i++)
                write8(i);
#endif
#if 1
    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
            if ((UEINTX & 1<<RWAL) == 0)    //read-write allowed?
            {
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

                if (Endpoint_WaitUntilReady())
                    return;
            }

            if (CurrDFPageByteDiv16 == (DATAFLASH_PAGE_SIZE >> 4))
            {
                CurrDFPageByteDiv16 = 0;
                CurrDFPage++;
                Dataflash_SelectChipFromPage(CurrDFPage);
                Dataflash_SendByte(DF_CMD_MAINMEMPAGEREAD);
                Dataflash_SendAddressBytes(CurrDFPage, 0);
                Dataflash_SendByte(0x00);
                Dataflash_SendByte(0x00);
                Dataflash_SendByte(0x00);
                Dataflash_SendByte(0x00);
            }

            for (uint8_t i = 0; i < 16; i++)
                //write8(Dataflash_ReceiveByte());
                write8(i);

            CurrDFPageByteDiv16++;
            BytesInBlockDiv16++;

            if (IsMassStoreReset)
                return;
        }

        TotalBlocks--;
    }
#endif
    if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

    Dataflash_DeselectChip();
}

static inline void Endpoynt_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOYNT_EPNUM_MASK));
    UERST = 0;
}

void USBSD::MassStorage_Task()
{
	if (state != DEVICE_STATE_Configured)
	    return;

    if (ReadInCommandBlock())
    {
        if (CommandBlock.Flags & MS_KOMMAND_DIR_DATA_IN)
            selectEndpoint(MASS_STORAGE_IN_EPADDR);

		cmdStatus.Status = decodeSCSICmd() ? MS_ZCZI_COMMAND_Pass : MS_ZCZI_COMMAND_Fail;
		cmdStatus.Tag = CommandBlock.Tag;
		cmdStatus.DataTransferResidue = CommandBlock.DataTransferLength;

		if ((cmdStatus.Status == MS_ZCZI_COMMAND_Fail) && cmdStatus.DataTransferResidue)
		    UECONX |= 1<<STALLRQ;   // stall transaction

		ReturnCommandStatus();
	}

	if (IsMassStoreReset)
	{
        Endpoynt_ResetEndpoint(MASS_STORAGE_OUT_EPADDR);
        Endpoynt_ResetEndpoint(MASS_STORAGE_IN_EPADDR);
        selectEndpoint(MASS_STORAGE_OUT_EPADDR);
        UECONX |= 1<<STALLRQC;  // clear stall
        UECONX |= 1<<RSTDT; // reset data toggle
		selectEndpoint(MASS_STORAGE_IN_EPADDR);
        UECONX |= 1<<STALLRQC;  // clear stall
        UECONX |= 1<<RSTDT; // reset data toggle
		IsMassStoreReset = false;
	}
}

bool USBSD::ReadInCommandBlock()
{
	uint16_t BytesTransferred;
	selectEndpoint(MASS_STORAGE_OUT_EPADDR);

	if ((UEINTX & 1<<RXOUTI) == 0)  // is out received?
	    return false;

	BytesTransferred = 0;

	while (readStream(&CommandBlock, (sizeof(CommandBlock) -
        sizeof(CommandBlock.SCSICommandData)),
	    &BytesTransferred) == ENDPOYNT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return false;
	}

	if ((CommandBlock.Signature         != M2S_CBW_SIGNATURE) ||
	    (CommandBlock.LUN               >= TOTAL_LUNS)       ||
		(CommandBlock.Flags              & 0x1F)             ||
		(CommandBlock.SCSICommandLength == 0)                ||
		(CommandBlock.SCSICommandLength >  sizeof(CommandBlock.SCSICommandData)))
	{
        UECONX |= 1<<STALLRQ;
		selectEndpoint(MASS_STORAGE_IN_EPADDR);
        UECONX |= 1<<STALLRQ;       // stall transaction
		return false;
	}

	BytesTransferred = 0;

	while (readStream(&CommandBlock.SCSICommandData, CommandBlock.SCSICommandLength,
	                               &BytesTransferred) == ENDPOYNT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return false;
	}

    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
	return true;
}

void USBSD::ReturnCommandStatus()
{
	uint16_t BytesTransferred;
	selectEndpoint(MASS_STORAGE_OUT_EPADDR);

	while (UECONX & 1<<STALLRQ)
		if (IsMassStoreReset)
		    return;

	selectEndpoint(MASS_STORAGE_IN_EPADDR);

	while (UECONX & 1<<STALLRQ)
		if (IsMassStoreReset)
		    return;

	BytesTransferred = 0;

	while (Endpoynt_Write_Stream_LE(&cmdStatus, sizeof(cmdStatus),
	                                &BytesTransferred) == ENDPOYNT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return;
	}

    UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
}

void USBSD::clearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & R2EQDIR_DEVICETOHOST)
    {
        while ((UEINTX & 1<<RXOUTI) == 0)   // out received?
        {
            if (state == DEVICE_STATE_Unattached)
                return;
        }

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
    }
    else
    {
        while ((UEINTX & 1<<TXINI) == 0)    // in ready?
        {
            if (state == DEVICE_STATE_Unattached)
                return;
        }

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
    }
}

static void Device_GetSerialString(uint16_t *UnicodeString)
{
    uint8_t CurrentGlobalInt = SREG;
    cli();
    uint8_t SigReadAddress = INTERNAL_ZERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0;
        SerialCharNum < INTERNAL_ZERIAL_LENGTH_BITS / 4; SerialCharNum++)
    {
        uint8_t SerialByte = boot_signature_byte_get(SigReadAddress);

        if (SerialCharNum & 0x01)
        {
            SerialByte >>= 4;
            SigReadAddress++;
        }

        SerialByte &= 0x0F;

        UnicodeString[SerialCharNum] = SerialByte >= 10 ?
            ('A' - 10) + SerialByte : '0' + SerialByte;
    }

    __asm__ __volatile__("" ::: "memory");
    SREG = CurrentGlobalInt;
    __asm__ __volatile__("" ::: "memory");
}

static inline void setDeviceAddress(const uint8_t Address)
{
    UDADDR = (UDADDR & (1 << ADDEN)) | (Address & 0x7F);
}

void USBSD::USB_Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        *(RequestHeader++) = read8();

    if (UEINTX & 1<<RXSTPI)
    {
        const uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

        switch (USB_ControlRequest.bRequest)
        {
        case M2S_REQ_MassStorageReset:
            if (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_CLASS | R2EQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                IsMassStoreReset = true;
            }

            break;
        case M2S_REQ_GetMaxLUN:
            if (bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_CLASS | R2EQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(TOTAL_LUNS - 1);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }
            break;
        case R2EQ_GetStatus:
            if ((bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_DEVICE)) ||
                (bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (USB_ControlRequest.bmRequestType)
                {
                case (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_DEVICE):
                {
                    if (USB_Device_CurrentlySelfPowered)
                        CurrentStatus |= F2EATURE_SELFPOWERED_ENABLED;

                    if (USB_Device_RemoteWakeupEnabled)
                        CurrentStatus |= F2EATURE_REMOTE_WAKEUP_ENABLED;
                }
                    break;
                case (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_ENDPOINT):
                {
                    uint8_t EndpointIndex = ((uint8_t)USB_ControlRequest.wIndex &
                            ENDPOYNT_EPNUM_MASK);

                    if (EndpointIndex >= ENDPOYNT_TOTAL_ENDPOINTS)
                        return;

                    selectEndpoint(EndpointIndex);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    selectEndpoint(0);
                }
                    break;
                default:
                    return;
                }

                UEINTX &= ~(1<<RXSTPI); // clear setup
                write16(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }

            break;
        case R2EQ_ClearFeature:
        case R2EQ_SetFeature:
            if ((bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_DEVICE)) ||
                (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_ENDPOINT)))
            {
                Device_ClearSetFeature();
            }

            break;
        case R2EQ_SetAddress:
            if (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_DEVICE))
            {
                uint8_t DeviceAddress = (USB_ControlRequest.wValue & 0x7F);
                setDeviceAddress(DeviceAddress);
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                while ((UEINTX & 1<<TXINI) == 0);   // in ready?
                UDADDR |= 1<<ADDEN; // enable dev addr
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case R2EQ_GetDescriptor:
            if ((bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_DEVICE)) ||
                (bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_INTERFACE)))
            {
                const void* DescriptorPointer;
                uint16_t descSize;

                if (USB_ControlRequest.wValue == (DTYPE_String << 8 | UZE_INTERNAL_SERIAL))
                {
                    struct
                    {
                        DescHeader header;
                        uint16_t UnicodeString[INTERNAL_ZERIAL_LENGTH_BITS / 4];
                    } sigDesc;

                    sigDesc.header.type = DTYPE_String;
                    sigDesc.header.size = USB_STRING_LEN(INTERNAL_ZERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
                      USB_ControlRequest.wIndex, &DescriptorPointer)) == 0)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write_Control_PStream_LE(DescriptorPointer, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
            }

            break;
        case R2EQ_GetConfiguration:
            if (bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }
            break;
        case R2EQ_SetConfiguration:
            if (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_DEVICE))
            {
                if ((uint8_t)USB_ControlRequest.wValue > FYXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
                clearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                configureEndpoint(MASS_STORAGE_IN_EPADDR,  E2P_TYPE_BULK, MASS_STORAGE_IO_EPSIZE, 1);
                configureEndpoint(MASS_STORAGE_OUT_EPADDR, E2P_TYPE_BULK, MASS_STORAGE_IO_EPSIZE, 1);
                UDIEN |= 1<<SOFE;
            }
            break;
        default:
            break;
        }
    }

    if (UEINTX & 1<<RXSTPI)
    {
        UEINTX &= ~(1<<RXSTPI);     // clear setup
        UECONX |= 1<<STALLRQ;       // stall transaction
    }
}

void USBSD::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);    // clear int sof (start of frame)
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = 1<<PINDIV;
            PLLCSR = 1<<PINDIV | 1<<PLLE;
            while ((PLLCSR & 1<<PLOCK) == 0);   // pll is ready?
            state = DEVICE_STATE_Powered;
            IsMassStoreReset = false;
        }
        else
        {
            PLLCSR = 0;     // pll off
            state = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);   // disable int susp
        UDIEN |= 1<<WAKEUPE;    // enable int wakeup
        USBCON |= 1<<FRZCLK;    // clk freeze
        PLLCSR = 0; // pll off
        state = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = 1<<PINDIV;
        PLLCSR = 1<<PINDIV | 1<<PLLE;
        while ((PLLCSR & 1<<PLOCK) == 0);
        USBCON &= ~(1<<FRZCLK); // clk unfreeze
        UDINT &= ~(1<<WAKEUPI); // clear int wakeupi
        UDIEN &= ~(1<<WAKEUPE); // disable int wakeupi
        UDIEN |= 1<<SUSPE;      // enable int suspi

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Addressed : DEVICE_STATE_Powered;

        IsMassStoreReset = false;
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);  // clear int eorst
        state = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);   // clear int susp
        UDIEN &= ~(1<<SUSPE);   // disable int susp
        UDIEN |= 1<<WAKEUPE;    // enable int wakeup
        configureEndpoint(0, 0, 8, 1);
        UEIENX |= 1<<RXSTPE;    // enable int rxstpi
    }
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    USB::instance->gen();
}

void USBSD::com()
{
    uint8_t PrevSelectedEndpoint = getCurrentEndpoint();
    selectEndpoint(0);
    UEIENX &= ~(1<<RXSTPE); // disable int rxstp
    sei();
    USB_Device_ProcessControlRequest();
    selectEndpoint(0);
    UEIENX |= 1<<RXSTPE;    // enable int rxstp
    selectEndpoint(PrevSelectedEndpoint);
}

ISR(USB_COM_vect, ISR_BLOCK)
{
    USB::instance->com();
}

void USBSD::usbTask()
{
    if (state == DEVICE_STATE_Unattached)
        return;

    uint8_t prevEndp = getCurrentEndpoint();
    selectEndpoint(0);
    
    if (UEINTX & 1<<RXSTPI) // setup received?
        USB_Device_ProcessControlRequest();

    selectEndpoint(prevEndp);
}

USBSD::USBSD()
{
    USBCON &= ~(1<<OTGPADE);
    UHWCON |= 1<<UVREGE;
    PLLFRQ = (1 << PDIV2);
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1 << USBE);
    USBCON |=  (1 << USBE);
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0; // pll off
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM); // full speed
    USBCON |= 1<<VBUSTE;
    configureEndpoint(0, 0, 8, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);  // attach
    USBCON |= 1<<OTGPADE;
}

int main(void)
{
    Serial serial;
    g_serial = &serial;
    serial.init();
    serial.write("onzin\r\n");
    Dataflash_Init();
    USBSD usbsd;

    if (!(DataflashManager_CheckDataflashOperation()))
    {
    }

    sei();

	for (;;)
	{
        usbsd.MassStorage_Task();
        usbsd.usbTask();
	}
}






