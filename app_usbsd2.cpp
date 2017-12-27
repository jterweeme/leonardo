#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/boot.h>

#define DATAFLASH_CHIP1                      (1 << 0)
#define DATAFLASH_CHIP2                      (1 << 1)
#define DF_STATUS_SECTORPROTECTION_ON           (1 << 1)
#define DF_STATUS_READY                         (1 << 7)

#define DATAFLASH_NO_CHIP                    0
#define DATAFLASH_TOTALCHIPS                 2
#define DATAFLASH_PAGE_SIZE                  1024
#define DATAFLASH_PAGES                      8192

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

#define DF_CMD_BINARYPAGESIZEMODEON             ((char[]){0x3D, 0x2A, 0x80, 0xA6})
#define DF_CMD_BINARYPAGESIZEMODEON_BYTE1       0x3D
#define DF_CMD_BINARYPAGESIZEMODEON_BYTE2       0x2A
#define DF_CMD_BINARYPAGESIZEMODEON_BYTE3       0x80
#define DF_CMD_BINARYPAGESIZEMODEON_BYTE4       0xA6

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

#define USB_DevizeState GPIOR0

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

#define D2TYPE_Device 0x01
#define D2TYPE_Configuration 0x02
#define D2TYPE_String 0x03
#define D2TYPE_Interface 0x04
#define D2TYPE_Endpoint 0x05
#define D2TYPE_DeviceQualifier 0x06
#define D2TYPE_Other 0x07
#define D2TYPE_InterfacePower 0x08

#define USB_CZCP_NoDeviceClass 0x00
#define USB_CZCP_NoDeviceSubclass 0x00
#define USB_CZCP_NoDeviceProtocol 0x00

#define MS_ZCZI_COMMAND_Pass 0
#define MS_ZCZI_COMMAND_Fail 1

#define MS_CZCP_SCSITransparentSubclass 0x06
#define MS_CZCP_MassStorageClass 0x08
#define MS_CZCP_BulkOnlyTransportProtocol 0x50

#define DEVIZE_STATE_Unattached 0
#define DEVIZE_STATE_Powered 1
#define DEVIZE_STATE_Default 2
#define DEVIZE_STATE_Addressed 3
#define DEVIZE_STATE_Configured 4
#define DEVIZE_STATE_Suspended 5

#define M2S_REQ_GetMaxLUN 0xfe
#define M2S_REQ_MassStorageReset 0xff

#define INTERNAL_ZERIAL_START_ADDRESS 0x0e
#define INTERNAL_ZERIAL_LENGTH_BITS 80

#define LANGUAGE_ID_ENG2 0x0409

#define MIN2(x, y)               (((x) < (y)) ? (x) : (y))

#define KONCAT(x, y)            x ## y

#define KONCAT_EXPANDED(x, y)   KONCAT(x, y)

#define USB_ZTRING_LEN(UnicodeChars) (sizeof(USB_Dezcriptor_Header_t) + ((UnicodeChars) << 1))

#define USB_STRING_DEZCRIPTOR(String)     { .Header = {.Size = sizeof(USB_Dezcriptor_Header_t) + (sizeof(String) - 2), .Type = D2TYPE_String}, .UnicodeString = String }

#define USB_STRING_DEZCRIPTOR_ARRAY(...)  { .Header = {.Size = sizeof(USB_Dezcriptor_Header_t) + sizeof((uint16_t){__VA_ARGS__}), .Type = D2TYPE_String}, .UnicodeString = {__VA_ARGS__} }

static inline uint16_t zwapEndian_16(const uint16_t Word)
{
    uint8_t Temp;

    union
    {
        uint16_t Word;
        uint8_t  Bytes[2];
    } Data;

    Data.Word = Word;

    Temp = Data.Bytes[0];
    Data.Bytes[0] = Data.Bytes[1];
    Data.Bytes[1] = Temp;

    return Data.Word;
}

static inline uint32_t zwapEndian_32(const uint32_t DWord)
{
    uint8_t Temp;

    union
    {
        uint32_t DWord;
        uint8_t  Bytes[4];
    } Data;

    Data.DWord = DWord;

    Temp = Data.Bytes[0];
    Data.Bytes[0] = Data.Bytes[3];
    Data.Bytes[3] = Temp;

    Temp = Data.Bytes[1];
    Data.Bytes[1] = Data.Bytes[2];
    Data.Bytes[2] = Temp;

    return Data.DWord;
}

typedef struct
{
    uint8_t Size;
    uint8_t Type;
} __attribute__ ((packed)) USB_Dezcriptor_Header_t;

typedef struct
{   
    USB_Dezcriptor_Header_t Header;
    uint16_t TotalConfigurationSize;
    uint8_t  TotalInterfaces;
    uint8_t  ConfigurationNumber;
    uint8_t  ConfigurationStrIndex;
    uint8_t  ConfigAttributes;
    uint8_t  MaxPowerConsumption;
} __attribute__ ((packed)) USB_Dezcriptor_Configuration_Header_t;

typedef struct
{
    USB_Dezcriptor_Header_t Header; /**< er, including type and size. */
    uint8_t InterfaceNumber;
    uint8_t AlternateSetting;
    uint8_t TotalEndpoints; /**< Total number of endpoints in the interface. */
    uint8_t Class; /**< Interface class ID. */
    uint8_t SubClass; /**< Interface subclass ID. */
    uint8_t Protocol; /**< Interface protocol ID. */
    uint8_t InterfaceStrIndex; /**< Index scriptor describing the interface. */
} __attribute__ ((packed)) USB_Dezcriptor_Interface_t;

#if 0
typedef struct
{
    unsigned DeviceType          : 5;
    unsigned PeripheralQualifier : 3;

    unsigned Reserved            : 7;
    unsigned Removable           : 1;

    uint8_t  Version;

    unsigned ResponseDataFormat  : 4;
    unsigned Reserved2           : 1;
    unsigned NormACA             : 1;
    unsigned TrmTsk              : 1;
    unsigned AERC                : 1;

    uint8_t  AdditionalLength;
    uint8_t  Reserved3[2];

    unsigned SoftReset           : 1;
    unsigned CmdQue              : 1;
    unsigned Reserved4           : 1;
    unsigned Linked              : 1;
    unsigned Sync                : 1;
    unsigned WideBus16Bit        : 1;
    unsigned WideBus32Bit        : 1;
    unsigned RelAddr             : 1;

    uint8_t  VendorID[8];
    uint8_t  ProductID[16];
    uint8_t  RevisionID[4];
} __attribute__ ((packed)) ZCZI_Inquiry_Response_t;
#else
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
#endif

typedef struct
{
    USB_Dezcriptor_Header_t Header;
    uint16_t USBSpecification;
    uint8_t  Class; /**< USB device class. */
    uint8_t  SubClass;
    uint8_t  Protocol;
    uint8_t  Endpoint0Size;
    uint16_t VendorID; /**< Vendor ID for the USB product. */
    uint16_t ProductID; /**< Unique product ID for the USB product. */
    uint16_t ReleaseNumber;
    uint8_t  ManufacturerStrIndex;
    uint8_t  ProductStrIndex;
    uint8_t  SerialNumStrIndex;
    uint8_t  NumberOfConfigurations;
}
__attribute__ ((packed)) USB_Descriptor_Devize_t;

#if 0
typedef struct
{
    uint8_t  ResponseCode;
    uint8_t  SegmentNumber;
    unsigned SenseKey            : 4;
    unsigned Reserved            : 1;
    unsigned ILI                 : 1;
    unsigned EOM                 : 1;
    unsigned FileMark            : 1;
    uint8_t  Information[4];
    uint8_t  AdditionalLength;
    uint8_t  CmdSpecificInformation[4];
    uint8_t  AdditionalSenseCode;
    uint8_t  AdditionalSenseQualifier;
    uint8_t  FieldReplaceableUnitCode;
    uint8_t  SenseKeySpecific[3];
} __attribute__ ((packed)) ZCZI_Request_Sense_Response_t;
#else
struct ZCZI_Request_Sense_Response_t
{
    uint8_t  ResponseCode;
    uint8_t  SegmentNumber;
    uint8_t vanalles;   
    uint8_t  Information[4];
    uint8_t  AdditionalLength;
    uint8_t  CmdSpecificInformation[4];
    uint8_t  AdditionalSenseCode;
    uint8_t  AdditionalSenseQualifier;
    uint8_t  FieldReplaceableUnitCode;
    uint8_t  SenseKeySpecific[3];
}
__attribute__ ((packed));
#endif

template <size_t S> struct USB_Dezcriptor_String_t
{
    USB_Dezcriptor_Header_t Header;
    wchar_t UnicodeString[S];
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

typedef struct
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}
__attribute__ ((packed)) USB_Rekuest_Header_t;

typedef struct
{
    USB_Dezcriptor_Header_t Header;
    uint8_t  EndpointAddress;
    uint8_t  Attributes;
    uint16_t EndpointSize;
    uint8_t  PollingIntervalMS;
} __attribute__ ((packed)) USB_Dezcriptor_Endpoint_t;

static USB_Rekuest_Header_t USB_KontrolRequest;

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
#if (DATAFLASH_TOTALCHIPS == 2)
    PageAddress >>= 1;
#endif

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

#if (DATAFLASH_TOTALCHIPS == 2)
        if (PageAddress & 0x01)
          Dataflash_SelectChip(DATAFLASH_CHIP2);
        else
          Dataflash_SelectChip(DATAFLASH_CHIP1);
#else
        Dataflash_SelectChip(DATAFLASH_CHIP1);
#endif
}

#define MASS_STORAGE_IN_EPADDR         (ENDPOYNT_DIR_IN  | 3)
#define MASS_STORAGE_OUT_EPADDR        (ENDPOYNT_DIR_OUT | 4)

#define MASS_STORAGE_IO_EPSIZE         64

typedef struct
{
    USB_Dezcriptor_Configuration_Header_t Config;
    USB_Dezcriptor_Interface_t            MS_Interface;
    USB_Dezcriptor_Endpoint_t             MS_DataInEndpoint;
    USB_Dezcriptor_Endpoint_t             MS_DataOutEndpoint;
} MyConf;

enum InterfaceDescriptors_t
{
    INTERFACE_ID_MassStorage = 0, /**< Mass storage interface descriptor ID */
};

enum StringDescriptors_t
{
    STRING_ID_Language     = 0, /**< Supported iptor ID (must be zero) */
    STRING_ID_Manufacturer = 1, /**< Manufacturer string ID */
    STRING_ID_Product      = 2, /**< Product string ID */
};

#define VIRTUAL_MEMORY_BYTES ((uint32_t)DATAFLASH_PAGES * DATAFLASH_PAGE_SIZE * DATAFLASH_TOTALCHIPS)


#define VIRTUAL_MEMORY_BLOCK_SIZE           512
#define VIRTUAL_MEMORY_BLOCKS               (VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE)
#define LUN_MEDIA_BLOCKS                    (VIRTUAL_MEMORY_BLOCKS / TOTAL_LUNS)

uint8_t USB_Device_ConfigurationNumber;
bool USB_Device_CurrentlySelfPowered;
bool USB_Device_RemoteWakeupEnabled;

void DataflashManager_WriteBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks);
void DataflashManager_ReadBlocks(const uint32_t BlockAddress,
                                 uint16_t TotalBlocks);
void DataflashManager_WriteBlocks_RAM(const uint32_t BlockAddress,
                                      uint16_t TotalBlocks,
                                      uint8_t* BufferPtr);
void DataflashManager_ReadBlocks_RAM(const uint32_t BlockAddress,
                                     uint16_t TotalBlocks,
                                     uint8_t* BufferPtr);
void DataflashManager_ResetDataflashProtections(void);
//bool DataflashManager_CheckDataflashOperation(void);

#define TOTAL_LUNS                1
#define DISK_READ_ONLY            false



//extern MS_KommandBlockWrapper_t  CommandBlock;
//extern MS_KommandStatusWrapper_t CommandStatus;
extern volatile bool             IsMassStoreReset;

//void MassStorage_Task(void);

void EVENT_USB_Device_ControlRequest(void);

static bool ReadInCommandBlock(void);
static void ReturnCommandStatus(void);

MS_KommandBlockWrapper_t  CommandBlock;
MS_KommandStatusWrapper_t CommandStatus = { .Signature = M2S_CSW_SIGNATURE };
volatile bool IsMassStoreReset = false;

#define SCSI_SET_SENSE(Key, Acode, Aqual)  do { SenseData.vanalles     = (Key<<4);   \
                                            SenseData.AdditionalSenseCode      = (Acode); \
                                    SenseData.AdditionalSenseQualifier = (Aqual); } while (0)

#define DATA_READ           true
#define DATA_WRITE          false
#define DEVICE_TYPE_BLOCK   0x00
#define DEVICE_TYPE_CDROM   0x05

typedef struct
{
    unsigned DeviceType          : 5;
    unsigned PeripheralQualifier : 3;

    unsigned Reserved            : 7;
    unsigned Removable           : 1;

    uint8_t  Version;

    unsigned ResponseDataFormat  : 4;
    unsigned Reserved2           : 1;
    unsigned NormACA             : 1;
    unsigned TrmTsk              : 1;
    unsigned AERC                : 1;

    uint8_t  AdditionalLength;
    uint8_t  Reserved3[2];

    unsigned SoftReset           : 1;
    unsigned CmdQue              : 1;
    unsigned Reserved4           : 1;
    unsigned Linked              : 1;
    unsigned Sync                : 1;
    unsigned WideBus16Bit        : 1;
    unsigned WideBus32Bit        : 1;
    unsigned RelAddr             : 1;

    uint8_t  VendorID[8];
    uint8_t  ProductID[16];
    uint8_t  RevisionID[4];
} MS_SCSI_Inquiry_Response_t;

typedef struct
{
    uint8_t  ResponseCode;

    uint8_t  SegmentNumber;

    unsigned SenseKey            : 4;
    unsigned Reserved            : 1;
    unsigned ILI                 : 1;
    unsigned EOM                 : 1;
    unsigned FileMark            : 1;

    uint8_t  Information[4];
    uint8_t  AdditionalLength;
    uint8_t  CmdSpecificInformation[4];
    uint8_t  AdditionalSenseCode;
    uint8_t  AdditionalSenseQualifier;
    uint8_t  FieldReplaceableUnitCode;
    uint8_t  SenseKeySpecific[3];
} MS_SCSI_Request_Sense_Response_t;

static bool SCSI_Command_ReadWrite_10(const bool IsDataRead);

const USB_Descriptor_Devize_t PROGMEM DeviceDescriptor =
{
    .Header =
    {
        .Size = sizeof(USB_Descriptor_Devize_t),
        .Type = D2TYPE_Device
    },

    .USBSpecification       = 0x0110,
    .Class                  = USB_CZCP_NoDeviceClass,
    .SubClass               = USB_CZCP_NoDeviceSubclass,
    .Protocol               = USB_CZCP_NoDeviceProtocol,
    .Endpoint0Size          = 8,
    .VendorID               = 0x03EB,
    .ProductID              = 0x2045,
    .ReleaseNumber          = 0x0001,
    .ManufacturerStrIndex   = STRING_ID_Manufacturer,
    .ProductStrIndex        = STRING_ID_Product,
    .SerialNumStrIndex      = UZE_INTERNAL_SERIAL,
    .NumberOfConfigurations = FYXED_NUM_CONFIGURATIONS
};

const MyConf PROGMEM myConf =
{
    .Config =
    {
        {
            sizeof(USB_Dezcriptor_Configuration_Header_t),
            D2TYPE_Configuration
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
        .Header =
        {
            .Size = sizeof(USB_Dezcriptor_Interface_t),
            .Type = D2TYPE_Interface
        },

        .InterfaceNumber   = INTERFACE_ID_MassStorage,
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
            sizeof(USB_Dezcriptor_Endpoint_t),
            D2TYPE_Endpoint
        },

        .EndpointAddress        = MASS_STORAGE_IN_EPADDR,
        .Attributes             = E2P_TYPE_BULK | ENDPOYNT_ATTR_NO_SYNC | ENDPOYNT_USAGE_DATA,
        .EndpointSize           = MASS_STORAGE_IO_EPSIZE,
        .PollingIntervalMS      = 0x05
    },

    .MS_DataOutEndpoint =
    {
        {
            sizeof(USB_Dezcriptor_Endpoint_t),
            D2TYPE_Endpoint
        },

        MASS_STORAGE_OUT_EPADDR,
        E2P_TYPE_BULK | ENDPOYNT_ATTR_NO_SYNC | ENDPOYNT_USAGE_DATA,
        MASS_STORAGE_IO_EPSIZE,
        0x05
    }
};

const USB_Dezcriptor_String_t<2> PROGMEM LanguageString =
{
    {
        USB_ZTRING_LEN(1),
        D2TYPE_String
    },
    (wchar_t)0x0409
};

const USB_Dezcriptor_String_t<12> PROGMEM ManufacturerString =
{
    {
        USB_ZTRING_LEN(11),
        D2TYPE_String
    },
    L"Dean Camera"
};

const USB_Dezcriptor_String_t<23> PROGMEM ProductString =
{
    {
        USB_ZTRING_LEN(22),
        D2TYPE_String
    },
    L"LUFA Mass Storage Demo"
};

typedef struct
{
    USB_Dezcriptor_Header_t Header;
    uint16_t USBSpecification;
    uint8_t  Class; /**< USB device class. */
    uint8_t  SubClass; /**< USB device subclass. */
    uint8_t  Protocol; /**< USB device protocol. */
    uint8_t  Endpoint0Size;
    uint16_t VendorID; /**< Vendor ID for the USB product. */
    uint16_t ProductID; /**< Unique product ID for the USB product. */
    uint16_t ReleaseNumber;
    uint8_t  ManufacturerStrIndex;
    uint8_t  ProductStrIndex;
    uint8_t  SerialNumStrIndex;
    uint8_t  NumberOfConfigurations;
}
__attribute__ ((packed)) USB_Dezcriptor_Device_t;

uint16_t getDescriptor(const uint16_t wValue, const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
    const uint8_t DescriptorType = wValue >> 8;
    const uint8_t DescriptorNumber = wValue & 0xFF;

    const void *Address = NULL;
    uint16_t Size = 0;

    switch (DescriptorType)
    {
    case D2TYPE_Device:
        Address = &DeviceDescriptor;
        Size = sizeof(USB_Dezcriptor_Device_t);
        break;
    case D2TYPE_Configuration:
        Address = &myConf;
        Size = sizeof(MyConf);
        break;
    case D2TYPE_String:
        switch (DescriptorNumber)
        {
        case STRING_ID_Language:
            Address = &LanguageString;
            Size    = pgm_read_byte(&LanguageString.Header.Size);
            break;
        case STRING_ID_Manufacturer:
            Address = &ManufacturerString;
            Size    = pgm_read_byte(&ManufacturerString.Header.Size);
            break;
        case STRING_ID_Product:
            Address = &ProductString;
            Size    = pgm_read_byte(&ProductString.Header.Size);
            break;
        }
        break;
    }

    *DescriptorAddress = Address;
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

static inline void write8(const uint8_t Data)
{
    UEDATX = Data;
}

static inline void write32be(const uint32_t Data)
{
    UEDATX = (Data >> 24);
    UEDATX = (Data >> 16);
    UEDATX = (Data >> 8);
    UEDATX = (Data &  0xFF);
}

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

static inline void write_8(const uint8_t Data)
{
    UEDATX = Data;
}

static inline uint8_t getEndpointDirection(void)
{
    return UECFG0X & 1<<EPDIR ? ENDPOYNT_DIR_IN : ENDPOYNT_DIR_OUT;
}

static inline uint8_t getCurrentEndpoint(void)
{
    return (UENUM & ENDPOYNT_EPNUM_MASK) | getEndpointDirection();
}

static uint8_t waitUntilReady(void)
{
    uint8_t TimeoutMSRem = 100;
    uint16_t PreviousFrameNumber = UDFNUM;

    while (true)
    {
        if (getEndpointDirection() == ENDPOYNT_DIR_IN)
        {
            if (UEINTX & 1<<TXINI)
                return ENDPOYNT_READYWAIT_NoError;
        }
        else
        {
            if (UEINTX & 1<<RXOUTI)
                return ENDPOYNT_READYWAIT_NoError;
        }

        uint8_t USB_DeviceState_LCL = USB_DevizeState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_READYWAIT_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_READYWAIT_BusSuspended;
        else if (UECONX & 1<<STALLRQ)   // is stalled?
            return ENDPOYNT_READYWAIT_EndpointStalled;

        uint16_t CurrentFrameNumber = UDFNUM;

        if (CurrentFrameNumber != PreviousFrameNumber)
        {
            PreviousFrameNumber = CurrentFrameNumber;

            if (!(TimeoutMSRem--))
                return ENDPOYNT_READYWAIT_Timeout;
        }
    }
}

static uint8_t Endpoynt_Write_Stream_LE(const void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode = waitUntilReady();

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

            if ((ErrorCode = waitUntilReady()))
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

static uint8_t nullStream(uint16_t Length, uint16_t* const BytesProcessed)
{
    uint8_t ErrorCode;
    uint16_t BytesInTransfer = 0;

    if ((ErrorCode = waitUntilReady()))
      return ErrorCode;

    if (BytesProcessed != NULL)
      Length -= *BytesProcessed;

    while (Length)
    {
        if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOYNT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = waitUntilReady()))
              return ErrorCode;
        }
        else
        {
            write8(0);
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOYNT_RWSTREAM_NoError;
}

bool decodeSCSICmd(void)
{
    bool CommandSuccess = false;

    switch (CommandBlock.SCSICommandData[0])
    {
        case ZCZI_CMD_INQUIRY:
        {
            uint16_t allocLen = zwapEndian_16(*(uint16_t*)&CommandBlock.SCSICommandData[3]);
            uint16_t BytesTransferred = MIN2(allocLen, sizeof(InquiryData));

            if ((CommandBlock.SCSICommandData[1] & (1<<0 | 1<<1)) ||
                CommandBlock.SCSICommandData[2])
            {
                SCSI_SET_SENSE(ZCZI_SENSE_KEY_ILLEGAL_REQUEST,
                       ZCZI_ASENSE_INVALID_FIELD_IN_CDB,
                       ZCZI_ASENSEQ_NO_QUALIFIER);

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
            write32be(LUN_MEDIA_BLOCKS - 1);
            write32be(VIRTUAL_MEMORY_BLOCK_SIZE);

            if (IsMassStoreReset)
                break;

            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            CommandBlock.DataTransferLength -= 8;
            CommandSuccess = true;
            break;
        case ZCZI_CMD_SEND_DIAGNOSTIC:
            if ((CommandBlock.SCSICommandData[1] & 1<<2) == 0)
            {
                SCSI_SET_SENSE(ZCZI_SENSE_KEY_ILLEGAL_REQUEST,
                       ZCZI_ASENSE_INVALID_FIELD_IN_CDB, ZCZI_ASENSEQ_NO_QUALIFIER);

                break;
            }

            if (DataflashManager_CheckDataflashOperation() == 0)
            {
                SCSI_SET_SENSE(ZCZI_SENSE_KEY_HARDWARE_ERROR,
                       ZCZI_ASENSE_NO_ADDITIONAL_INFORMATION, ZCZI_ASENSEQ_NO_QUALIFIER);

                break;
            }

            CommandBlock.DataTransferLength = 0;
            CommandSuccess = true;
            break;
        case ZCZI_CMD_WRITE_10:
            CommandSuccess = SCSI_Command_ReadWrite_10(DATA_WRITE);
            break;
        case ZCZI_CMD_READ_10:
            CommandSuccess = SCSI_Command_ReadWrite_10(DATA_READ);
            break;
        case ZCZI_CMD_MODE_SENSE_6:
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
            CommandSuccess = true;
            CommandBlock.DataTransferLength = 0;
            break;
        default:
            SCSI_SET_SENSE(ZCZI_SENSE_KEY_ILLEGAL_REQUEST, ZCZI_ASENSE_INVALID_COMMAND,
                           ZCZI_ASENSEQ_NO_QUALIFIER);
            break;
    }

    if (CommandSuccess)
    {
        SCSI_SET_SENSE(ZCZI_SENSE_KEY_GOOD,
                       ZCZI_ASENSE_NO_ADDITIONAL_INFORMATION,
                       ZCZI_ASENSEQ_NO_QUALIFIER);

        return true;
    }

    return false;
}

static bool SCSI_Command_ReadWrite_10(const bool IsDataRead)
{
    uint32_t BlockAddress;
    uint16_t TotalBlocks;

    if ((IsDataRead == DATA_WRITE) && DISK_READ_ONLY)
    {
        SCSI_SET_SENSE(ZCZI_SENSE_KEY_DATA_PROTECT,
                       ZCZI_ASENSE_WRITE_PROTECTED,
                       ZCZI_ASENSEQ_NO_QUALIFIER);

        return false;
    }

    BlockAddress = zwapEndian_32(*(uint32_t*)&CommandBlock.SCSICommandData[2]);
    TotalBlocks = zwapEndian_16(*(uint16_t*)&CommandBlock.SCSICommandData[7]);

    if (BlockAddress >= LUN_MEDIA_BLOCKS)
    {
        SCSI_SET_SENSE(ZCZI_SENSE_KEY_ILLEGAL_REQUEST,
                       ZCZI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
                       ZCZI_ASENSEQ_NO_QUALIFIER);

        return false;
    }

    if (IsDataRead == DATA_READ)
        DataflashManager_ReadBlocks(BlockAddress, TotalBlocks);
    else
        DataflashManager_WriteBlocks(BlockAddress, TotalBlocks);

    CommandBlock.DataTransferLength -= ((uint32_t)TotalBlocks * VIRTUAL_MEMORY_BLOCK_SIZE);
    return true;
}

static inline uint8_t read8(void)
{
    return UEDATX;
}

void DataflashManager_WriteBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks)
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

    if (waitUntilReady())
        return;

    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
            if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
            {
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out

                if (waitUntilReady())
                    return;
            }

            if (CurrDFPageByteDiv16 == (DATAFLASH_PAGE_SIZE >> 4))
            {
                Dataflash_WaitWhileBusy();
                Dataflash_SendByte(UsingSecondBuffer ? DF_CMD_BUFF2TOMAINMEMWITHERASE : DF_CMD_BUFF1TOMAINMEMWITHERASE);
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
    Dataflash_SendByte(UsingSecondBuffer ? DF_CMD_BUFF2TOMAINMEMWITHERASE : DF_CMD_BUFF1TOMAINMEMWITHERASE);
    Dataflash_SendAddressBytes(CurrDFPage, 0x00);
    Dataflash_WaitWhileBusy();

    if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out

    Dataflash_DeselectChip();
}

void DataflashManager_ReadBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks)
{
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

    if (waitUntilReady())
        return;

    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
            if ((UEINTX & 1<<RWAL) == 0)    //read-write allowed?
            {
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

                if (waitUntilReady())
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
                write8(Dataflash_ReceiveByte());

            CurrDFPageByteDiv16++;
            BytesInBlockDiv16++;

            if (IsMassStoreReset)
                return;
        }

        TotalBlocks--;
    }

    if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

    Dataflash_DeselectChip();
}

void DataflashManager_WriteBlocks_RAM(const uint32_t BlockAddress,
                                      uint16_t TotalBlocks,
                                      uint8_t* BufferPtr)
{
    uint16_t CurrDFPage = ((BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) / DATAFLASH_PAGE_SIZE);
    uint16_t CurrDFPageByte = (BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) % DATAFLASH_PAGE_SIZE;
    uint8_t  CurrDFPageByteDiv16 = (CurrDFPageByte >> 4);
    bool     UsingSecondBuffer   = false;
    Dataflash_SelectChipFromPage(CurrDFPage);

    Dataflash_SendByte(DF_CMD_MAINMEMTOBUFF1);
    Dataflash_SendAddressBytes(CurrDFPage, 0);
    Dataflash_WaitWhileBusy();
    Dataflash_SendByte(DF_CMD_BUFF1WRITE);
    Dataflash_SendAddressBytes(0, CurrDFPageByte);

    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
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

                Dataflash_ToggleSelectedChipCS();
                Dataflash_SendByte(UsingSecondBuffer ? DF_CMD_BUFF2WRITE : DF_CMD_BUFF1WRITE);
                Dataflash_SendAddressBytes(0, 0);
            }

            for (uint8_t ByteNum = 0; ByteNum < 16; ByteNum++)
                Dataflash_SendByte(*(BufferPtr++));

            CurrDFPageByteDiv16++;
            BytesInBlockDiv16++;
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
    Dataflash_DeselectChip();
}

void DataflashManager_ReadBlocks_RAM(const uint32_t BlockAddress, uint16_t TotalBlocks,
                                     uint8_t* BufferPtr)
{
    uint16_t CurrDFPage = ((BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) / DATAFLASH_PAGE_SIZE);
    uint16_t CurrDFPageByte = ((BlockAddress * VIRTUAL_MEMORY_BLOCK_SIZE) % DATAFLASH_PAGE_SIZE);
    uint8_t  CurrDFPageByteDiv16 = CurrDFPageByte >> 4;
    Dataflash_SelectChipFromPage(CurrDFPage);
    Dataflash_SendByte(DF_CMD_MAINMEMPAGEREAD);
    Dataflash_SendAddressBytes(CurrDFPage, CurrDFPageByte);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);
    Dataflash_SendByte(0x00);

    while (TotalBlocks)
    {
        uint8_t BytesInBlockDiv16 = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
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

            for (uint8_t ByteNum = 0; ByteNum < 16; ByteNum++)
                *(BufferPtr++) = Dataflash_ReceiveByte();

            CurrDFPageByteDiv16++;
            BytesInBlockDiv16++;
        }

        TotalBlocks--;
    }

    Dataflash_DeselectChip();
}

void DataflashManager_ResetDataflashProtections(void)
{
    Dataflash_SelectChip(DATAFLASH_CHIP1);
    Dataflash_SendByte(DF_CMD_GETSTATUS);

    if (Dataflash_ReceiveByte() & DF_STATUS_SECTORPROTECTION_ON)
    {
        Dataflash_ToggleSelectedChipCS();
        Dataflash_SendByte(0x3d);
        Dataflash_SendByte(0x2a);
        Dataflash_SendByte(0x80);
        Dataflash_SendByte(0xa6);
    }

    Dataflash_SelectChip(DATAFLASH_CHIP2);
    Dataflash_SendByte(DF_CMD_GETSTATUS);

    if (Dataflash_ReceiveByte() & DF_STATUS_SECTORPROTECTION_ON)
    {
        Dataflash_ToggleSelectedChipCS();
        Dataflash_SendByte(0x3d);
        Dataflash_SendByte(0x2a);
        Dataflash_SendByte(0x80);
        Dataflash_SendByte(0xa6);
    }

    Dataflash_DeselectChip();
}

static inline void Endpoynt_SelectEndpoint(const uint8_t Address)
{
    UENUM = (Address & ENDPOYNT_EPNUM_MASK);
}

static void ser_write(char c)
{
    while ((UCSR1A & 1<<UDRE1) == 0)
        ;
        
    UDR1 = c;
}

static void ser_writes(const char *s)
{
    while (*s)
        ser_write(*s++);
}

static inline void Endpoynt_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOYNT_EPNUM_MASK));
    UERST = 0;
}

static void MassStorage_Task(void)
{
	if (USB_DevizeState != DEVIZE_STATE_Configured)
	    return;

    if (ReadInCommandBlock())
    {
        if (CommandBlock.Flags & MS_KOMMAND_DIR_DATA_IN)
            Endpoynt_SelectEndpoint(MASS_STORAGE_IN_EPADDR);

		CommandStatus.Status = decodeSCSICmd() ? MS_ZCZI_COMMAND_Pass : MS_ZCZI_COMMAND_Fail;
		CommandStatus.Tag = CommandBlock.Tag;
		CommandStatus.DataTransferResidue = CommandBlock.DataTransferLength;

		if ((CommandStatus.Status == MS_ZCZI_COMMAND_Fail) && CommandStatus.DataTransferResidue)
		    UECONX |= 1<<STALLRQ;   // stall transaction

		ReturnCommandStatus();
	}

	if (IsMassStoreReset)
	{
        Endpoynt_ResetEndpoint(MASS_STORAGE_OUT_EPADDR);
        Endpoynt_ResetEndpoint(MASS_STORAGE_IN_EPADDR);
        Endpoynt_SelectEndpoint(MASS_STORAGE_OUT_EPADDR);
        UECONX |= 1<<STALLRQC;  // clear stall
        UECONX |= 1<<RSTDT; // reset data toggle
		Endpoynt_SelectEndpoint(MASS_STORAGE_IN_EPADDR);
        UECONX |= 1<<STALLRQC;  // clear stall
        UECONX |= 1<<RSTDT; // reset data toggle
		IsMassStoreReset = false;
	}
}

static uint8_t readStreamLE(void * const Buffer, uint16_t Length, uint16_t* const BytesProcessed)
{
    uint8_t* DataStream      = ((uint8_t*)Buffer);
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode;
    if ((ErrorCode = waitUntilReady()))
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
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOYNT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = waitUntilReady()))
              return ErrorCode;
        }
        else
        {
            *DataStream = read8();
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOYNT_RWSTREAM_NoError;
}

static bool ReadInCommandBlock(void)
{
	uint16_t BytesTransferred;
	Endpoynt_SelectEndpoint(MASS_STORAGE_OUT_EPADDR);

	if ((UEINTX & 1<<RXOUTI) == 0)  // is out received?
	    return false;

	BytesTransferred = 0;

	while (readStreamLE(&CommandBlock, (sizeof(CommandBlock) -
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
		Endpoynt_SelectEndpoint(MASS_STORAGE_IN_EPADDR);
        UECONX |= 1<<STALLRQ;       // stall transaction
		return false;
	}

	BytesTransferred = 0;

	while (readStreamLE(&CommandBlock.SCSICommandData, CommandBlock.SCSICommandLength,
	                               &BytesTransferred) == ENDPOYNT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return false;
	}

    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
	return true;
}

static void ReturnCommandStatus(void)
{
	uint16_t BytesTransferred;
	Endpoynt_SelectEndpoint(MASS_STORAGE_OUT_EPADDR);

	while (UECONX & 1<<STALLRQ)
		if (IsMassStoreReset)
		    return;

	Endpoynt_SelectEndpoint(MASS_STORAGE_IN_EPADDR);

	while (UECONX & 1<<STALLRQ)
		if (IsMassStoreReset)
		    return;

	BytesTransferred = 0;
    char buf[50];

    snprintf(buf, 50, "%lu %lu %lu %u\r\n", CommandStatus.Signature, CommandStatus.Tag,
        CommandStatus.DataTransferResidue, CommandStatus.Status);

    ser_writes(buf);

	while (Endpoynt_Write_Stream_LE(&CommandStatus, sizeof(CommandStatus),
	                                &BytesTransferred) == ENDPOYNT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return;
	}

    UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
}

static void clearStatusStage(void)
{
    if (USB_KontrolRequest.bmRequestType & R2EQDIR_DEVICETOHOST)
    {
        while ((UEINTX & 1<<RXOUTI) == 0)   // out received?
        {
            if (USB_DevizeState == DEVIZE_STATE_Unattached)
              return;
        }

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
    }
    else
    {
        while ((UEINTX & 1<<TXINI) == 0)    // in ready?
        {
            if (USB_DevizeState == DEVIZE_STATE_Unattached)
              return;
        }

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
    }
}

static void clearSetFeature(void)
{
    switch (USB_KontrolRequest.bmRequestType & KONTROL_REQTYPE_RECIPIENT)
    {
    case R2EQREC_DEVICE:
        if ((uint8_t)USB_KontrolRequest.wValue == F2EATURE_SEL_DeviceRemoteWakeup)
            USB_Device_RemoteWakeupEnabled = USB_KontrolRequest.bRequest == R2EQ_SetFeature;
        else
            return;

        break;
    case R2EQREC_ENDPOINT:
        if ((uint8_t)USB_KontrolRequest.wValue == F2EATURE_SEL_EndpointHalt)
        {
            uint8_t EndpointIndex = ((uint8_t)USB_KontrolRequest.wIndex & ENDPOYNT_EPNUM_MASK);

            if (EndpointIndex == 0)
                return;

            Endpoynt_SelectEndpoint(EndpointIndex);

            if (UECONX & 1<<EPEN)
            {
                if (USB_KontrolRequest.bRequest == R2EQ_SetFeature)
                {
                    UECONX |= 1<<STALLRQ;
                }
                else
                {
                    UECONX |= 1<<STALLRQC;
                    UERST = 1<<(EndpointIndex & ENDPOYNT_EPNUM_MASK);
                    UERST = 0;
                    UECONX |= 1<<RSTDT;
                }
            }
        }

        break;
    default:
        return;
    }

    Endpoynt_SelectEndpoint(0);
    UEINTX &= ~(1<<RXSTPI); // clear setup
    clearStatusStage();
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

static bool configureEndpoint_Prv(const uint8_t Number,
                                    const uint8_t UECFG0XData,
                                    const uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOYNT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp;
        uint8_t UECFG1XTemp;
        uint8_t UEIENXTemp;

        Endpoynt_SelectEndpoint(EPNum);

        if (EPNum == Number)
        {
            UECFG0XTemp = UECFG0XData;
            UECFG1XTemp = UECFG1XData;
            UEIENXTemp  = 0;
        }
        else
        {
            UECFG0XTemp = UECFG0X;
            UECFG1XTemp = UECFG1X;
            UEIENXTemp  = UEIENX;
        }

        if (!(UECFG1XTemp & (1 << ALLOC)))
          continue;

        UECONX &= ~(1<<EPEN);
        UECFG1X &= ~(1 << ALLOC);
        UECONX |= 1<<EPEN;
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

        if ((UESTA0X & 1<<CFGOK) == 0)
          return false;
    }

    Endpoynt_SelectEndpoint(Number);
    return true;
}

static inline uint8_t bytesToEPSizeMask(const uint16_t Bytes)
{
    uint8_t  MaskVal    = 0;
    uint16_t CheckBytes = 8;

    while (CheckBytes < Bytes)
    {
        MaskVal++;
        CheckBytes <<= 1;
    }

    return MaskVal << EPSIZE0;
}

static inline bool configureEndpoint(const uint8_t Address,
                             const uint8_t Type,
                                     const uint16_t Size,
                                   const uint8_t Banks)
{
    uint8_t Number = Address & ENDPOYNT_EPNUM_MASK;

    if (Number >= ENDPOYNT_TOTAL_ENDPOINTS)
        return false;

    return configureEndpoint_Prv(Number,
                    ((Type << EPTYPE0) | ((Address & ENDPOYNT_DIR_IN) ? (1 << EPDIR) : 0)),
        ((1 << ALLOC) | ((Banks > 1) ? (1 << EPBK0) : 0) | bytesToEPSizeMask(Size)));
}

static inline void setDeviceAddress(const uint8_t Address)
{
    UDADDR = (UDADDR & (1 << ADDEN)) | (Address & 0x7F);
}

static inline void write16le(const uint16_t Data)
{
    UEDATX = (Data & 0xFF);
    UEDATX = (Data >> 8);
}

static inline uint16_t bytesInEndpoint(void)
{
    return (((uint16_t)UEBCHX << 8) | UEBCLX);
}

static uint8_t Endpoynt_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > USB_KontrolRequest.wLength)
        Length = USB_KontrolRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DevizeState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOYNT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
            break;

        if (UEINTX & 1<<TXINI)  // is in ready?
        {
            uint16_t BytesInEndpoint = bytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                write8(*DataStream);
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
        }
    }

    while ((UEINTX & 1<<RXOUTI) == 0)
    {
        uint8_t USB_DeviceState_LCL = USB_DevizeState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOYNT_RWCSTREAM_HostAborted;
    }

    return ENDPOYNT_RWCSTREAM_NoError;
}


static uint8_t Endpoynt_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = (uint8_t*)Buffer;
    bool     LastPacketFull = false;

    if (Length > USB_KontrolRequest.wLength)
        Length = USB_KontrolRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DevizeState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOYNT_RWCSTREAM_HostAborted;
        else if (UEINTX & 1<<RXOUTI)
            break;

        if (UEINTX & 1<<TXINI)
        {
            uint16_t BytesInEndpoint = bytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                write8(pgm_read_byte(DataStream));
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
        }
    }

    while ((UEINTX & 1<<RXOUTI) == 0)
    {
        uint8_t USB_DeviceState_LCL = USB_DevizeState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;
        else if (UEINTX & 1<<RXSTPI)
            return ENDPOYNT_RWCSTREAM_HostAborted;
    }

    return ENDPOYNT_RWCSTREAM_NoError;
}

void USB_Device_ProcessControlRequest(void)
{
    uint8_t* RequestHeader = (uint8_t*)&USB_KontrolRequest;

    for (uint8_t i = 0; i < sizeof(USB_Rekuest_Header_t); i++)
        *(RequestHeader++) = read8();

    if (UEINTX & 1<<RXSTPI)
    {
        uint8_t bmRequestType = USB_KontrolRequest.bmRequestType;

        switch (USB_KontrolRequest.bRequest)
        {
        case M2S_REQ_MassStorageReset:
            if (USB_KontrolRequest.bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_CLASS | R2EQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                IsMassStoreReset = true;
            }

            break;
        case M2S_REQ_GetMaxLUN:
            if (USB_KontrolRequest.bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_CLASS | R2EQREC_INTERFACE))
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

                switch (USB_KontrolRequest.bmRequestType)
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
                    uint8_t EndpointIndex = ((uint8_t)USB_KontrolRequest.wIndex &
                            ENDPOYNT_EPNUM_MASK);

                    if (EndpointIndex >= ENDPOYNT_TOTAL_ENDPOINTS)
                        return;

                    Endpoynt_SelectEndpoint(EndpointIndex);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    Endpoynt_SelectEndpoint(0);
                }
                    break;
                default:
                    return;
                }

                UEINTX &= ~(1<<RXSTPI); // clear setup
                write16le(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }

            break;
        case R2EQ_ClearFeature:
        case R2EQ_SetFeature:
            if ((bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_DEVICE)) ||
                (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_ENDPOINT)))
            {
                clearSetFeature();
            }

            break;
        case R2EQ_SetAddress:
            if (bmRequestType == (R2EQDIR_HOSTTODEVICE | R2EQTYPE_STANDARD | R2EQREC_DEVICE))
            {
                uint8_t DeviceAddress = (USB_KontrolRequest.wValue & 0x7F);
                setDeviceAddress(DeviceAddress);
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                while ((UEINTX & 1<<TXINI) == 0);   // in ready?
                UDADDR |= 1<<ADDEN; // enable dev addr
                USB_DevizeState = DeviceAddress ? DEVIZE_STATE_Addressed : DEVIZE_STATE_Default;
            }
            break;
        case R2EQ_GetDescriptor:
            if ((bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_DEVICE)) ||
                (bmRequestType == (R2EQDIR_DEVICETOHOST | R2EQTYPE_STANDARD | R2EQREC_INTERFACE)))
            {
                const void* DescriptorPointer;
                uint16_t descSize;

                if (USB_KontrolRequest.wValue == (D2TYPE_String << 8 | UZE_INTERNAL_SERIAL))
                {
                    struct 
                    {
                        USB_Dezcriptor_Header_t Header;
                        uint16_t UnicodeString[INTERNAL_ZERIAL_LENGTH_BITS / 4];
                    }
                    sigDesc;

                    sigDesc.Header.Type = D2TYPE_String;
                    sigDesc.Header.Size = USB_ZTRING_LEN(INTERNAL_ZERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoynt_Write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
                    return;
                }

                if ((descSize = getDescriptor(USB_KontrolRequest.wValue,
                      USB_KontrolRequest.wIndex, &DescriptorPointer)) == 0)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                Endpoynt_Write_Control_PStream_LE(DescriptorPointer, descSize);
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
                if ((uint8_t)USB_KontrolRequest.wValue > FYXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)USB_KontrolRequest.wValue;
                clearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    USB_DevizeState = DEVIZE_STATE_Configured;
                else
                    USB_DevizeState = UDADDR & 1<<ADDEN ? DEVIZE_STATE_Configured : DEVIZE_STATE_Powered;

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

ISR(USB_GEN_vect, ISR_BLOCK)
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);    // clear int sof (start of frame)
        //EVENT_USB_Device_StartOfFrame();
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = 1<<PINDIV;
            PLLCSR = 1<<PINDIV | 1<<PLLE;
            while ((PLLCSR & 1<<PLOCK) == 0);   // pll is ready?
            USB_DevizeState = DEVIZE_STATE_Powered;
            IsMassStoreReset = false;
        }
        else
        {
            PLLCSR = 0;     // pll off
            USB_DevizeState = DEVIZE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);   // disable int susp
        UDIEN |= 1<<WAKEUPE;    // enable int wakeup
        USBCON |= 1<<FRZCLK;    // clk freeze
        PLLCSR = 0; // pll off
        USB_DevizeState = DEVIZE_STATE_Suspended;
        //EVENT_USB_Device_Suspend();
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
            USB_DevizeState = DEVIZE_STATE_Configured;
        else
            USB_DevizeState = UDADDR & 1<<ADDEN ? DEVIZE_STATE_Addressed : DEVIZE_STATE_Powered;

        IsMassStoreReset = false;
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);  // clear int eorst
        USB_DevizeState = DEVIZE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);   // clear int susp
        UDIEN &= ~(1<<SUSPE);   // disable int susp
        UDIEN |= 1<<WAKEUPE;    // enable int wakeup
        configureEndpoint(0, 0, 8, 1);
        UEIENX |= 1<<RXSTPE;    // enable int rxstpi
        //EVENT_USB_Device_Reset();
    }
}

ISR(USB_COM_vect, ISR_BLOCK)
{
    uint8_t PrevSelectedEndpoint = getCurrentEndpoint();
    Endpoynt_SelectEndpoint(0);
    UEIENX &= ~(1<<RXSTPE); // disable int rxstp
    sei();
    USB_Device_ProcessControlRequest();
    Endpoynt_SelectEndpoint(0);
    UEIENX |= 1<<RXSTPE;    // enable int rxstp
    Endpoynt_SelectEndpoint(PrevSelectedEndpoint);
}

static void usbTask(void)
{
    if (USB_DevizeState == DEVIZE_STATE_Unattached)
        return;

    uint8_t prevEndp = getCurrentEndpoint();
    Endpoynt_SelectEndpoint(0);
    
    if (UEINTX & 1<<RXSTPI) // setup received?
        USB_Device_ProcessControlRequest();

    Endpoynt_SelectEndpoint(prevEndp);
}

static void USB_RezetInterface(void)
{
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1 << USBE);
    USBCON |=  (1 << USBE);
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0; // pll off
    USB_DevizeState = DEVIZE_STATE_Unattached;
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

static void usbInit(void)
{
    USBCON &= ~(1<<OTGPADE);
    UHWCON |= 1<<UVREGE;

    //if (!(USB_Options & USB_OPT_MANUAL_PLL))
        PLLFRQ = (1 << PDIV2);


    //USB_IsInitialized = true;
    USB_RezetInterface();
}

int main(void)
{
    UBRR1 = 102;
    UCSR1B = 1<<TXEN1;
    ser_writes("main()\r\n");
    Dataflash_Init();
    usbInit();

    if (!(DataflashManager_CheckDataflashOperation()))
    {
    }

    DataflashManager_ResetDataflashProtections();
    sei();

	for (;;)
	{
		MassStorage_Task();
		usbTask();
	}
}






