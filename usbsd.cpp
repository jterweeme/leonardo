#include "usbsd.h"
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "misc.h"
#include <stdio.h>

//static uint32_t swapEndian(

template <class T> const T& min(const T& a, const T& b) { return !(b < a) ? a : b; }

static uint16_t swap16(uint16_t v) { return v >> 8 | v << 8; }

static const bool
    DISK_READ_ONLY = false,
    DATA_READ = true,
    DATA_WRITE = false;

static const uint8_t
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product = 2,
    MS_CSCP_SCSITransparentSubclass = 6,
    MS_CSCP_MassStorageClass = 8,
    MS_CSCP_BulkOnlyTransportProtocol = 0x50,
    MASS_STORAGE_IN_EPADDR = ENDPOINT_DIR_IN | 3,
    MASS_STORAGE_OUT_EPADDR = ENDPOINT_DIR_OUT | 4,
    MASS_STORAGE_IO_EPSIZE = 64,
    MS_REQ_MassStorageReset = 0xff,
    MS_REQ_GetMaxLUN = 0xfe,
    MS_SCSI_COMMAND_Pass = 0,
    MS_SCSI_COMMAND_Fail = 1,
    MS_COMMAND_DIR_DATA_OUT = 0<<7,
    MS_COMMAND_DIR_DATA_IN = 1<<7,
    DEVICE_TYPE_BLOCK = 0,
    SCSI_CMD_TEST_UNIT_READY = 0x00,
    SCSI_CMD_REQUEST_SENSE = 0x03,
    SCSI_CMD_READ_6 = 0x08,
    SCSI_CMD_INQUIRY = 0x12,
    SCSI_CMD_MODE_SENSE_6 = 0x1a,
    SCSI_CMD_START_STOP_UNIT = 0x1b,
    SCSI_CMD_SEND_DIAGNOSTIC = 0x1d,
    SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1e,
    SCSI_CMD_READ_CAPACITY_10 = 0x25,
    SCSI_CMD_READ_10 = 0x28,
    SCSI_CMD_WRITE_10 = 0x2a,
    SCSI_CMD_VERIFY_10 = 0x2f,
    SCSI_SENSE_KEY_ILLEGAL_REQUEST = 5,
    SCSI_ASENSEQ_NO_QUALIFIER = 0,
    SCSI_SENSE_KEY_GOOD = 0,
    SCSI_ASENSE_NO_ADDITIONAL_INFORMATION = 0,
    SCSI_SENSE_KEY_DATA_PROTECT = 0x07,
    SCSI_ASENSE_INVALID_COMMAND = 0x20,
    SCSI_ASENSE_WRITE_PROTECTED = 0x27,
    SCSI_ASENSE_INVALID_FIELD_IN_CDB = 0x24;

static const uint16_t
    VIRTUAL_MEMORY_BYTES = 32768,
    VIRTUAL_MEMORY_BLOCK_SIZE = 1024,
    TOTAL_LUNS = 2,
    VIRTUAL_MEMORY_BLOCKS = VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE,
    LUN_MEDIA_BLOCKS = VIRTUAL_MEMORY_BLOCKS / TOTAL_LUNS;


static const uint32_t
    MS_CSW_SIGNATURE = 0x53425355,
    MS_CBW_SIGNATURE = 0x43425355;

struct CmdStatusWrapper
{
    uint32_t sig;
    uint32_t tag;
    uint32_t dataTransferResidue;
    uint8_t status;
}
__attribute__ ((packed));

struct CmdBlockWrapper
{
    uint32_t sig;
    uint32_t tag;
    uint32_t dataTransferLen;
    uint8_t flags;
    uint8_t lun;
    uint8_t scsiCmdLen;
    uint8_t scsiCmdData[16];
}
__attribute__ ((packed));

const DescDev PROGMEM DeviceDescriptor =
{
    {
        sizeof(DescDev),
        DTYPE_Device
    },
    0x0110,
    USB_CSCP_NoDeviceClass,
    USB_CSCP_NoDeviceSubclass,
    USB_CSCP_NoDeviceProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03eb,
    0x2045,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    USE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};

struct MyConfiguration
{
    DescConf config;
    DescIface msInterface;
    DescEndpoint inpoint;
    DescEndpoint outpoint;
}
__attribute__ ((packed));

const MyConfiguration PROGMEM myConfiguration =
{
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        sizeof(MyConfiguration),
        1,
        1,
        NO_DESCRIPTOR,
        USB_CONFIG_ATTR_RESERVED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        {
            sizeof(DescIface),
            DTYPE_Interface
        },

        0, // INTERFACE_ID_MassStorage
        0,
        2,
        MS_CSCP_MassStorageClass,
        MS_CSCP_SCSITransparentSubclass,
        MS_CSCP_BulkOnlyTransportProtocol,
        NO_DESCRIPTOR
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },
        
        MASS_STORAGE_IN_EPADDR,
        EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        MASS_STORAGE_IO_EPSIZE,
        5
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        MASS_STORAGE_OUT_EPADDR,
        EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        MASS_STORAGE_IO_EPSIZE,
        5
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

const USB_Descriptor_String_t<13> PROGMEM ProductString =
{
    {
        USB_STRING_LEN(12),
        DTYPE_String
    },
    L"LUFA Adapter"
};

uint16_t USBSD::getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddr)
{
    const uint8_t descType = wValue >> 8;
    const uint8_t descNumber = wValue & 0xFF;
    const void* addr = NULL;
    uint16_t size = NO_DESCRIPTOR;

    switch (descType)
    {
        case DTYPE_Device:
            addr = &DeviceDescriptor;
            size = sizeof(DescDev);
            break;
        case DTYPE_Configuration:
            addr = &myConfiguration;
            size = sizeof(myConfiguration);
            break;
        case DTYPE_String:
            switch (descNumber)
            {
                case 0x00:
                    addr = &LanguageString;
                    size = pgm_read_byte(&LanguageString.Header.size);
                    break;
                case 0x01:
                    addr = &ManufacturerString;
                    size = pgm_read_byte(&ManufacturerString.Header.size);
                    break;
                case 0x02:
                    addr = &ProductString;
                    size = pgm_read_byte(&ProductString.Header.size);
                    break;
            }

            break;
    }

    *descAddr = addr;
    return size;
}

USBSD::USBSD() :
    _inpoint(MASS_STORAGE_IN_EPADDR, MASS_STORAGE_IO_EPSIZE, EP_TYPE_BULK, 1),
    _outpoint(MASS_STORAGE_OUT_EPADDR, MASS_STORAGE_IO_EPSIZE, EP_TYPE_BULK, 1)
{
    USBCON &= ~(1<<OTGPADE);
    UHWCON |= 1<<UVREGE;    // enable USB pad regulator
    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);     // disable VBUS transition interrupt
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);   // disable USB controller clock inputs
    USBCON |= 1<<USBE;      // enable USB controller clock inputs
    USBCON &= ~(1<<FRZCLK); // enable clock inputs
    PLLCSR = 0;
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);
    USBCON |= 1<<VBUSTE;    // enable VBUS transition interrupt
    configureEndpoint(_control.addr, _control.type, _control.size, 1);
    UDINT &= ~(1<<SUSPI);   // clear suspend interrupt flag
    UDIEN |= 1<<SUSPE;      // enable SUSPI interrupt
    UDIEN |= 1<<EORSTE;     // enable EORSTI interrupt
    UDCON &= ~(1<<DETACH);  // reconnect device
    USBCON |= 1<<OTGPADE;   // otgpad on
    sei();

    uint8_t prevEndpoint = getCurrentEndpoint();
    _control.select();

    if (UEINTX & 1<<RXSTPI)
        processCtrlReq();

    selectEndpoint(prevEndpoint);
}

static CmdBlockWrapper cmdBlock;
static CmdStatusWrapper cmdStatus;

void USBSD::retCmdStatus()
{
    uint16_t BytesTransferred;
    _outpoint.select();

    while (UECONX & 1<<STALLRQ)
        if (isMassStoreReset)
            return;

    _inpoint.select();

    while (UECONX & 1<<STALLRQ)
        if (isMassStoreReset)
            return;

    BytesTransferred = 0;

    while (writeStream(&cmdStatus, sizeof(cmdStatus),
                                    &BytesTransferred) == ENDPOINT_RWSTREAM_IncompleteTransfer)
    {
        if (isMassStoreReset)
            return;
    }

    UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
}

void USBSD::processCtrlReq()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        RequestHeader[i] = read8();

    if (UEINTX & 1<<RXSTPI) // endpoint isSetupReceived?
    {
        uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

        switch (USB_ControlRequest.bRequest)
        {
        case MS_REQ_MassStorageReset:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);     // clear setup
                Endpoint_ClearStatusStage();
                isMassStoreReset = true;
            }
            break;
        case MS_REQ_GetMaxLUN:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI);
                write8(1);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
                Endpoint_ClearStatusStage();
            }
            break;
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (USB_ControlRequest.bmRequestType)
                {
                    case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                        if (USB_Device_CurrentlySelfPowered)
                            CurrentStatus |= FEATURE_SELFPOWERED_ENABLED;

                        if (USB_Device_RemoteWakeupEnabled)
                            CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;

                        break;
                    case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                        selectEndpoint((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);
                        CurrentStatus = UECONX & 1<<STALLRQ;
                        selectEndpoint(ENDPOINT_CONTROLEP);
                        break;
                    default:
                        return;
                }

                UEINTX &= ~(1<<RXSTPI);
                write16(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
                Endpoint_ClearStatusStage();
            }

            break;
        case REQ_ClearFeature:
        case REQ_SetFeature:
            if ((bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                Device_ClearSetFeature();
            }

            break;
        case REQ_SetAddress:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                uint8_t DeviceAddress = USB_ControlRequest.wValue & 0x7F;
                UDADDR = UDADDR & 1<<ADDEN | DeviceAddress & 0x7F;
                UEINTX &= ~(1<<RXSTPI);
                Endpoint_ClearStatusStage();

                while ((UEINTX & 1<<TXINI) == 0)
                    ;

                UDADDR |= 1<<ADDEN; // enable dev addr
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }

            break;

        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void *DescriptorPointer;
                uint16_t descSize;

                if (USB_ControlRequest.wValue == (DTYPE_String << 8 | USE_INTERNAL_SERIAL))
                {
                    struct
                    {
                        DescHeader Header;
                        uint16_t UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
                    }
                    sigDesc;

                    sigDesc.Header.type = DTYPE_String;
                    sigDesc.Header.size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
                    Device_GetSerialString(sigDesc.UnicodeString);
                    UEINTX &= ~(1<<RXSTPI);     // clear setup
                    write_Control_Stream_LE(&sigDesc, sizeof(sigDesc));
                    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // endpoint clear out
                    return;
                }

                if ((descSize = getDescriptor(USB_ControlRequest.wValue,
                          USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
                {
                    return;
                }

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write_Control_PStream_LE(DescriptorPointer, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // endpoint clear out
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
                Endpoint_ClearStatusStage();
            }

            break;
        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                if ((uint8_t)USB_ControlRequest.wValue > FIXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
                Endpoint_ClearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                configureEndpoint(_inpoint);
                configureEndpoint(_outpoint);
            }
            break;
        default:
            break;
        }
    }

    if (UEINTX & 1<<RXSTPI)
    {
        UEINTX &= ~(1<<RXSTPI);     // clear setup
        UECONX |= 1<<STALLRQ;
    }
}

void USBSD::usbTask()
{
    uint8_t prevEndp = getCurrentEndpoint();
    _control.select();

    if (UEINTX & 1<<RXSTPI)
        processCtrlReq();

    selectEndpoint(prevEndp);
}

bool USBSD::readInCmdBlock()
{
    uint16_t bytesTransferred;
    
    _outpoint.select();
    
    if ((UEINTX & 1<<RXOUTI) == 0)  // out not received?
        return false;

    bytesTransferred = 0;

    while (readStream(&cmdBlock, sizeof(cmdBlock) - sizeof(cmdBlock.scsiCmdData),
        &bytesTransferred) == ENDPOINT_RWSTREAM_IncompleteTransfer)
    {
        if (isMassStoreReset)
            return false;
    }

    if ((cmdBlock.sig != MS_CBW_SIGNATURE) ||
        (cmdBlock.lun >= 10) ||
        (cmdBlock.flags & 0x1f) ||
        (cmdBlock.scsiCmdLen == 0) ||
        (cmdBlock.scsiCmdLen > sizeof(cmdBlock.scsiCmdData)))
    {
        UECONX |= 1<<STALLRQ;   // stall transaction
        _inpoint.select();
        UECONX |= 1<<STALLRQ;   // stall transaction
        return false;
    }

    bytesTransferred = 0;

    while (readStream(&cmdBlock.scsiCmdData, cmdBlock.scsiCmdLen, &bytesTransferred) ==
        ENDPOINT_RWSTREAM_IncompleteTransfer)
    {
        if (isMassStoreReset)
            return false;
    }

    UEINTX &= ~(1<<RXOUTI);     // clear out
    return true;
}

struct SCSI_Inquiry_Response_t
{
    uint8_t byte1;  // devType(5), peripheralQualifier(3)
    uint8_t byte2;  // reserved(7), removable(1)
    uint8_t version;
    uint8_t byte4;  // responseDataFormat(4), reserved2(1), normACA(1), trmTsk(1), aerc(1)
    uint8_t additionalLength;
    uint8_t reserved3[2];
    uint8_t byte8; // softReset(1), cmdQue(1), reserved(1), linked(1), sync(1), wideBus16(1),
            /// wideBus32(1), relAddr(1)
    uint8_t revisionID[4];
}
__attribute__ ((packed));

static const SCSI_Inquiry_Response_t inquiryData =
{
    DEVICE_TYPE_BLOCK << 3 | 0,
    1,
    0,
    2<<4 | 0<<3 | 0<<2 | 0<<1 | 0<<0,
    0x1f,
    {0, 0},
    0,
    {'0', '.', '0', '0'}
};

struct SCSI_Request_Sense_Response_t
{
    uint8_t responseCode;
    uint8_t segmentNumber;
    unsigned SenseKey : 4;
    unsigned Reserved : 1;
    unsigned ILI : 1;
    unsigned EOM : 1;
    unsigned FileMark : 1;
    uint8_t information[4];
    uint8_t additionalLength;
    uint8_t cmdSpecificInformation[4];
    uint8_t additionalSenseCode;
    uint8_t additionalSenseQualifier;
    uint8_t fieldReplaceableUnitCode;
    uint8_t senseKeySpecific[3];
}
__attribute__ ((packed));

static SCSI_Request_Sense_Response_t senseData =
{
    0x70, 0x0a
};

void SCSI_SET_SENSE(uint8_t key, uint8_t acode, uint8_t aqual)
{
    senseData.SenseKey = key;
    senseData.additionalSenseCode = acode;
    senseData.additionalSenseQualifier = aqual;
}

static bool scsiCmdRW10(bool isDataRead)
{
    Serial::instance->write("scsiCmdRW10\r\n");
    uint32_t blockAddr;
    uint16_t totalBlocks;

    if (isDataRead == DATA_WRITE && DISK_READ_ONLY)
    {
        SCSI_SET_SENSE(SCSI_SENSE_KEY_DATA_PROTECT, SCSI_ASENSE_WRITE_PROTECTED,
            SCSI_ASENSEQ_NO_QUALIFIER);
        
        return false;
    }
}

bool USBSD::decodeSCSICmd()
{
    Serial::instance->write("decodeSCSICmd()\r\n");
    char buf[50];
    snprintf(buf, 50, "%u\r\n", cmdBlock.scsiCmdData[0]);
    Serial::instance->write(buf);
    bool success = false;

    switch (cmdBlock.scsiCmdData[0])
    {
    case SCSI_CMD_INQUIRY:
    {
        uint16_t allocLen = swap16(*(uint16_t *)&cmdBlock.scsiCmdData[3]);
        uint16_t bytesTransferred = min(allocLen, (uint16_t)sizeof(inquiryData));
        char buf[50];
        snprintf(buf, 50, "allocLen: %u\r\n", allocLen);
        Serial::instance->write(buf);

        if (cmdBlock.scsiCmdData[1] & (1<<0 | 1<<1) || cmdBlock.scsiCmdData[2])
        {
            SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST, SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                    SCSI_ASENSEQ_NO_QUALIFIER);

            break;
        }

        Serial::instance->write("scsiCmdInq() - 2\r\n");
        writeStream(&inquiryData, bytesTransferred, NULL);
        nullStream(allocLen - bytesTransferred, NULL);
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        cmdBlock.dataTransferLen -= bytesTransferred;
        success = true;
    }
        break;
    case SCSI_CMD_REQUEST_SENSE:
    {
        uint8_t allocLen = cmdBlock.scsiCmdData[4];
        uint8_t bytesTransferred = min(allocLen, (uint8_t)sizeof(senseData));
        writeStream(&senseData, bytesTransferred, NULL);
        nullStream(allocLen - bytesTransferred, NULL);
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        cmdBlock.dataTransferLen -= bytesTransferred;
        success = true;
    }
        break;
    case SCSI_CMD_READ_CAPACITY_10:
        Serial::instance->write("readCap10()\r\n");
        write32(LUN_MEDIA_BLOCKS - 1);
        write32(VIRTUAL_MEMORY_BLOCK_SIZE);

        if (isMassStoreReset)
            break;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        cmdBlock.dataTransferLen -= 8;
        success = true;
        break;
    case SCSI_CMD_SEND_DIAGNOSTIC:
        if ((cmdBlock.scsiCmdData[1] & 1<<2) == 0)
        {
            SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST, SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                SCSI_ASENSEQ_NO_QUALIFIER);

            break;
        }

        cmdBlock.dataTransferLen = 0;
        success = true;
        break;
    case SCSI_CMD_WRITE_10:
        scsiCmdRW10(DATA_WRITE);
        success = true;
        break;
    case SCSI_CMD_READ_10:
        scsiCmdRW10(DATA_READ);
        success = true;
        break;
    case SCSI_CMD_MODE_SENSE_6:
        write8(0);
        write8(0);
        write8(0);  // DISK read-write
        write8(0);
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // endpoint clear in
        cmdBlock.dataTransferLen -= 4;
        success = true;
        break;
    case SCSI_CMD_START_STOP_UNIT:
    case SCSI_CMD_TEST_UNIT_READY:
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
    case SCSI_CMD_VERIFY_10:
        success = true;
        break;
    default:
        SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST, SCSI_ASENSE_INVALID_COMMAND,
            SCSI_ASENSEQ_NO_QUALIFIER);

        break;
    }

    if (success)
    {
        SCSI_SET_SENSE(SCSI_SENSE_KEY_GOOD, SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
            SCSI_ASENSEQ_NO_QUALIFIER);

        return true;
    }

    return false;
}

void USBSD::msTask()
{
    if (state != DEVICE_STATE_Configured)
        return;

    if (readInCmdBlock())
    {
        if (cmdBlock.flags & MS_COMMAND_DIR_DATA_IN)
            _inpoint.select();

        cmdStatus.status = decodeSCSICmd() ? MS_SCSI_COMMAND_Pass : MS_SCSI_COMMAND_Fail;
        cmdStatus.tag = cmdBlock.tag;
        cmdStatus.dataTransferResidue = cmdBlock.dataTransferLen;

        if (cmdStatus.status == MS_SCSI_COMMAND_Fail && cmdStatus.dataTransferResidue)
            UECONX |= 1<<STALLRQ;

        retCmdStatus();
    }

    if (isMassStoreReset)
    {
        _outpoint.reset();
        _inpoint.reset();
        _outpoint.select();
        UECONX |= 1<<STALLRQ;       // clear stall
        UECONX |= 1<<RSTDT;         // reset data toggle
        _inpoint.select();
        UECONX |= 1<<STALLRQ;       // clear stall
        UECONX |= 1<<RSTDT;         // reset data toggle
        isMassStoreReset = false;
    }
}

void USBSD::gen()
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE) // start of frame
        UDINT &= ~(1<<SOFI);

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)   // usb connect
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
            {
                PLLCSR = USB_PLL_PSC;
                PLLCSR = USB_PLL_PSC | 1<<PLLE;
                while (!(PLLCSR & 1<<PLOCK));
            }

            state = DEVICE_STATE_Powered;
        }
        else    // disconnect
        {
            if (!(USB_Options & USB_OPT_MANUAL_PLL))
                PLLCSR = 0;

            state = DEVICE_STATE_Unattached;
        }
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<SUSPE)  // suspend
    {
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPI;
        USBCON |= 1<<FRZCLK;

        if (!(USB_Options & USB_OPT_MANUAL_PLL))
            PLLCSR = 0;

        state = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)   // wakeup
    {
        if (!(USB_Options & USB_OPT_MANUAL_PLL))
        {
            PLLCSR = USB_PLL_PSC;
            PLLCSR = USB_PLL_PSC | 1<<PLLE;   // PLL on
            while (!(PLLCSR & 1<<PLOCK));   // PLL is ready?
        }

        USBCON &= ~(1<<FRZCLK);
        UDINT &= ~(1<<WAKEUPI);
        UDIEN &= ~(1<<WAKEUPI);
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            state = DEVICE_STATE_Configured;
        else
            state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE) // reset
    {
        UDINT &= ~(1<<EORSTI);
        state = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        configureEndpoint(_control.addr, _control.type, _control.size, 1);
        UEIENX |= 1<<RXSTPE;
    }
}

void USBSD::com()
{
    uint8_t prevEndp = getCurrentEndpoint();
    _control.select();
    UEIENX &= ~(1<<RXSTPE);
    sei();
    processCtrlReq();
    _control.select();
    UEIENX |= 1<<RXSTPE;
    selectEndpoint(prevEndp);
}

ISR(USB_GEN_vect)
{
    USBSD::instance->gen();
}

ISR(USB_COM_vect)
{
    USBSD::instance->com();
}


