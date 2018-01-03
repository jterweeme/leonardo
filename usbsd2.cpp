#include "usbsd2.h"

template <class T> const T& min(const T& a, const T& b) { return !(b < a) ? a : b; }

#ifdef DEBUG
Serial *g_serial;
#endif

static inline uint16_t zwapEndian_16(uint16_t word)
{
    return word >> 8 | word << 8;
}

static inline uint32_t zwapEndian_32(const uint32_t dword)
{
    return dword >> 24 & 0xff | dword << 8 & 0xff0000 |
        dword >> 8 & 0xff00 | dword << 24 & 0xff000000;
}

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
    0,
    0,
    0,
    8,
    0x03EB,
    0x2045,
    0x0001,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    UZE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};


volatile bool IsMassStoreReset = false;

static constexpr bool DATA_READ = true, DATA_WRITE = false;
static constexpr uint8_t DEVICE_TYPE_BLOCK = 0;

const MyConf PROGMEM myConf =
{
    {
        {
            sizeof(DescConf),
            DTYPE_Configuration
        },

        sizeof(MyConf),
        1,
        1,
        0,    // no descriptor
        USB_CONFYG_ATTR_RESERVED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        {
            sizeof(DescIface),
            DTYPE_Interface
        },

        0, // mass storage
        0,
        2,
        MS_CZCP_MassStorageClass,
        MS_CZCP_SCSITransparentSubclass,
        MS_CZCP_BulkOnlyTransportProtocol,
        0
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        MASS_STORAGE_IN_EPADDR,
        E2P_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        MASS_STORAGE_IO_EPSIZE,
        0x05
    },
    {
        {
            sizeof(DescEndpoint),
            DTYPE_Endpoint
        },

        MASS_STORAGE_OUT_EPADDR,
        E2P_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
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
    const uint8_t DescriptorNumber = wValue & 0xFF;
    const void *Address = NULL;
    uint16_t Size = 0;

    switch (wValue >> 8)
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
            Size = pgm_read_byte(&LanguageString.header.size);
            break;
        case STRING_ID_Manufacturer:
            Address = &ManufacturerString;
            Size = pgm_read_byte(&ManufacturerString.header.size);
            break;
        case STRING_ID_Product:
            Address = &ProductString;
            Size = pgm_read_byte(&ProductString.header.size);
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

bool USBSD::decodeSCSICmd()
{
    bool success = false;

    switch (cmdBlock.SCSICommandData[0])
    {
        case ZCZI_CMD_INQUIRY:
        {
            uint16_t allocLen = zwapEndian_16(*(uint16_t*)&cmdBlock.SCSICommandData[3]);
            uint16_t BytesTransferred = min(allocLen, sizeof(InquiryData));

            if ((cmdBlock.SCSICommandData[1] & (1<<0 | 1<<1)) || cmdBlock.SCSICommandData[2])
            {
                SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
                SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_FIELD_IN_CDB;
                SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
                break;
            }

            writeStream2(&InquiryData, BytesTransferred, NULL);
            nullStream(allocLen - BytesTransferred, NULL);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            cmdBlock.DataTransferLength -= BytesTransferred;
            success = true;
        }
            break;
        case ZCZI_CMD_REQUEST_SENSE:
        {
            uint8_t AllocationLength = cmdBlock.SCSICommandData[4];
            uint8_t BytesTransferred = min((size_t)AllocationLength, sizeof(SenseData));
            writeStream2(&SenseData, BytesTransferred, NULL);
            nullStream(AllocationLength - BytesTransferred, NULL);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            cmdBlock.DataTransferLength -= BytesTransferred;
            success = true;
        }
            break;
        case ZCZI_CMD_READ_CAPACITY_10:
            write32be(LUN_MEDIA_BLOCKS - 1);
            write32be(VIRTUAL_MEMORY_BLOCK_SIZE);

            if (IsMassStoreReset)
                break;

            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            cmdBlock.DataTransferLength -= 8;
            success = true;
            break;
        case ZCZI_CMD_SEND_DIAGNOSTIC:

            if ((cmdBlock.SCSICommandData[1] & 1<<2) == 0)
            {
                SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
                SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_FIELD_IN_CDB;
                SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
                break;
            }

            cmdBlock.DataTransferLength = 0;
            success = true;
            break;
        case ZCZI_CMD_WRITE_10:
            success = SCSI_Command_ReadWrite_10(DATA_WRITE);
            break;
        case ZCZI_CMD_READ_10:
            success = SCSI_Command_ReadWrite_10(DATA_READ);
            break;
        case ZCZI_CMD_MODE_SENSE_6:
            write8(0x00);
            write8(0x00);
            write8(DISK_READ_ONLY ? 0x80 : 0x00);
            write8(0x00);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
            cmdBlock.DataTransferLength -= 4;
            success = true;
            break;
        case ZCZI_CMD_START_STOP_UNIT:
        case ZCZI_CMD_TEST_UNIT_READY:
        case ZCZI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case ZCZI_CMD_VERIFY_10:
            success = true;
            cmdBlock.DataTransferLength = 0;
            break;
        default:
            SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
            SenseData.AdditionalSenseCode = ZCZI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
            break;
    }

    if (success)
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

    if (IsDataRead == DATA_WRITE && DISK_READ_ONLY)
    {
        SenseData.vanalles = ZCZI_SENSE_KEY_DATA_PROTECT << 4;
        SenseData.AdditionalSenseCode = ZCZI_ASENSE_WRITE_PROTECTED;
        SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
        return false;
    }

    BlockAddress = zwapEndian_32(*(uint32_t*)&cmdBlock.SCSICommandData[2]);
    totalBlocks = zwapEndian_16(*(uint16_t*)&cmdBlock.SCSICommandData[7]);

    if (BlockAddress >= LUN_MEDIA_BLOCKS)
    {
        SenseData.vanalles = ZCZI_SENSE_KEY_ILLEGAL_REQUEST << 4;
        SenseData.AdditionalSenseCode = ZCZI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = ZCZI_ASENSEQ_NO_QUALIFIER;
        return false;
    }

#ifdef DEBUG
    char buf[50];
    snprintf(buf, 50, "%u ", BlockAddress);
    g_serial->write(buf);
    snprintf(buf, 50, "%u\r\n", totalBlocks);
    g_serial->write(buf);
#endif

    if (IsDataRead == DATA_READ)
        DataflashManager_ReadBlocks(BlockAddress, totalBlocks);
#if 0
    else
        DataflashManager_WriteBlocks(BlockAddress, totalBlocks);
#endif
    cmdBlock.DataTransferLength -= ((uint32_t)totalBlocks * VIRTUAL_MEMORY_BLOCK_SIZE);
    return true;
}

void USBSD::DataflashManager_ReadBlocks(const uint32_t BlockAddress, uint16_t TotalBlocks)
{
    if (waitUntilReady())
        return;

    for (uint16_t i = 0; i < TotalBlocks; i++)
    {
        uint8_t BytesInBlockDiv16 = 0;
        uint16_t k = 0;

        while (BytesInBlockDiv16 < (VIRTUAL_MEMORY_BLOCK_SIZE >> 4))
        {
            if ((UEINTX & 1<<RWAL) == 0)    //read-write allowed?
            {
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in

                if (waitUntilReady())
                    return;
            }

            uint8_t buf[16];
            _sd.readData(BlockAddress, i + k++ * 16, 16, buf);

            for (uint8_t j = 0; j < 16; j++)
                write8(buf[j]);

            BytesInBlockDiv16++;

            if (IsMassStoreReset)
                return;
        }
    }

    if ((UEINTX & 1<<RWAL) == 0)    // read-write allowed?
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
}

void USBSD::MassStorage_Task()
{
	if (state != DEVICE_STATE_Configured)
	    return;

    if (ReadInCommandBlock())
    {
        if (cmdBlock.Flags & MS_KOMMAND_DIR_DATA_IN)
            selectEndpoint(MASS_STORAGE_IN_EPADDR);

		cmdStatus.Status = decodeSCSICmd() ? MS_ZCZI_COMMAND_Pass : MS_ZCZI_COMMAND_Fail;
		cmdStatus.Tag = cmdBlock.Tag;
		cmdStatus.DataTransferResidue = cmdBlock.DataTransferLength;

		if ((cmdStatus.Status == MS_ZCZI_COMMAND_Fail) && cmdStatus.DataTransferResidue)
		    UECONX |= 1<<STALLRQ;   // stall transaction

		ReturnCommandStatus();
	}

	if (IsMassStoreReset)
	{
        _outpoint.reset();
        _inpoint.reset();
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

	while (readStream(&cmdBlock, (sizeof(cmdBlock) -
        sizeof(cmdBlock.SCSICommandData)),
	    &BytesTransferred) == ENDPOINT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return false;
	}

	if (cmdBlock.Signature != M2S_CBW_SIGNATURE || cmdBlock.LUN >= TOTAL_LUNS ||
        cmdBlock.Flags & 0x1F || cmdBlock.SCSICommandLength == 0 ||
		(cmdBlock.SCSICommandLength > sizeof(cmdBlock.SCSICommandData)))
	{
        UECONX |= 1<<STALLRQ;
		selectEndpoint(MASS_STORAGE_IN_EPADDR);
        UECONX |= 1<<STALLRQ;       // stall transaction
		return false;
	}

	BytesTransferred = 0;

	while (readStream(&cmdBlock.SCSICommandData, cmdBlock.SCSICommandLength,
	                               &BytesTransferred) == ENDPOINT_RWSTREAM_IncompleteTransfer)
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

	while (writeStream2(&cmdStatus, sizeof(cmdStatus),
	                                &BytesTransferred) == ENDPOINT_RWSTREAM_IncompleteTransfer)
	{
		if (IsMassStoreReset)
		    return;
	}

    UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
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
    UDADDR = UDADDR & 1<<ADDEN | Address & 0x7F;
}

void USBSD::Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&_ctrlReq;

    for (uint8_t i = 0; i < sizeof(USBRequest); i++)
        *(RequestHeader++) = read8();

    if (UEINTX & 1<<RXSTPI)
    {
        const uint8_t bmRequestType = _ctrlReq.bmRequestType;

        switch (_ctrlReq.bRequest)
        {
        case M2S_REQ_MassStorageReset:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                IsMassStoreReset = true;
            }

            break;
        case M2S_REQ_GetMaxLUN:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                UEINTX &= ~(1<<RXSTPI); // clear setup
                write8(TOTAL_LUNS - 1);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }
            break;
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;

                switch (bmRequestType)
                {
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                {
                    if (USB_Device_CurrentlySelfPowered)
                        CurrentStatus |= FEATURE_SELFPOWERED_ENABLED;

                    if (USB_Device_RemoteWakeupEnabled)
                        CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;
                }
                    break;
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                {
                    uint8_t endpointIndex = ((uint8_t)_ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);

                    if (endpointIndex >= ENDPOINT_TOTAL_ENDPOINTS)
                        return;

                    selectEndpoint(endpointIndex);
                    CurrentStatus = UECONX & 1<<STALLRQ;
                    _control.select();
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
                uint8_t DeviceAddress = _ctrlReq.wValue & 0x7F;
                setDeviceAddress(DeviceAddress);
                UEINTX &= ~(1<<RXSTPI); // clear setup
                clearStatusStage();
                while ((UEINTX & 1<<TXINI) == 0);   // in ready?
                UDADDR |= 1<<ADDEN; // enable dev addr
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void *descPtr;
                uint16_t descSize;

                if (_ctrlReq.wValue == (DTYPE_String << 8 | UZE_INTERNAL_SERIAL))
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

                if ((descSize = getDescriptor(_ctrlReq.wValue, _ctrlReq.wIndex, &descPtr)) == 0)
                    return;

                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write_Control_PStream_LE(descPtr, descSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);    // clear out
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                UEINTX &= ~(1<<RXSTPI);     // clear setup
                write8(USB_Device_ConfigurationNumber);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON); // clear in
                clearStatusStage();
            }
            break;
        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                if ((uint8_t)_ctrlReq.wValue > FIXED_NUM_CONFIGURATIONS)
                    return;

                UEINTX &= ~(1<<RXSTPI);
                USB_Device_ConfigurationNumber = (uint8_t)_ctrlReq.wValue;
                clearStatusStage();

                if (USB_Device_ConfigurationNumber)
                    state = DEVICE_STATE_Configured;
                else
                    state = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

                configureEndpoint(_inpoint);
                configureEndpoint(_outpoint);
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

USBSD::USBSD() :
    _sd(&_board.pin9),
    _inpoint(MASS_STORAGE_IN_EPADDR, MASS_STORAGE_IO_EPSIZE, EP_TYPE_BULK, 1),
    _outpoint(MASS_STORAGE_OUT_EPADDR, MASS_STORAGE_IO_EPSIZE, EP_TYPE_BULK, 1)
{
    _sd.init(SPI_HALF_SPEED);
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





