#include "cdc.h"
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "misc.h"
#include <stdio.h>

static constexpr uint8_t
    STRING_ID_LANGUAGE = 0,
    STRING_ID_MANUFACTURER = 1,
    STRING_ID_PRODUCT = 2,
    CDC_CSCP_CDCClass = 0x02,
    CDC_CSCP_NoSpecificSubclass = 0x00,
    CDC_CSCP_ACMSubclass = 0x02,
    CDC_CSCP_ATCommandProtocol = 0x01,
    CDC_CSCP_NoSpecificProtocol = 0x00,
    CDC_CSCP_VendorSpecificProtocol = 0xFF,
    CDC_CSCP_CDCDataClass = 0x0A,
    CDC_CSCP_NoDataSubclass = 0x00,
    CDC_CSCP_NoDataProtocol = 0x00,
    CDC_REQ_SendEncapsulatedCommand = 0x00,
    CDC_REQ_GetEncapsulatedResponse = 0x01,
    CDC_REQ_SetLineEncoding = 0x20,
    CDC_REQ_GetLineEncoding = 0x21,
    CDC_REQ_SetControlLineState = 0x22,
    CDC_REQ_SENDBREAK = 0x23,
    CDC_NOTIF_SerialState = 0x20,
    CDC_LINEENCODING_OneStopBit = 0,
    CDC_LINEENCODING_OneAndAHalfStopBits = 1,
    CDC_LINEENCODING_TwoStopBits = 2,
    CDC_DSUBTYPE_CSInterface_Header = 0x00,
    CDC_DSUBTYPE_CSInterface_CallManagement = 0x01,
    CDC_DSUBTYPE_CSInterface_ACM = 0x02,
    CDC_DSUBTYPE_CSInterface_DirectLine       = 0x03,
    CDC_DSUBTYPE_CSInterface_TelephoneRinger = 0x04,
    CDC_DSUBTYPE_CSInterface_TelephoneCall = 0x05,
    CDC_DSUBTYPE_CSInterface_Union            = 0x06,
    CDC_DSUBTYPE_CSInterface_CountrySelection = 0x07,
    CDC_DSUBTYPE_CSInterface_TelephoneOpModes = 0x08,
    CDC_DSUBTYPE_CSInterface_USBTerminal = 0x09,
    CDC_DSUBTYPE_CSInterface_NetworkChannel   = 0x0A,
    CDC_DSUBTYPE_CSInterface_ProtocolUnit     = 0x0B,
    CDC_DSUBTYPE_CSInterface_ExtensionUnit    = 0x0C,
    CDC_DSUBTYPE_CSInterface_MultiChannel     = 0x0D,
    CDC_DSUBTYPE_CSInterface_CAPI             = 0x0E,
    CDC_DSUBTYPE_CSInterface_Ethernet         = 0x0F,
    CDC_DSUBTYPE_CSInterface_ATM              = 0x10,
    CDC_NOTIFICATION_EPADDR = ENDPOINT_DIR_IN | 2,
    CDC_TX_EPADDR = ENDPOINT_DIR_IN | 3,
    CDC_RX_EPADDR = ENDPOINT_DIR_OUT | 4,
    CDC_NOTIFICATION_EPSIZE = 8,
    CDC_TXRX_EPSIZE = 16;

static const DescDev PROGMEM DeviceDescriptor =
{
    sizeof(DescDev),
    DTYPE_DEVICE,
    0x0110,
    CDC_CSCP_CDCClass,
    CDC_CSCP_NoSpecificSubclass,
    CDC_CSCP_NoSpecificProtocol,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x204B,
    0x0001,
    0x01,
    0x02,
    USE_INTERNAL_SERIAL,
    FIXED_NUM_CONFIGURATIONS
};

struct CDC_Header
{
    uint8_t size;
    uint8_t type;
    uint8_t subtype;
    uint16_t cdcSpec;
}
__attribute__ ((packed));

struct CDC_ACM
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint8_t Capabilities;
}
__attribute__ ((packed));

struct CDC_Union
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint8_t MasterInterfaceNumber;
    uint8_t SlaveInterfaceNumber;
}
__attribute__ ((packed));

struct MyConf
{
    DescConf config;
    DescIface cci;
    CDC_Header cdcHeader;
    CDC_ACM cdcACM;
    CDC_Union cdcUnion;
    DescEndpoint notif;
    DescIface dci;
    DescEndpoint outpoint;
    DescEndpoint inpoint;
}
__attribute__ ((packed));

static const MyConf PROGMEM myConf =
{
    {
        sizeof(DescConf),
        DTYPE_CONFIGURATION,
        sizeof(MyConf),
        2,
        1,
        NO_DESCRIPTOR,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        0,
        0,
        1,
        CDC_CSCP_CDCClass,
        CDC_CSCP_ACMSubclass,
        CDC_CSCP_ATCommandProtocol,
        NO_DESCRIPTOR
    },
    {
        sizeof(CDC_Header),
        DTYPE_CSInterface,
        CDC_DSUBTYPE_CSInterface_Header,
        0x0110,
    },
    {
        sizeof(CDC_ACM),
        DTYPE_CSInterface,
        CDC_DSUBTYPE_CSInterface_ACM,
        0x06,
    },
    {
        sizeof(CDC_Union),
        DTYPE_CSInterface,
        CDC_DSUBTYPE_CSInterface_Union,
        0,
        1,
    },
    {
        sizeof(DescEndpoint),
        DTYPE_Endpoint,
        CDC_NOTIFICATION_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        CDC_NOTIFICATION_EPSIZE,
        0xFF
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        1,
        0,
        2,
        CDC_CSCP_CDCDataClass,
        CDC_CSCP_NoDataSubclass,
        CDC_CSCP_NoDataProtocol,
        0
    },
    {
        sizeof(DescEndpoint),
        DTYPE_Endpoint,
        CDC_RX_EPADDR,
        EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        CDC_TXRX_EPSIZE,
        0x05
    },
    {
        sizeof(DescEndpoint),
        DTYPE_Endpoint,
        CDC_TX_EPADDR,
        EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        CDC_TXRX_EPSIZE,
        0x05
    }
};

static const DescString<2> PROGMEM languageString =
{
    USB_STRING_LEN(1),
    DTYPE_STRING,
    (wchar_t)0x0409
};

static const DescString<12> PROGMEM manufacturerString =
{
    USB_STRING_LEN(11),
    DTYPE_STRING,
    L"Dean Camera"
};

static const DescString<23> PROGMEM productString =
{
    USB_STRING_LEN(22),
    DTYPE_STRING,
    L"LUFA USB-RS232 Adapter"
};

uint16_t CDC::getDesc(uint16_t wValue, uint16_t wIndex, const void ** const descAddress)
{
    const uint8_t descNumber = wValue & 0xFF;
    const void* addr = NULL;
    uint16_t size = 0;

    switch (wValue >> 8)
    {
    case DTYPE_DEVICE:
        addr = &DeviceDescriptor;
        size = sizeof(DescDev);
        break;
    case DTYPE_CONFIGURATION:
        addr = &myConf;
        size = sizeof(myConf);
        break;
    case DTYPE_STRING:
        switch (descNumber)
        {
        case STRING_ID_LANGUAGE:
            addr = &languageString;
            size = pgm_read_byte(&languageString.size);
            break;
        case STRING_ID_MANUFACTURER:
            addr = &manufacturerString;
            size = pgm_read_byte(&manufacturerString.size);
            break;
        case STRING_ID_PRODUCT:
            addr = &productString;
            size = pgm_read_byte(&productString.size);
            break;
        }

        break;
    }

    *descAddress = addr;
    return size;
}

uint8_t CDC::sendByte(uint8_t Data)
{
    if (state != DEVICE_STATE_Configured)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    _inpoint.select();

    if ((*p_ueintx & 1<<rwal) == 0)
    {
        *p_ueintx &= ~(1<<txini | 1<<fifocon);
        uint8_t ErrorCode;

        if ((ErrorCode = waitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;
    }

    write8(Data);
    return ENDPOINT_READYWAIT_NoError;
}

uint8_t CDC::flush()
{
    if ((state != DEVICE_STATE_Configured) || !_lineEncoding.BaudRateBPS)
        return ENDPOINT_RWSTREAM_DeviceDisconnected;

    uint8_t ErrorCode;
    _inpoint.select();

    if (!(bytesInEndpoint()))
        return ENDPOINT_READYWAIT_NoError;

    bool bankFull = !(*p_ueintx & 1<<rwal);
    *p_ueintx &= ~(1<<txini | 1<<fifocon);

    if (bankFull)
    {
        if ((ErrorCode = waitUntilReady()) != ENDPOINT_READYWAIT_NoError)
            return ErrorCode;

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    }

    return ENDPOINT_READYWAIT_NoError;
}

int16_t CDC::receive()
{
    if ((state != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return -1;

    int16_t ReceivedByte = -1;
    _outpoint.select();

    if (*p_ueintx & 1<<rxouti) // is OUT received?
    {
        if (bytesInEndpoint())
            ReceivedByte = read8();

        if (!(bytesInEndpoint()))
            *p_ueintx &= ~(1<<rxouti | 1<<fifocon);
    }

    return ReceivedByte;
}

CDC::CDC() :
    _inpoint(CDC_TX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _outpoint(CDC_RX_EPADDR, CDC_TXRX_EPSIZE, EP_TYPE_BULK, 0),
    _notif(CDC_NOTIFICATION_EPADDR, CDC_NOTIFICATION_EPSIZE, EP_TYPE_INTERRUPT, 0)
{
    *p_usbcon &= ~(1<<otgpade);
    *p_uhwcon |= 1<<uvrege;    // enable USB pad regulator
    *p_usbcon &= ~(1<<vbuste);     // disable VBUS transition interrupt
    *p_udien = 0;
    *p_usbint = 0;
    *p_udint = 0;
    *p_usbcon &= ~(1<<usbe);   // disable USB controller clock inputs
    *p_usbcon |= 1<<usbe;      // enable USB controller clock inputs
    *p_usbcon &= ~(1<<frzclk); // enable clock inputs
    *p_pllcsr = 0;
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    *p_udcon &= ~(1<<lsm);
    *p_usbcon |= 1<<vbuste;    // enable VBUS transition interrupt
    configureEndpoint(_control.addr, _control.type, _control.size, 1);
    *p_udint &= ~(1<<suspi);   // clear suspend interrupt flag
    *p_udien |= 1<<suspe;      // enable SUSPI interrupt
    *p_udien |= 1<<eorste;     // enable EORSTI interrupt
    *p_udcon &= ~(1<<detach);  // reconnect device
    *p_usbcon |= 1<<otgpade;   // otgpad on
    sei();

    if ((state != DEVICE_STATE_Configured) || !(_lineEncoding.BaudRateBPS))
        return;

    _inpoint.select();

    if (*p_ueintx & 1<<txini)
        flush();

    if (state == DEVICE_STATE_Unattached)
        return;

    uint8_t prevEndpoint = getCurrentEndpoint();
    _control.select();

    if (*p_ueintx & 1<<rxstpi)
        procCtrlReq();

    selectEndpoint(prevEndpoint);
}

void CDC::customCtrl()
{
    if ((UEINTX & 1<<RXSTPI) == 0)
        return;

    if (_ctrlReq.wIndex != _control.addr)
        return;

    switch (_ctrlReq.bRequest)
    {
    case CDC_REQ_GetLineEncoding:
        if (_ctrlReq.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
        {
            *p_ueintx &= ~(1<<RXSTPI);
            while ((UEINTX & 1<<TXINI) == 0);
            write32(_lineEncoding.BaudRateBPS);
            write8(_lineEncoding.CharFormat);
            write8(_lineEncoding.ParityType);
            write8(_lineEncoding.DataBits);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            clearStatusStage();
        }
        break;
    case CDC_REQ_SetLineEncoding:
        if (_ctrlReq.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
        {
            UEINTX &= ~(1<<RXSTPI);

            while ((UEINTX & 1<<RXOUTI) == 0)
                if (state == DEVICE_STATE_Unattached)
                    return;

            _lineEncoding.BaudRateBPS = read32();
            _lineEncoding.CharFormat = read8();
            _lineEncoding.ParityType = read8();
            _lineEncoding.DataBits = read8();
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
            clearStatusStage();
        }

        break;
    case CDC_REQ_SetControlLineState:
        if (_ctrlReq.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
        {
            *p_ueintx &= ~(1<<rxstpi);
            clearStatusStage();
        }
        break;
    case CDC_REQ_SENDBREAK:
        if (_ctrlReq.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
        {
            *p_ueintx &= ~(1<<rxstpi);
            clearStatusStage();
        }

        break;
    }
}

void CDC::configure()
{
    memset(&_lineEncoding, 0, sizeof(_lineEncoding));
    configureEndpoint(_inpoint);
    configureEndpoint(_outpoint);
    configureEndpoint(_notif);
}







