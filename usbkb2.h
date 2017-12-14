#ifndef _USBKB2_H_
#define _USBKB2_H_

#define F_CPU 16000000UL

#include "busby.h"

static constexpr uint8_t
    CDC_DSUBTYPE_CSInterface_Header           = 0x00,
    CDC_DSUBTYPE_CSInterface_CallManagement   = 0x01,
    CDC_DSUBTYPE_CSInterface_ACM              = 0x02,
    CDC_DSUBTYPE_CSInterface_DirectLine       = 0x03,
    CDC_DSUBTYPE_CSInterface_TelephoneRinger  = 0x04,
    CDC_DSUBTYPE_CSInterface_TelephoneCall    = 0x05,
    CDC_DSUBTYPE_CSInterface_Union            = 0x06,
    CDC_DSUBTYPE_CSInterface_CountrySelection = 0x07,
    CDC_DSUBTYPE_CSInterface_TelephoneOpModes = 0x08,
    CDC_DSUBTYPE_CSInterface_USBTerminal      = 0x09,
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
    CDC_TXRX_EPSIZE = 16,
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
    CDC_REQ_SendBreak = 0x23,
    CDC_NOTIF_SerialState = 0x20,
    CDC_LINEENCODING_OneStopBit          = 0,
    CDC_LINEENCODING_OneAndAHalfStopBits = 1,
    CDC_LINEENCODING_TwoStopBits         = 2,
    CDC_PARITY_None  = 0,
    CDC_PARITY_Odd   = 1,
    CDC_PARITY_Even  = 2,
    CDC_PARITY_Mark  = 3,
    CDC_PARITY_Space = 4;

struct CDC_LineEncoding_t
{
    uint32_t BaudRateBPS;
    uint8_t CharFormat;
    uint8_t ParityType;
    uint8_t DataBits;
};

class USBKB : public USB
{
private:
    Endpoint _inpoint;
    Endpoint _outpoint;
    Endpoint _notif;
    CDC_LineEncoding_t _lineEncoding;
    bool configureEndpoints();
    void EVENT_USB_Device_ControlRequest();
    void Device_ProcessControlRequest();
    bool Endpoint_ConfigureEndpointTable(Endpoint *table, uint8_t entries);
    uint16_t getDescriptor(uint16_t wValue, uint8_t wIndex, const void **descAddress);
public:
    int16_t receive();
    uint8_t sendByte(uint8_t data);
    USBKB();
    uint8_t flush();
    void gen();
    void com();
};

struct USB_CDC_Descriptor_FunctionalHeader_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint16_t CDCSpecification;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalHeader_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint16_t bcdCDC;
}
__attribute__ ((packed));

struct USB_CDC_Descriptor_FunctionalACM_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t Capabilities;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalACM_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
}
__attribute__ ((packed));

struct USB_CDC_Descriptor_FunctionalUnion_t
{
    DescHeader Header;
    uint8_t Subtype;
    uint8_t MasterInterfaceNumber;
    uint8_t SlaveInterfaceNumber;
}
__attribute__ ((packed));

struct USB_CDC_StdDescriptor_FunctionalUnion_t
{
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface0;
}
__attribute__ ((packed));

struct USB_HID_Descriptor_HID_t
{
    uint16_t HIDSpec;
    uint8_t CountryCode;
    uint8_t TotalReportDescriptors;
    uint8_t HIDReportType;
    uint16_t HIDReportLength;
}
__attribute__ ((packed));

struct USB_Descriptor_Configuration_t
{
    DescConf Config;
    DescIface HID_Interface;
    USB_HID_Descriptor_HID_t HID_KeyboardHID;
    DescEndpoint HID_ReportINEndpoint;
}
__attribute__ ((packed));


#endif



