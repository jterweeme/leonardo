#ifndef _BUSBY_H_
#define _BUSBY_H_
#include <stdint.h>
#include <avr/io.h>

#define ENDPOINT_DIR_MASK                  0x80
#define ENDPOINT_DIR_OUT                   0x00
#define ENDPOINT_DIR_IN                    0x80
#define EP_TYPE_CONTROL                    0x00
#define EP_TYPE_ISOCHRONOUS                0x01
#define EP_TYPE_BULK                       0x02
#define EP_TYPE_INTERRUPT                  0x03
#define ENDPOINT_EPNUM_MASK                     0x0F
#define ENDPOINT_CONTROLEP                      0
#define NO_DESCRIPTOR    0
#define USB_CONFIG_POWER_MA(mA)           ((mA) >> 1)

static constexpr uint8_t
    USB_CSCP_NoDeviceClass          = 0x00,
    USB_CSCP_NoDeviceSubclass       = 0x00,
    USB_CSCP_NoDeviceProtocol       = 0x00,
    USB_CSCP_VendorSpecificClass    = 0xFF,
    USB_CSCP_VendorSpecificSubclass = 0xFF,
    USB_CSCP_VendorSpecificProtocol = 0xFF,
    USB_CSCP_IADDeviceClass         = 0xEF,
    USB_CSCP_IADDeviceSubclass      = 0x02,
    USB_CSCP_IADDeviceProtocol      = 0x01,
    ENDPOINT_RWSTREAM_NoError            = 0,
    ENDPOINT_RWSTREAM_EndpointStalled    = 1,
    ENDPOINT_RWSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWSTREAM_BusSuspended       = 3,
    ENDPOINT_RWSTREAM_Timeout            = 4,
    ENDPOINT_RWSTREAM_IncompleteTransfer = 5;

enum USB_DescriptorTypes_t
{
    DTYPE_Device = 0x01,
    DTYPE_Configuration = 0x02,
    DTYPE_String = 0x03, /**< Indicates that the descriptor is a string descriptor. */
    DTYPE_Interface = 0x04, /**< hat the descriptor is an interface descriptor. */
    DTYPE_Endpoint = 0x05, /**< In the descriptor is an endpoint descriptor. */
    DTYPE_DeviceQualifier = 0x06, /**< Ir is a device qualifier descriptor. */
    DTYPE_Other = 0x07, /**< Indicates that the descriptor is of other type. */
    DTYPE_InterfacePower = 0x08, /**< descriptor is an interface power descriptor. */
    DTYPE_InterfaceAssociation      = 0x0B,
    DTYPE_CSInterface               = 0x24,
    DTYPE_CSEndpoint                = 0x25,
};


enum Endpoint_WaitUntilReady_ErrorCodes_t
{
    ENDPOINT_READYWAIT_NoError = 0, /**< Endpoint is ready for next packet, no error. */
    ENDPOINT_READYWAIT_EndpointStalled = 1, /**< The endpoint was stalled during the stream*/
    ENDPOINT_READYWAIT_DeviceDisconnected  = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
};

enum Endpoint_ControlStream_RW_ErrorCodes_t
{
    ENDPOINT_RWCSTREAM_NoError = 0, /**< Command completed successfully, no error. */
    ENDPOINT_RWCSTREAM_HostAborted = 1, /**< The aborted the transfer prematurely. */
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended = 3,
};

enum USB_Device_States_t
{
    DEVICE_STATE_Unattached = 0,
    DEVICE_STATE_Powered = 1,
    DEVICE_STATE_Default = 2,
    DEVICE_STATE_Addressed  = 3,
    DEVICE_STATE_Configured = 4,
    DEVICE_STATE_Suspended = 5,
};

struct USB_Request_Header_t
{
    uint8_t  bmRequestType; /**< Type of the request. */
    uint8_t  bRequest; /**< Request command code. */
    uint16_t wValue; /**< wValue parameter of the request. */
    uint16_t wIndex; /**< wIndex parameter of the request. */
    uint16_t wLength; /**< Length of the data to transfer in bytes. */
}
__attribute__ ((packed));

class USB
{
protected:
    USB_Request_Header_t USB_ControlRequest;
    volatile bool USB_IsInitialized;
    volatile uint8_t state;
    uint8_t USB_Device_ConfigurationNumber;
    bool USB_Device_CurrentlySelfPowered;
    bool USB_Device_RemoteWakeupEnabled;
    void selectEndpoint(uint8_t addr) { UENUM = addr & ENDPOINT_EPNUM_MASK; }
    void write8(uint8_t dat) { UEDATX = dat; }
    inline uint8_t read8() { return UEDATX; }
    void write16(uint16_t dat) { UEDATX = dat & 0xff; UEDATX = dat >> 8; }
    uint16_t Endpoint_BytesInEndpoint() { return ((uint16_t)UEBCHX<<8) | UEBCLX; }
    uint8_t Endpoint_GetCurrentEndpoint();
    uint8_t Endpoint_GetEndpointDirection();
    uint8_t Endpoint_WaitUntilReady();
    static constexpr uint8_t ENDPOINT_TOTAL_ENDPOINTS = 7;
    void Endpoint_ClearEndpoints();
public:
    static USB *instance;
    USB();
    virtual void gen() { }
    virtual void com() { }
};

#endif




