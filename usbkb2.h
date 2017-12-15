#ifndef _USBKB2_H_
#define _USBKB2_H_

#define F_CPU 16000000UL
#include "busby2.h"

#define CONTROL_REQTYPE_DIRECTION  0x80
#define CONTROL_REQTYPE_TYPE       0x60
#define CONTROL_REQTYPE_RECIPIENT  0x1F
#define REQDIR_HOSTTODEVICE        (0 << 7)
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_STANDARD           (0 << 5)
#define REQTYPE_CLASS              (1 << 5)
#define REQTYPE_VENDOR             (2 << 5)
#define REQREC_DEVICE              (0 << 0)
#define REQREC_INTERFACE           (1 << 0)
#define REQREC_ENDPOINT            (2 << 0)
#define REQREC_OTHER               (3 << 0)

enum USB_Control_Request_t
{
REQ_GetStatus           = 0,
REQ_ClearFeature        = 1,
REQ_SetFeature          = 3,
REQ_SetAddress          = 5,
REQ_GetDescriptor       = 6,
REQ_SetDescriptor       = 7,
REQ_GetConfiguration    = 8,
REQ_SetConfiguration    = 9,
REQ_GetInterface        = 10,
REQ_SetInterface        = 11,
REQ_SynchFrame          = 12,
};

enum USB_Feature_Selectors_t
{
FEATURE_SEL_EndpointHalt       = 0x00,
FEATURE_SEL_DeviceRemoteWakeup = 0x01,
FEATURE_SEL_TestMode           = 0x02,
};

struct USB_Endpoint_Table_t
{
    uint8_t  Address;
    uint16_t Size;
    uint8_t  Type;
    uint8_t  Banks;
};

struct USB_ClassInfo_HID_Device_t
{
    struct Config
    {
        uint8_t  InterfaceNumber;
        USB_Endpoint_Table_t ReportINEndpoint;
        void*    PrevReportINBuffer;
        uint8_t  PrevReportINBufferSize;
    };

    struct State
    {
        bool UsingReportProtocol;
        uint16_t PrevFrameNum;
        uint16_t IdleCount;
        uint16_t IdleMSRemaining;
    };
};


class USBKB : public USB
{
private:
    uint8_t Endpoint_Write_Stream_LE(const void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Stream_LE (void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length);
    uint8_t Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length);
    void USB_Device_ProcessControlRequest();
    void Endpoint_ClearStatusStage();
    void Device_ClearSetFeature();
    uint8_t Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length);
    void USB_Device_GetInternalSerialDescriptor();
    bool Endpoint_ConfigureEndpoint_Prv(uint8_t Number, uint8_t UECFG0XData, uint8_t UECFG1XData);
    uint16_t IdleMSRemaining = 0;
public:
    static USBKB *instance;
    void com();
    void gen();
    USBKB();
    void USBTask();
private:
    bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                     const uint8_t Entries); 

    uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint16_t wIndex,
                                    const void** const DescriptorAddress);

    static inline uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
    {
        uint8_t  MaskVal    = 0;
        uint16_t CheckBytes = 8;

        while (CheckBytes < Bytes)
        {
            MaskVal++;
            CheckBytes <<= 1;
        }

        return (MaskVal << EPSIZE0);
    }

    inline bool Endpoint_ConfigureEndpoint(uint8_t Address, uint8_t Type,
                                             const uint16_t Size,
                                             const uint8_t Banks)
    {
        uint8_t Number = (Address & ENDPOINT_EPNUM_MASK);

        if (Number >= ENDPOINT_TOTAL_ENDPOINTS)
            return false;

        return Endpoint_ConfigureEndpoint_Prv(Number,
          ((Type << EPTYPE0) | ((Address & ENDPOINT_DIR_IN) ? (1 << EPDIR) : 0)),
          ((1 << ALLOC) | ((Banks > 1) ? (1 << EPBK0) : 0) | Endpoint_BytesToEPSizeMask(Size)));
    }


};

#endif



