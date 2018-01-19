#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include "busby.h"
#include "usbsound.h"



#define STRING_ID_Language     0
#define STRING_ID_Manufacturer 1
#define STRING_ID_Product      2




static const DescDev PROGMEM DeviceDescriptor =
{
    sizeof(DescDev),
    DTYPE_Device,
    0x0110,
    0,
    0,
    0,
    FIXED_CONTROL_ENDPOINT_SIZE,
    0x03EB,
    0x2046,
    0x0002,
    STRING_ID_Manufacturer,
    STRING_ID_Product,
    0,
    FIXED_NUM_CONFIGURATIONS
};

static const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    {
        sizeof(DescConf),
        DTYPE_Configuration,
        sizeof(USB_Descriptor_Configuration_t),
        2,
        1,
        0,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_AudioControl,
        0,
        0,
        AUDIO_CSCP_AudioClass,
        AUDIO_CSCP_ControlSubclass,
        AUDIO_CSCP_ControlProtocol,
        0
    },
    {
        sizeof(DescAudioIface),
        DTYPE_CSInterface,
        AUDIO_DSUBTYPE_CSInterface_Header,
        0x0100,
        sizeof(DescAudioIface) + sizeof(DescAudioInTerminal) + sizeof(DescAudioOutTerminal),
        1,
        INTERFACE_ID_AudioStream,
    },
    {
        sizeof(DescAudioInTerminal),
        DTYPE_CSInterface,
        AUDIO_DSUBTYPE_CSInterface_InputTerminal,
        0x01,
        AUDIO_TERMINAL_STREAMING,
        0x00,
        2,
        AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT,
        0,
        0
    },
    {
        sizeof(DescAudioOutTerminal),
        DTYPE_CSInterface,
        AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
        0x02,
        AUDIO_TERMINAL_OUT_SPEAKER,
        0x00,
        0x01,
        0
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_AudioStream,
        0,
        0,
        AUDIO_CSCP_AudioClass,
        AUDIO_CSCP_AudioStreamingSubclass,
        AUDIO_CSCP_StreamingProtocol,
        0
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_AudioStream,
        1,
        1,
        AUDIO_CSCP_AudioClass,
        AUDIO_CSCP_AudioStreamingSubclass,
        AUDIO_CSCP_StreamingProtocol,
        0
    },
    {
        sizeof(DescAudioIfaceAS),
        DTYPE_CSInterface,
        AUDIO_DSUBTYPE_CSInterface_General,
        0x01,
        1,
        0x0001
    },
    {
        sizeof(AudioFormat) + sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates),
        DTYPE_CSInterface,
        AUDIO_DSUBTYPE_CSInterface_FormatType,
        0x01,
        0x02,
        0x02,
        16,
        sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates) / sizeof(SampleFreq),
    },
    {
        {0x40, 0x1f, 0x00}, // 8000
        {0x11, 0x2b, 0x00}, // 11025
        {0x22, 0x56, 0x00}, // 22050
        {0x44, 0xac, 0x00}, // 44100
        {0x80, 0xbb, 0x00}  // 48000
    },
    {
        {
            sizeof(AudioEndpoint),
            DTYPE_Endpoint,
            AUDIO_STREAM_EPADDR,
            EP_TIPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA,
            AUDIO_STREAM_EPSIZE,
            0x01
        },

        0,
        0
    },
    {
        sizeof(AudioEndpointSpc),
        DTYPE_CSEndpoint,
        AUDIO_DSUBTYPE_CSEndpoint_General,
        AUDIO_EP_ACCEPTS_SMALL_PACKETS | AUDIO_EP_SAMPLE_FREQ_CONTROL,
        0x00,
        0x0000
    }
};

class USBSound : public USB
{
public:
    bool StreamingAudioInterfaceSelected = false;
    uint32_t CurrentAudioSampleFrequency = 48000;
    void sampleCallback();
    void connect();
    USBSound();
    void procCtrlReq();
};

static USBSound *g_usbSound;

static constexpr size_t UZB_STRING_LEN(size_t uniChars) { return 2 + (uniChars << 1); }

static const DescString<2> PROGMEM LanguageString =
{
    UZB_STRING_LEN(1),
    DTYPE_String,
    (wchar_t)0x0409
};

static const DescString<12> PROGMEM ManufacturerString =
{
    UZB_STRING_LEN(11),
    DTYPE_String,
    L"Dean Camera"
};

static const DescString<20> PROGMEM ProductString =
{
    UZB_STRING_LEN(19),
    DTYPE_String,
    L"LUFA Audio Out Demo"
};

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
    const uint8_t  DescriptorType   = (wValue >> 8);
    const uint8_t  DescriptorNumber = (wValue & 0xFF);

    const void* Address = NULL;
    uint16_t    Size    = 0;

    switch (DescriptorType)
    {
        case DTYPE_Device:
            Address = &DeviceDescriptor;
            Size = sizeof(DescDev);
            break;
        case DTYPE_Configuration:
            Address = &ConfigurationDescriptor;
            Size    = sizeof(USB_Descriptor_Configuration_t);
            break;
        case DTYPE_String:
            switch (DescriptorNumber)
            {
                case STRING_ID_Language:
                    Address = &LanguageString;
                    Size    = pgm_read_byte(&LanguageString.size);
                    break;
                case STRING_ID_Manufacturer:
                    Address = &ManufacturerString;
                    Size    = pgm_read_byte(&ManufacturerString.size);
                    break;
                case STRING_ID_Product:
                    Address = &ProductString;
                    Size    = pgm_read_byte(&ProductString.size);
                    break;
            }

            break;
    }

    *DescriptorAddress = Address;
    return Size;
}



void USBSound::connect(void)
{
    *p_ddrc |= 1<<7;
    *p_ddrd |= 1<<0 | 1<<5;
    *p_portc |= 1<<7;
    *p_portd |= 1<<0 | 1<<5;
	*p_timsk0 = 1<<ocie0a;
	*p_ocr0a = (F_CPU >> 3) / CurrentAudioSampleFrequency - 1;
	*p_tccr0a = 1<<wgm01;  // CTC mode
	*p_tccr0b = 1<<cs01;   // Fcpu/8 speed
	*p_ddrc |= 1<<6 | 1<<5;
	*p_tccr3a = (1<<WGM30 | (1 << COM3A1) | 1<<COM3A0 | 1<<COM3B1 | (1<<COM3B0));
	*p_tccr3b = 1<<wgm32 | 1<<cs30;  // Fast 8-Bit PWM, F_CPU speed
}

static inline void Endpoynt_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOINT_EPNUM_MASK));
    UERST = 0;
}

#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");

static inline uint8_t GetGlobalInterruptMask(void)
{
    GCC_MEMORY_BARRIER();
    return SREG;
}

static inline void SetGlobalInterruptMask(const uint8_t GlobalIntState)
{
    GCC_MEMORY_BARRIER();
    SREG = GlobalIntState;
    GCC_MEMORY_BARRIER();
}

static inline void UZB_Device_GetSerialString(uint16_t* const UnicodeString)
{
    uint8_t CurrentGlobalInt = GetGlobalInterruptMask();
    cli();
    uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0; SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4); SerialCharNum++)
    {
        uint8_t SerialByte = boot_signature_byte_get(SigReadAddress);

        if (SerialCharNum & 0x01)
        {
            SerialByte >>= 4;
            SigReadAddress++;
        }

        SerialByte &= 0x0F;

        UnicodeString[SerialCharNum] = ((SerialByte >= 10) ? (('A' - 10) + SerialByte) : ('0' + SerialByte));
    }

    SetGlobalInterruptMask(CurrentGlobalInt);
}

void USBSound::sampleCallback()
{
    uint8_t PrevEndpoint = getCurrentEndpoint();
    selectEndpoint(AUDIO_STREAM_EPADDR);

    if (UEINTX & 1<<RXOUTI && StreamingAudioInterfaceSelected)
    {
        int8_t left   = ((int16_t)read16le() >> 8);
        int8_t right  = ((int16_t)read16le() >> 8);

        if ((UEINTX & 1<<RWAL) == 0)
            UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);

        *p_ocr3a = left  ^ 1<<7;
        *p_ocr3b = right ^ 1<<7;
    }

    selectEndpoint(PrevEndpoint);
}

void USBSound::procCtrlReq()
{
    uint8_t* RequestHeader = (uint8_t*)&_ctrlReq;

    for (uint8_t i = 0; i < sizeof(USBRequest); i++)
        *RequestHeader++ = read8();

    uint8_t bmRequestType = _ctrlReq.bmRequestType;

    switch (_ctrlReq.bRequest)
    {
    case REQ_SetInterface:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_INTERFACE))
        {
            *p_ueintx &= ~(1<<rxstpi);
            clearStatusStage();
            StreamingAudioInterfaceSelected = ((_ctrlReq.wValue) != 0);
        }
        break;
    case AUDIO_REQ_GetStatus:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE) ||
		    bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            *p_ueintx &= ~(1<<rxstpi);
            clearStatusStage();
        }
        break;
    case AUDIO_REQ_SetCurrent:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            uint8_t EndpointAddress = (uint8_t)_ctrlReq.wIndex;
            uint8_t EndpointControl = _ctrlReq.wValue >> 8;

            if ((EndpointAddress == AUDIO_STREAM_EPADDR) &&
                    (EndpointControl == AUDIO_EPCONTROL_SamplingFreq))
            {
                uint8_t SampleRate[3];
                UEINTX &= ~(1<<RXSTPI);
                readControlStreamLE(SampleRate, sizeof(SampleRate));
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

                CurrentAudioSampleFrequency = (((uint32_t)SampleRate[2] << 16) |
                        ((uint32_t)SampleRate[1] << 8) | (uint32_t)SampleRate[0]);

                OCR0A = (F_CPU >> 3) / CurrentAudioSampleFrequency - 1;
            }
        }
        break;
    case AUDIO_REQ_GetCurrent:
        if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            uint8_t EndpointAddress = (uint8_t)_ctrlReq.wIndex;
            uint8_t EndpointControl = (_ctrlReq.wValue >> 8);

            if ((EndpointAddress == AUDIO_STREAM_EPADDR) &&
                    (EndpointControl == AUDIO_EPCONTROL_SamplingFreq))
            {
                uint8_t SampleRate[3];
                SampleRate[2] = (CurrentAudioSampleFrequency >> 16);
                SampleRate[1] = (CurrentAudioSampleFrequency >> 8);
                SampleRate[0] = (CurrentAudioSampleFrequency &  0xFF);
                UEINTX &= ~(1<<RXSTPI);
                write_Control_Stream_LE(SampleRate, sizeof(SampleRate));
                UEINTX &= ~((1 << RXOUTI) | (1 << FIFOCON));
            }
        }

        break;
	}

    if (UEINTX & 1<<RXSTPI)
    {
        switch (_ctrlReq.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                uint8_t CurrentStatus = 0;
            
                switch (_ctrlReq.bmRequestType)
                {
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
                {
                    if (USB_Device_RemoteWakeupEnabled)
                        CurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;
        
                    break;
                }
                case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
                {
                    uint8_t EndpointIndex = ((uint8_t)_ctrlReq.wIndex & ENDPOINT_EPNUM_MASK);
            
                    if (EndpointIndex >= ENDPOINT_TOTAL_ENDPOINTS)
                        return;
            
                    selectEndpoint(EndpointIndex);
                    CurrentStatus = ((UECONX & (1 << STALLRQ)) ? true : false);
                    selectEndpoint(0);
                    break;
                }
                default:
                    return;
                }
            
                UEINTX &= ~(1<<RXSTPI);
                write16le(CurrentStatus);
                UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
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
                setDevAddr(DeviceAddress);
                UEINTX &= ~(1<<RXSTPI);
                clearStatusStage();

                while ((UEINTX & 1<<TXINI) == 0)
                    ;

                *p_udaddr |= 1<<adden;
                state = DeviceAddress ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                const void* DescriptorPointer;
                uint16_t    DescriptorSize;
            
                if ((DescriptorSize = CALLBACK_USB_GetDescriptor(_ctrlReq.wValue,
                        _ctrlReq.wIndex, &DescriptorPointer)) == 0)
                {
                    return;
                }
            
                UEINTX &= ~(1<<RXSTPI);
                write_Control_PStream_LE(DescriptorPointer, DescriptorSize);
                UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                *p_ueintx &= ~(1<<rxstpi);     // clear setup
                write8(USB_Device_ConfigurationNumber);
                *p_ueintx &= ~(1<<txini | 1<<fifocon); // clear in
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
            
                configureEndpoint(AUDIO_STREAM_EPADDR, EP_TIPE_ISOCHRONOUS, AUDIO_STREAM_EPSIZE, 2);
            }
            break;

        default:
            break;
        }
    }

    if (*p_ueintx & 1<<rxstpi)      // setup received?
    {
        *p_ueintx &= ~(1<<rxstpi);  // clear setup
        *p_ueconx |= 1<<stallrq;    // stall transaction
    }
}

USBSound::USBSound()
{
    *p_usbcon &= ~(1<<otgpade);
    *p_uhwcon |= 1<<uvrege;
    *p_pllfrq |= 1<<pdiv2;
    *p_usbcon &= ~(1<<vbuste);
    *p_udien = 0;
    *p_usbint = 0;
    *p_udint = 0;
    *p_usbcon &= ~(1<<usbe);
    *p_usbcon |= 1<<usbe;
    *p_usbcon &= ~(1<<frzclk);
    *p_pllcsr = 0;
    state = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    *p_udcon &= ~(1<<lsm);
    *p_usbcon |= 1<<vbuste;
    configureEndpoint(0, 0, 8, 1);
    *p_udint &= ~(1<<suspi);
    *p_udien |= 1<<suspe;
    *p_udien |= 1<<eorste;
    *p_udcon &= ~(1<<detach);
    *p_usbcon |= 1<<otgpade;
}

ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
    g_usbSound->sampleCallback();
}

int main(void)
{
    USBSound usbSound;
    g_usbSound = &usbSound;
    sei();

	for (;;)
	{
	}
}


