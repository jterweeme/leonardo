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

static uint8_t USB_DeviceState = 0;
#define INTERNAL_SERIAL_LENGTH_BITS 80
#define INTERNAL_SERIAL_START_ADDRESS 0x0e

#ifndef ATTR_PACKED
#define ATTR_PACKED __attribute__ ((packed))
#endif

#define FIXED_CONTROL_ENDPOINT_SIZE      8
#define FIXED_NUM_CONFIGURATIONS         1

#define UZB_CONFIG_ATTR_RESERVED          0x80
#define UZB_CONFIG_ATTR_SELFPOWERED       0x40

#define UZB_CONFIG_POWER_MA(mA)           ((mA) >> 1)

#define FEATURE_SEL_EndpointHalt 0
#define FEATURE_SEL_DeviceRemoteWakeup 1
#define FEATURE_SEL_TestMode 2

#define VEATURE_SELFPOWERED_ENABLED 1<<0
#define VEATURE_REMOTE_WAKEUP_ENABLED 1<<1

#define ENDPOYNT_TOTAL_ENDPOINTS 7
#define ENDPOYNT_DIR_OUT 0x00
#define ENDPOYNT_DIR_IN 0x80
#define ENDPOYNT_EPNUM_MASK 0x0f

#define AUDYO_STREAM_EPADDR          (ENDPOYNT_DIR_OUT | 1)
#define AUDYO_STREAM_EPSIZE           256

#define ENDPOYNT_ATTR_SYNC                (3 << 2)
#define ENDPOYNT_USAGE_DATA               (0 << 4)

#define EP_TIPE_ISOCHRONOUS 0x01

#define AUDIO_OUT_STEREO
#define AUDIO_STREAM_EPADDR          (ENDPOYNT_DIR_OUT | 1)
#define AUDIO_STREAM_EPSIZE           256

#define AUDYO_CHANNEL_LEFT_FRONT           (1 << 0)
#define AUDYO_CHANNEL_RIGHT_FRONT          (1 << 1)
#define AUDYO_EP_SAMPLE_FREQ_CONTROL      (1 << 0)
#define AUDYO_EP_PITCH_CONTROL            (1 << 1)
#define AUDYO_EP_FULL_PACKETS_ONLY        (1 << 7)
#define AUDYO_EP_ACCEPTS_SMALL_PACKETS    (0 << 7)

#define AUDYO_DSUBTYPE_CSInterface_General        0x01
#define AUDYO_DSUBTYPE_CSInterface_FormatType     0x02
#define AUDYO_DSUBTYPE_CSInterface_FormatSpecific 0x03
#define AUDYO_DSUBTYPE_CSEndpoint_General         0x01

#define AUDYO_TERMINAL_UNDEFINED           0x0100
#define AUDYO_TERMINAL_STREAMING           0x0101
#define AUDYO_TERMINAL_VENDOR              0x01FF
#define AUDYO_TERMINAL_IN_UNDEFINED        0x0200
#define AUDYO_TERMINAL_OUT_SPEAKER         0x0301
#define AUDYO_TERMINAL_OUT_HEADPHONES      0x0302
#define AUDYO_TERMINAL_OUT_HEAD_MOUNTED    0x0303

#define AUDYO_EPCONTROL_SamplingFreq 0x01
#define AUDYO_EPCONTROL_Pitch        0x02

#define AUDYO_REQ_SetCurrent    0x01
#define AUDYO_REQ_SetMinimum    0x02
#define AUDYO_REQ_SetMaximum    0x03
#define AUDYO_REQ_SetResolution 0x04
#define AUDYO_REQ_SetMemory     0x05
#define AUDYO_REQ_GetCurrent    0x81
#define AUDYO_REQ_GetMinimum    0x82
#define AUDYO_REQ_GetMaximum    0x83
#define AUDYO_REQ_GetResolution 0x84
#define AUDYO_REQ_GetMemory     0x85
#define AUDYO_REQ_GetStatus     0xFF

#define KONTROL_REQTYPE_DIRECTION  0x80
#define KONTROL_REQTYPE_TYPE       0x60
#define KONTROL_REQTYPE_RECIPIENT  0x1F
#define DEVIZE_STATE_Unattached 0
#define DEVIZE_STATE_Powered    1
#define DEVIZE_STATE_Default    2
#define DEVIZE_STATE_Addressed  3
#define DEVIZE_STATE_Configured 4
#define DEVIZE_STATE_Suspended  5

#define ENDPOYNT_RWSTREAM_NoError            0
#define ENDPOYNT_RWSTREAM_EndpointStalled    1
#define ENDPOYNT_RWSTREAM_DeviceDisconnected 2
#define ENDPOYNT_RWSTREAM_BusSuspended       3
#define ENDPOYNT_RWSTREAM_Timeout            4
#define ENDPOYNT_RWSTREAM_IncompleteTransfer 5

#define ENDPOYNT_RWCSTREAM_NoError            0
#define ENDPOYNT_RWCSTREAM_HostAborted        1
#define ENDPOYNT_RWCSTREAM_DeviceDisconnected 2
#define ENDPOYNT_RWCSTREAM_BusSuspended       3

#define AUDYO_CSCP_AudioClass                     0x01
#define AUDYO_CSCP_ControlSubclass                0x01
#define AUDYO_CSCP_ControlProtocol                0x00
#define AUDYO_CSCP_AudioStreamingSubclass         0x02
#define AUDYO_CSCP_MIDIStreamingSubclass          0x03
#define AUDYO_CSCP_StreamingProtocol              0x00

static USBRequest USB_ControlRequest;

#define USB_Device_ControlEndpointSize 8
static uint8_t USB_Device_ConfigurationNumber = 0;

static bool USB_Device_RemoteWakeupEnabled;
static bool USB_Device_CurrentlySelfPowered;

#define    AUDYO_DSUBTYPE_CSInterface_Header         0x01
#define    AUDYO_DSUBTYPE_CSInterface_InputTerminal  0x02
#define    AUDYO_DSUBTYPE_CSInterface_OutputTerminal 0x03
#define    AUDYO_DSUBTYPE_CSInterface_Mixer          0x04
#define    AUDYO_DSUBTYPE_CSInterface_Selector       0x05
#define    AUDYO_DSUBTYPE_CSInterface_Feature        0x06
#define    AUDYO_DSUBTYPE_CSInterface_Processing     0x07
#define    AUDYO_DSUBTYPE_CSInterface_Extension      0x08

typedef struct
{
    uint8_t Size;
    uint8_t Type;
} ATTR_PACKED UZB_Descriptor_Header_t;

typedef struct
{
    uint8_t size;
    uint8_t type;
    uint8_t  EndpointAddress;
    uint8_t  Attributes;
    uint16_t EndpointSize;
    uint8_t  PollingIntervalMS;
} ATTR_PACKED UZB_Descriptor_Endpoint_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 TerminalID;
    uint16_t                TerminalType;
    uint8_t                 AssociatedOutputTerminal;
    uint8_t                 TotalChannels;
    uint16_t                ChannelConfig;
    uint8_t                 ChannelStrIndex;
    uint8_t                 TerminalStrIndex;
} ATTR_PACKED UZB_Audio_Descriptor_InputTerminal_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 TerminalID;
    uint16_t                TerminalType;
    uint8_t                 AssociatedInputTerminal;
    uint8_t                 SourceID;
    uint8_t                 TerminalStrIndex;
} ATTR_PACKED UZB_Audio_Descriptor_OutputTerminal_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint16_t                ACSpecification;
    uint16_t                TotalLength;
    uint8_t                 InCollection;
    uint8_t                 InterfaceNumber;
} ATTR_PACKED UZB_Audio_Descriptor_Interface_AC_t;

struct SampleFreq
{
    uint8_t Byte1;
    uint8_t Byte2;
    uint8_t Byte3;
} ATTR_PACKED;

typedef struct
{
    UZB_Descriptor_Endpoint_t Endpoint;
    uint8_t                   Refresh;
    uint8_t                   SyncEndpointNumber;
} ATTR_PACKED UZB_Audio_Descriptor_StreamEndpoint_Std_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint8_t                 Subtype;
    uint8_t                 Attributes;
    uint8_t                 LockDelayUnits;
    uint16_t                LockDelay;
} ATTR_PACKED UZB_Audio_Descriptor_StreamEndpoint_Spc_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint16_t TotalConfigurationSize;
    uint8_t  TotalInterfaces;
    uint8_t  ConfigurationNumber;
    uint8_t  ConfigurationStrIndex;
    uint8_t  ConfigAttributes;
    uint8_t  MaxPowerConsumption;
} ATTR_PACKED UZB_Descriptor_Configuration_Header_t;

typedef struct
{
    UZB_Descriptor_Header_t Header;
    uint8_t Subtype;
    uint8_t TerminalLink;
    uint8_t FrameDelay;
    uint16_t AudioFormat;
} ATTR_PACKED UZB_Audio_Descriptor_Interface_AS_t;

template <size_t S> struct UZB_Descriptor_String_t
{
    uint8_t size;
    uint8_t type;
    wchar_t  UnicodeString[S];
} ATTR_PACKED;

struct AudioFormat
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint8_t FormatType;
    uint8_t Channels;
    uint8_t SubFrameSize;
    uint8_t BitResolution;
    uint8_t TotalDiscreteSampleRates;
} ATTR_PACKED;

typedef struct
{
    UZB_Descriptor_Configuration_Header_t     Config;
    DescIface                Audio_ControlInterface;
    UZB_Audio_Descriptor_Interface_AC_t       Audio_ControlInterface_SPC;
    UZB_Audio_Descriptor_InputTerminal_t      Audio_InputTerminal;
    UZB_Audio_Descriptor_OutputTerminal_t     Audio_OutputTerminal;
    DescIface                Audio_StreamInterface_Alt0;
    DescIface                Audio_StreamInterface_Alt1;
    UZB_Audio_Descriptor_Interface_AS_t       Audio_StreamInterface_SPC;
    AudioFormat Audio_AudioFormat;
    SampleFreq                    Audio_AudioFormatSampleRates[5];
    UZB_Audio_Descriptor_StreamEndpoint_Std_t Audio_StreamEndpoint;
    UZB_Audio_Descriptor_StreamEndpoint_Spc_t Audio_StreamEndpoint_SPC;
} USB_Descriptor_Configuration_t;

#define INTERFACE_ID_AudioControl 0
#define INTERFACE_ID_AudioStream  1

#define STRING_ID_Language     0
#define STRING_ID_Manufacturer 1
#define STRING_ID_Product      2

#define LEDS_LED1        (1 << 0)

#define LEDS_LED2        (1 << 5)

#define LEDS_LED3        (1 << 7)

#define LEDS_ALL_LEDS    (LEDS_LED1 | LEDS_LED2 | LEDS_LED3)

#define LEDS_NO_LEDS     0


#define LEDS_PORTB_LEDS       (LEDS_LED1)
#define LEDS_PORTD_LEDS       (LEDS_LED2)
#define LEDS_PORTC_LEDS       (LEDS_LED3)

static inline void LEDs_Init(void)
{
    DDRB  |=  LEDS_PORTB_LEDS;
    PORTB |=  LEDS_PORTB_LEDS;
    DDRD  |=  LEDS_PORTD_LEDS;
    PORTD |=  LEDS_PORTD_LEDS;
    DDRC  |=  LEDS_PORTC_LEDS;
    PORTC &= ~LEDS_PORTC_LEDS;
}

static inline void LEDs_SetAllLEDs(const uint8_t LEDMask)
{
    PORTB = ((PORTB |  LEDS_PORTB_LEDS) & ~(LEDMask & LEDS_PORTB_LEDS));
    PORTD = ((PORTD |  LEDS_PORTD_LEDS) & ~(LEDMask & LEDS_PORTD_LEDS));
    PORTC = ((PORTC & ~LEDS_PORTC_LEDS) |  (LEDMask & LEDS_PORTC_LEDS));
}

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress);

#if !defined(LEDS_NO_LEDS)
    #define LEDS_NO_LEDS   0
#endif

#if !defined(LEDS_ALL_LEDS)
    #define LEDS_ALL_LEDS  (LEDS_LED1 | LEDS_LED2 | LEDS_LED3 | LEDS_LED4)
#endif

#if !defined(LEDS_LED1)
    #define LEDS_LED1      0
#endif

#if !defined(LEDS_LED2)
    #define LEDS_LED2      0
#endif

#if !defined(LEDS_LED3)
    #define LEDS_LED3      0
#endif

#if !defined(LEDS_LED4)
    #define LEDS_LED4      0
#endif

#define LEDMASK_USB_NOTREADY      LEDS_LED1

#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

static bool StreamingAudioInterfaceSelected = false;
static uint32_t CurrentAudioSampleFrequency = 48000;

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
        {
            sizeof(UZB_Descriptor_Configuration_Header_t),
            DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        2,
        1,
        0,
        UZB_CONFIG_ATTR_RESERVED | UZB_CONFIG_ATTR_SELFPOWERED,
        UZB_CONFIG_POWER_MA(100)
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_AudioControl,
        0,
        0,
        AUDYO_CSCP_AudioClass,
        AUDYO_CSCP_ControlSubclass,
        AUDYO_CSCP_ControlProtocol,
        0
    },
    {
        {
            sizeof(UZB_Audio_Descriptor_Interface_AC_t),
            DTYPE_CSInterface
        },
        AUDYO_DSUBTYPE_CSInterface_Header,
        0x0100,
        sizeof(UZB_Audio_Descriptor_Interface_AC_t) +
                                     sizeof(UZB_Audio_Descriptor_InputTerminal_t) +
                                     sizeof(UZB_Audio_Descriptor_OutputTerminal_t),

        1,
        INTERFACE_ID_AudioStream,
    },
    {
        {
            sizeof(UZB_Audio_Descriptor_InputTerminal_t),
            DTYPE_CSInterface
        },
        AUDYO_DSUBTYPE_CSInterface_InputTerminal,
        0x01,
        AUDYO_TERMINAL_STREAMING,
        0x00,
        2,
        AUDYO_CHANNEL_LEFT_FRONT | AUDYO_CHANNEL_RIGHT_FRONT,
        0,
        0
    },
    {
        {
            sizeof(UZB_Audio_Descriptor_OutputTerminal_t),
            DTYPE_CSInterface
        },
        AUDYO_DSUBTYPE_CSInterface_OutputTerminal,
        0x02,
        AUDYO_TERMINAL_OUT_SPEAKER,
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
        AUDYO_CSCP_AudioClass,
        AUDYO_CSCP_AudioStreamingSubclass,
        AUDYO_CSCP_StreamingProtocol,
        0
    },
    {
        sizeof(DescIface),
        DTYPE_Interface,
        INTERFACE_ID_AudioStream,
        1,
        1,
        AUDYO_CSCP_AudioClass,
        AUDYO_CSCP_AudioStreamingSubclass,
        AUDYO_CSCP_StreamingProtocol,
        0
    },
    {
        {
            sizeof(UZB_Audio_Descriptor_Interface_AS_t),
            DTYPE_CSInterface
        },
        AUDYO_DSUBTYPE_CSInterface_General,
        0x01,
        1,
        0x0001
    },
    {
        sizeof(AudioFormat) + sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates),
        DTYPE_CSInterface,
        AUDYO_DSUBTYPE_CSInterface_FormatType,
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
            sizeof(UZB_Audio_Descriptor_StreamEndpoint_Std_t),
            DTYPE_Endpoint,
            AUDIO_STREAM_EPADDR,
            EP_TIPE_ISOCHRONOUS | ENDPOYNT_ATTR_SYNC | ENDPOYNT_USAGE_DATA,
            AUDIO_STREAM_EPSIZE,
            0x01
        },

        0,
        0
    },
    {
        {
            sizeof(UZB_Audio_Descriptor_StreamEndpoint_Spc_t),
            DTYPE_CSEndpoint
        },
        AUDYO_DSUBTYPE_CSEndpoint_General,
        AUDYO_EP_ACCEPTS_SMALL_PACKETS | AUDYO_EP_SAMPLE_FREQ_CONTROL,
        0x00,
        0x0000
    }
};

static constexpr size_t UZB_STRING_LEN(size_t uniChars) { return 2 + (uniChars << 1); }

static const UZB_Descriptor_String_t<2> PROGMEM LanguageString =
{
    UZB_STRING_LEN(1),
    DTYPE_String,
    (wchar_t)0x0409
};

static const UZB_Descriptor_String_t<12> PROGMEM ManufacturerString =
{
    UZB_STRING_LEN(11),
    DTYPE_String,
    L"Dean Camera"
};

static const UZB_Descriptor_String_t<20> PROGMEM ProductString =
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



void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
	TIMSK0 = (1 << OCIE0A);
	OCR0A  = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
	TCCR0A = (1 << WGM01);  // CTC mode
	TCCR0B = (1 << CS01);   // Fcpu/8 speed

#if defined(AUDIO_OUT_MONO)
	DDRC  |= (1 << 6);
#elif defined(AUDIO_OUT_STEREO)
	DDRC  |= ((1 << 6) | (1 << 5));
#elif defined(AUDIO_OUT_PORTC)
	DDRC  |= 0xFF;
#endif

#if (defined(AUDIO_OUT_MONO) || defined(AUDIO_OUT_STEREO))
	TCCR3A = ((1<<WGM30) | (1 << COM3A1) | 1<<COM3A0 | 1<<COM3B1 | (1<<COM3B0));
	TCCR3B = ((1<<WGM32) | (1<<CS30));  // Fast 8-Bit PWM, F_CPU speed
#endif
}

void EVENT_USB_Device_Disconnect(void)
{
	TCCR0B = 0;
#if (defined(AUDIO_OUT_MONO) || defined(AUDIO_OUT_STEREO))
	TCCR3B = 0;
#endif

#if defined(AUDIO_OUT_MONO)
	DDRC  &= ~(1 << 6);
#elif defined(AUDIO_OUT_STEREO)
	DDRC  &= ~((1 << 6) | (1 << 5));
#elif defined(AUDIO_OUT_PORTC)
	PORTC = 0x00;
#endif

	StreamingAudioInterfaceSelected = false;
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

static inline void Endpoynt_ClearSETUP(void)
{
    UEINTX &= ~(1 << RXSTPI);
}

static inline void Endpoynt_ClearIN(void)
{
    UEINTX &= ~((1 << TXINI) | (1 << FIFOCON));
}

static inline bool Endpoynt_IsConfigured(void)
{
    return ((UESTA0X & (1 << CFGOK)) ? true : false);
}

static inline bool Endpoynt_IsReadWriteAllowed(void)
{
    return ((UEINTX & (1 << RWAL)) ? true : false);
}

static inline uint8_t Endpoynt_Read_8(void)
{
    return UEDATX;
}

static inline uint8_t Endpoynt_GetEndpointInterrupts(void)
{
    return UEINT;
}

static inline bool Endpoynt_IsINReady(void)
{
    return ((UEINTX & (1 << TXINI)) ? true : false);
}

static inline void Endpoynt_ClearOUT(void)
{
    UEINTX &= ~((1 << RXOUTI) | (1 << FIFOCON));
}

static inline void Endpoynt_StallTransaction(void)
{
    UECONX |= 1<<STALLRQ;
}

static inline bool Endpoynt_IsOUTReceived(void)
{
    return ((UEINTX & (1 << RXOUTI)) ? true : false);
}

static inline bool Endpoynt_IsSETUPReceived(void)
{
    return ((UEINTX & (1 << RXSTPI)) ? true : false);
}

static inline void Endpoynt_SelectEndpoint(const uint8_t Address)
{
    UENUM = (Address & ENDPOYNT_EPNUM_MASK);
}

static inline void Endpoynt_DisableEndpoint(void)
{
    UECONX &= ~(1 << EPEN);
}

static inline void Endpoynt_EnableEndpoint(void)
{
    UECONX |= (1 << EPEN);
}

static inline void Endpoynt_Write_8(const uint8_t Data)
{
    UEDATX = Data;
}

static inline uint8_t Endpoynt_GetEndpointDirection(void)
{
    return (UECFG0X & (1 << EPDIR)) ? ENDPOYNT_DIR_IN : ENDPOYNT_DIR_OUT;
}

static inline uint16_t Endpoynt_BytesInEndpoint(void)
{
    return (((uint16_t)UEBCHX << 8) | UEBCLX);
}

static inline uint8_t Endpoynt_GetCurrentEndpoint(void)
{
    return ((UENUM & ENDPOYNT_EPNUM_MASK) | Endpoynt_GetEndpointDirection());
}

static inline void UZB_Device_SetDeviceAddress(const uint8_t Address)
{
    UDADDR = (UDADDR & (1 << ADDEN)) | (Address & 0x7F);
}

static inline void UZB_Device_EnableDeviceAddress(const uint8_t Address)
{
    (void)Address;

    UDADDR |= (1 << ADDEN);
}

static inline uint16_t Endpoynt_Read_16_LE(void)
{
    union
    {
        uint16_t Value;
        uint8_t  Bytes[2];
    } Data;

    Data.Bytes[0] = UEDATX;
    Data.Bytes[1] = UEDATX;

    return Data.Value;
}

static uint8_t Endpoynt_Read_Control_Stream_LE(void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = ((uint8_t*)Buffer);

    if (!Length)
        Endpoynt_ClearOUT();

    while (Length)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;

        if (Endpoynt_IsSETUPReceived())
            return ENDPOYNT_RWCSTREAM_HostAborted;

        if (Endpoynt_IsOUTReceived())
        {
            while (Length && Endpoynt_BytesInEndpoint())
            {
                *DataStream = Endpoynt_Read_8();
                DataStream += 1;
                Length--;
            }

            Endpoynt_ClearOUT();
        }
    }

    while ((Endpoynt_IsINReady()) == 0)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;
    }

    return ENDPOYNT_RWCSTREAM_NoError;
}

static uint8_t Endpoynt_Write_Control_Stream_LE(const void * const buf, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)buf);
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        Endpoynt_ClearIN();

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;

        if (Endpoynt_IsSETUPReceived())
            return ENDPOYNT_RWCSTREAM_HostAborted;

        if (Endpoynt_IsOUTReceived())
            break;

        if (*p_ueintx & 1<<txini)
        {
            uint16_t bytesInEp = Endpoynt_BytesInEndpoint();

            while (Length && bytesInEp < USB_Device_ControlEndpointSize)
            {
                Endpoynt_Write_8(*DataStream);
                DataStream++;
                Length--;
                bytesInEp++;
            }

            LastPacketFull = bytesInEp == USB_Device_ControlEndpointSize;
            *p_ueintx &= ~(1<<txini | 1<<fifocon);  // endpoint clear in
        }
    }

    while ((Endpoynt_IsOUTReceived()) == 0)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;

        if (Endpoynt_IsSETUPReceived())
            return ENDPOYNT_RWCSTREAM_HostAborted;
    }

    return ENDPOYNT_RWCSTREAM_NoError;
}

static bool Endpoynt_ConfigureEndpoint_Prv(const uint8_t Number,
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

        Endpoynt_DisableEndpoint();
        UECFG1X &= ~(1<<ALLOC);
        Endpoynt_EnableEndpoint();
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

        if ((Endpoynt_IsConfigured()) == 0)
          return false;
    }

    Endpoynt_SelectEndpoint(Number);
    return true;
}

static inline uint8_t Endpoynt_BytesToEPSizeMask(const uint16_t Bytes)
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


static inline bool Endpoynt_ConfigureEndpoint(const uint8_t Address,
                                              const uint8_t Type,
                                              const uint16_t Size,
                                              const uint8_t Banks)
{
    uint8_t Number = (Address & ENDPOYNT_EPNUM_MASK);

    if (Number >= ENDPOYNT_TOTAL_ENDPOINTS)
        return false;

    return Endpoynt_ConfigureEndpoint_Prv(Number,
                   ((Type << EPTYPE0) | ((Address & ENDPOYNT_DIR_IN) ? (1 << EPDIR) : 0)),
         ((1 << ALLOC) | ((Banks > 1) ? (1 << EPBK0) : 0) | Endpoynt_BytesToEPSizeMask(Size)));
}

static void Endpoynt_ClearStatusStage(void)
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while ((Endpoynt_IsOUTReceived()) == 0)
        {
            if (USB_DeviceState == DEVIZE_STATE_Unattached)
              return;
        }

        Endpoynt_ClearOUT();
    }
    else
    {
        while ((Endpoynt_IsINReady()) == 0)
        {
            if (USB_DeviceState == DEVIZE_STATE_Unattached)
              return;
        }

        Endpoynt_ClearIN();
    }
}

static void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= Endpoynt_ConfigureEndpoint(AUDIO_STREAM_EPADDR,
        EP_TIPE_ISOCHRONOUS, AUDIO_STREAM_EPSIZE, 2);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

static void UZB_Device_GetConfiguration(void)
{
    Endpoynt_ClearSETUP();
    Endpoynt_Write_8(USB_Device_ConfigurationNumber);
    Endpoynt_ClearIN();
    Endpoynt_ClearStatusStage();
}

static inline void Endpoynt_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOYNT_EPNUM_MASK));
    UERST = 0;
}

static inline bool Endpoynt_IsEnabled(void)
{
    return ((UECONX & (1 << EPEN)) ? true : false);
}

static inline bool UZB_Device_IsAddressSet(void)
{
    return UDADDR & 1<<ADDEN;
}

static inline void Endpoynt_ResetDataToggle(void)
{
    UECONX |= (1 << RSTDT);
}

static inline void Endpoynt_ClearStall(void)
{
    UECONX |= (1 << STALLRQC);
}

static inline bool Endpoynt_IsStalled(void)
{
    return ((UECONX & (1 << STALLRQ)) ? true : false);
}

static inline void Endpoynt_Write_16_LE(const uint16_t Data)
{
    UEDATX = (Data & 0xFF);
    UEDATX = (Data >> 8);
}


static void UZB_Device_ClearSetFeature(void)
{
    switch (USB_ControlRequest.bmRequestType & KONTROL_REQTYPE_RECIPIENT)
    {
        case REQREC_DEVICE:
        {
            if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_DeviceRemoteWakeup)
              USB_Device_RemoteWakeupEnabled = (USB_ControlRequest.bRequest == REQ_SetFeature);
            else
              return;
            
            break;
        }
        case REQREC_ENDPOINT:
        {
            if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_EndpointHalt)
            {
                uint8_t EndpointIndex = ((uint8_t)USB_ControlRequest.wIndex & ENDPOYNT_EPNUM_MASK);
 
                if (EndpointIndex == 0 || EndpointIndex >= ENDPOYNT_TOTAL_ENDPOINTS)
                  return;

                Endpoynt_SelectEndpoint(EndpointIndex);

                if (Endpoynt_IsEnabled())
                {
                    if (USB_ControlRequest.bRequest == REQ_SetFeature)
                    {   
                        Endpoynt_StallTransaction();
                    }
                    else
                    {
                        Endpoynt_ClearStall();
                        Endpoynt_ResetEndpoint(EndpointIndex);
                        Endpoynt_ResetDataToggle();
                    }
                }
            }
            
            break;
        }
        default:
            return;
    }

    Endpoynt_SelectEndpoint(0);
    Endpoynt_ClearSETUP();
    Endpoynt_ClearStatusStage();
}

static void UZB_Device_GetStatus(void)
{
    uint8_t CurrentStatus = 0;

    switch (USB_ControlRequest.bmRequestType)
    {
        case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE):
        {
            if (USB_Device_CurrentlySelfPowered)
                CurrentStatus |= VEATURE_SELFPOWERED_ENABLED;

            if (USB_Device_RemoteWakeupEnabled)
                CurrentStatus |= VEATURE_REMOTE_WAKEUP_ENABLED;

            break;
        }
        case (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT):
        {
            uint8_t EndpointIndex = ((uint8_t)USB_ControlRequest.wIndex & ENDPOYNT_EPNUM_MASK);

            if (EndpointIndex >= ENDPOYNT_TOTAL_ENDPOINTS)
                return;

            Endpoynt_SelectEndpoint(EndpointIndex);
            CurrentStatus = Endpoynt_IsStalled();
            Endpoynt_SelectEndpoint(0);
            break;
        }
        default:
            return;
    }

    Endpoynt_ClearSETUP();
    Endpoynt_Write_16_LE(CurrentStatus);
    Endpoynt_ClearIN();
    Endpoynt_ClearStatusStage();
}

static void UZB_Device_SetConfiguration(void)
{
    if ((uint8_t)USB_ControlRequest.wValue > FIXED_NUM_CONFIGURATIONS)
        return;

    Endpoynt_ClearSETUP();
    USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
    Endpoynt_ClearStatusStage();

    if (USB_Device_ConfigurationNumber)
        USB_DeviceState = DEVIZE_STATE_Configured;
    else
        USB_DeviceState = (UZB_Device_IsAddressSet()) ? DEVIZE_STATE_Configured : DEVIZE_STATE_Powered;

    EVENT_USB_Device_ConfigurationChanged();
}

#define UZB_STRING_LEN(UnicodeChars)  (sizeof(UZB_Descriptor_Header_t) + ((UnicodeChars) << 1))


#ifndef LUFA

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
#endif

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

uint8_t Endpoynt_Write_Control_PStream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        Endpoynt_ClearIN();

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState_LCL == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;

        if (Endpoynt_IsSETUPReceived())
            return ENDPOYNT_RWCSTREAM_HostAborted;

        if (Endpoynt_IsOUTReceived())
            break;

        if (Endpoynt_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoynt_BytesInEndpoint();

            while (Length && (BytesInEndpoint < USB_Device_ControlEndpointSize))
            {
                Endpoynt_Write_8(pgm_read_byte(DataStream));
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }

            LastPacketFull = (BytesInEndpoint == USB_Device_ControlEndpointSize);
            Endpoynt_ClearIN();
        }
    }

    while ((Endpoynt_IsOUTReceived()) == 0)
    {
        if (USB_DeviceState == DEVIZE_STATE_Unattached)
            return ENDPOYNT_RWCSTREAM_DeviceDisconnected;

        if (USB_DeviceState == DEVIZE_STATE_Suspended)
            return ENDPOYNT_RWCSTREAM_BusSuspended;

        if (Endpoynt_IsSETUPReceived())
            return ENDPOYNT_RWCSTREAM_HostAborted;
    }

    return ENDPOYNT_RWCSTREAM_NoError;
}

static void UZB_Device_GetDescriptor(void)
{
    const void* DescriptorPointer;
    uint16_t    DescriptorSize;

    if ((DescriptorSize = CALLBACK_USB_GetDescriptor(USB_ControlRequest.wValue,
            USB_ControlRequest.wIndex, &DescriptorPointer)) == 0)
    {
        return;
    }

    Endpoynt_ClearSETUP();
    Endpoynt_Write_Control_PStream_LE(DescriptorPointer, DescriptorSize);
    Endpoynt_ClearOUT();
}

void UZB_Device_ProcessControlRequest(void)
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USBRequest); i++)
        *(RequestHeader++) = Endpoynt_Read_8();

    uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

    switch (USB_ControlRequest.bRequest)
    {
    case REQ_SetInterface:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_INTERFACE))
        {
            *p_ueintx &= ~(1<<rxstpi);
            Endpoynt_ClearStatusStage();
            StreamingAudioInterfaceSelected = ((USB_ControlRequest.wValue) != 0);
        }
        break;
    case AUDYO_REQ_GetStatus:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE) ||
		    bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            *p_ueintx &= ~(1<<rxstpi);
            Endpoynt_ClearStatusStage();
        }
        break;
    case AUDYO_REQ_SetCurrent:
        if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            uint8_t EndpointAddress = (uint8_t)USB_ControlRequest.wIndex;
            uint8_t EndpointControl = (USB_ControlRequest.wValue >> 8);

            if ((EndpointAddress == AUDIO_STREAM_EPADDR) &&
                    (EndpointControl == AUDYO_EPCONTROL_SamplingFreq))
            {
                uint8_t SampleRate[3];
                Endpoynt_ClearSETUP();
                Endpoynt_Read_Control_Stream_LE(SampleRate, sizeof(SampleRate));
                Endpoynt_ClearIN();

                CurrentAudioSampleFrequency = (((uint32_t)SampleRate[2] << 16) |
                        ((uint32_t)SampleRate[1] << 8) | (uint32_t)SampleRate[0]);

                OCR0A = (F_CPU >> 3) / CurrentAudioSampleFrequency - 1;
            }
        }
        break;
    case AUDYO_REQ_GetCurrent:
        if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_ENDPOINT))
        {
            uint8_t EndpointAddress = (uint8_t)USB_ControlRequest.wIndex;
            uint8_t EndpointControl = (USB_ControlRequest.wValue >> 8);

            if ((EndpointAddress == AUDYO_STREAM_EPADDR) &&
                    (EndpointControl == AUDYO_EPCONTROL_SamplingFreq))
            {
					uint8_t SampleRate[3];
					SampleRate[2] = (CurrentAudioSampleFrequency >> 16);
					SampleRate[1] = (CurrentAudioSampleFrequency >> 8);
					SampleRate[0] = (CurrentAudioSampleFrequency &  0xFF);
					Endpoynt_ClearSETUP();
					Endpoynt_Write_Control_Stream_LE(SampleRate, sizeof(SampleRate));
					Endpoynt_ClearOUT();
            }
        }

        break;
	}

    if (Endpoynt_IsSETUPReceived())
    {
        switch (USB_ControlRequest.bRequest)
        {
        case REQ_GetStatus:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                UZB_Device_GetStatus();
            }

            break;
        case REQ_ClearFeature:
        case REQ_SetFeature:
            if ((bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_ENDPOINT)))
            {
                UZB_Device_ClearSetFeature();
            }

            break;
        case REQ_SetAddress:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
            {
                uint8_t DeviceAddress = (USB_ControlRequest.wValue & 0x7F);
                UZB_Device_SetDeviceAddress(DeviceAddress);
                Endpoynt_ClearSETUP();
                Endpoynt_ClearStatusStage();
                while ((Endpoynt_IsINReady()) == 0);
                UZB_Device_EnableDeviceAddress(DeviceAddress);
                USB_DeviceState = DeviceAddress ? DEVIZE_STATE_Addressed : DEVIZE_STATE_Default;
            }
            break;
        case REQ_GetDescriptor:
            if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
            {
                UZB_Device_GetDescriptor();
            }

            break;
        case REQ_GetConfiguration:
            if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
                UZB_Device_GetConfiguration();

            break;
        case REQ_SetConfiguration:
            if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
                UZB_Device_SetConfiguration();

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


ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
	uint8_t PrevEndpoint = Endpoynt_GetCurrentEndpoint();
	Endpoynt_SelectEndpoint(AUDIO_STREAM_EPADDR);

	if (Endpoynt_IsOUTReceived() && StreamingAudioInterfaceSelected)
	{
		int8_t LeftSample_8Bit   = ((int16_t)Endpoynt_Read_16_LE() >> 8);
		int8_t RightSample_8Bit  = ((int16_t)Endpoynt_Read_16_LE() >> 8);
		int8_t MixedSample_8Bit  = (((int16_t)LeftSample_8Bit + (int16_t)RightSample_8Bit) >> 1);

		if ((Endpoynt_IsReadWriteAllowed()) == 0)
			Endpoynt_ClearOUT();

#if defined(AUDIO_OUT_MONO)
		OCR3A = (MixedSample_8Bit ^ (1 << 7));
#elif defined(AUDIO_OUT_STEREO)
		OCR3A = (LeftSample_8Bit  ^ (1 << 7));
		OCR3B = (RightSample_8Bit ^ (1 << 7));
#elif defined(AUDIO_OUT_PORTC)
		PORTC = MixedSample_8Bit;
#endif

		uint8_t LEDMask = LEDS_NO_LEDS;

		if (MixedSample_8Bit > 16)
		    LEDMask = (LEDS_LED1 | LEDS_LED2 | LEDS_LED3 | LEDS_LED4);
		else if (MixedSample_8Bit > 8)
		    LEDMask = (LEDS_LED1 | LEDS_LED2 | LEDS_LED3);
		else if (MixedSample_8Bit > 4)
		    LEDMask = (LEDS_LED1 | LEDS_LED2);
		else if (MixedSample_8Bit > 2)
		    LEDMask = (LEDS_LED1);

		LEDs_SetAllLEDs(LEDMask);
	}

	Endpoynt_SelectEndpoint(PrevEndpoint);
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    if (UDINT & 1<<SOFI && UDIEN & 1<<SOFE)
    {
        UDINT &= ~(1<<SOFI);
        //EVENT_USB_Device_StartOfFrame();
    }

    if (USBINT & 1<<VBUSTI && USBCON & 1<<VBUSTE)
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = 1<<PINDIV;
            PLLCSR = 1<<PINDIV | 1<<PLLE;
            while ((PLLCSR & 1<<PLOCK) == 0);
            USB_DeviceState = DEVIZE_STATE_Powered;
            EVENT_USB_Device_Connect();
        }
        else
        {
            PLLCSR = 0;
            USB_DeviceState = DEVIZE_STATE_Unattached;
            EVENT_USB_Device_Disconnect();
        }
    }

    if (UDINT & 1<<SUSPI && UDIEN & 1<<SUSPE)
    {
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        USBCON |= 1<<FRZCLK;
        PLLCSR = 0;
        USB_DeviceState = DEVIZE_STATE_Suspended;
        //EVENT_USB_Device_Suspend();
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = 1<<PINDIV;
        PLLCSR = 1<<PINDIV | 1<<PLLE;
        while ((PLLCSR & 1<<PLOCK) == 0);
        USBCON &= ~(1<<FRZCLK);
        UDINT &= ~(1<<WAKEUPI);
        UDIEN &= ~(1<<WAKEUPE);
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            USB_DeviceState = DEVIZE_STATE_Configured;
        else
            USB_DeviceState = UDADDR & 1<<ADDEN ? DEVIZE_STATE_Addressed : DEVIZE_STATE_Powered;

        //EVENT_USB_Device_WakeUp();
    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        USB_DeviceState = DEVIZE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        Endpoynt_ConfigureEndpoint(0, 0, 8, 1);
        UEIENX |= 1<<RXSTPE;
        //EVENT_USB_Device_Reset();
    }
}

ISR(USB_COM_vect, ISR_BLOCK)
{
    uint8_t PrevSelectedEndpoint = Endpoynt_GetCurrentEndpoint();
    Endpoynt_SelectEndpoint(0);
    UEIENX &= ~(1<<RXSTPE);
    sei();
    UZB_Device_ProcessControlRequest();
    Endpoynt_SelectEndpoint(0);
    UEIENX |= 1<<RXSTPE;
    Endpoynt_SelectEndpoint(PrevSelectedEndpoint);
}

int main(void)
{
    LEDs_Init();
    USBCON &= ~(1<<OTGPADE);
    UHWCON |= 1<<UVREGE;
    PLLFRQ = 1<<PDIV2;
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1 << USBE);
    USBCON |=  (1 << USBE);
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0;
    USB_DeviceState = DEVIZE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);
    USBCON |= 1<<VBUSTE;
    Endpoynt_ConfigureEndpoint(0, 0, 8, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
    LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
    sei();

	for (;;)
	{
	}
}


