#ifndef _USBSOUND_H_
#define _USBSOUND_H_

#ifndef ATTR_PACKED
#define ATTR_PACKED __attribute__ ((packed))
#endif

static constexpr uint16_t
    AUDIO_STREAM_EPSIZE = 256,
    AUDIO_TERMINAL_UNDEFINED          = 0x0100,
    AUDIO_TERMINAL_STREAMING          = 0x0101,
    AUDIO_TERMINAL_VENDOR             = 0x01FF,
    AUDIO_TERMINAL_IN_UNDEFINED       = 0x0200,
    AUDIO_TERMINAL_OUT_SPEAKER        = 0x0301,
    AUDIO_TERMINAL_OUT_HEADPHONES     = 0x0302,
    AUDIO_TERMINAL_OUT_HEAD_MOUNTED   = 0x0303;

static constexpr uint8_t
    AUDIO_STREAM_EPADDR = ENDPOINT_DIR_OUT | 1,
    USB_Device_ControlEndpointSize = 8,
    EP_TIPE_ISOCHRONOUS = 0x01,
    AUDIO_CHANNEL_LEFT_FRONT       = 1 << 0,
    AUDIO_CHANNEL_RIGHT_FRONT      = 1 << 1,
    AUDIO_EP_SAMPLE_FREQ_CONTROL   = 1 << 0,
    AUDIO_EP_PITCH_CONTROL         = 1 << 1,
    AUDIO_EP_FULL_PACKETS_ONLY     = 1 << 7,
    AUDIO_EP_ACCEPTS_SMALL_PACKETS = 0 << 7,
    AUDIO_DSUBTYPE_CSInterface_General        = 0x01,
    AUDIO_DSUBTYPE_CSInterface_FormatType     = 0x02,
    AUDIO_DSUBTYPE_CSInterface_FormatSpecific = 0x03,
    AUDIO_DSUBTYPE_CSEndpoint_General         = 0x01,
    AUDIO_CSCP_AudioClass                   = 0x01,
    AUDIO_CSCP_ControlSubclass               = 0x01,
    AUDIO_CSCP_ControlProtocol               = 0x00,
    AUDIO_CSCP_AudioStreamingSubclass        = 0x02,
    AUDIO_CSCP_MIDIStreamingSubclass         = 0x03,
    AUDIO_CSCP_StreamingProtocol             = 0x00,
    AUDIO_DSUBTYPE_CSInterface_Header       = 0x01,
    AUDIO_DSUBTYPE_CSInterface_InputTerminal = 0x02,
    AUDIO_DSUBTYPE_CSInterface_OutputTerminal = 0x03,
    AUDIO_DSUBTYPE_CSInterface_Mixer        = 0x04,
    AUDIO_DSUBTYPE_CSInterface_Selector      = 0x05,
    AUDIO_DSUBTYPE_CSInterface_Feature       = 0x06,
    AUDIO_DSUBTYPE_CSInterface_Processing    = 0x07,
    AUDIO_DSUBTYPE_CSInterface_Extension     = 0x08,
    AUDIO_REQ_SetCurrent   = 0x01,
    AUDIO_REQ_SetMinimum   = 0x02,
    AUDIO_REQ_SetMaximum   = 0x03,
    AUDIO_REQ_SetResolution = 0x04,
    AUDIO_REQ_SetMemory    = 0x05,
    AUDIO_REQ_GetCurrent   = 0x81,
    AUDIO_REQ_GetMinimum   = 0x82,
    AUDIO_REQ_GetMaximum   = 0x83,
    AUDIO_REQ_GetResolution = 0x84,
    AUDIO_REQ_GetMemory     = 0x85,
    AUDIO_REQ_GetStatus     = 0xFF,
    AUDIO_EPCONTROL_SamplingFreq = 0x01,
    AUDIO_EPCONTROL_Pitch        = 0x02;

struct DescAudioInTerminal
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint8_t TerminalID;
    uint16_t TerminalType;
    uint8_t AssociatedOutputTerminal;
    uint8_t TotalChannels;
    uint16_t ChannelConfig;
    uint8_t ChannelStrIndex;
    uint8_t TerminalStrIndex;
} ATTR_PACKED;

struct DescAudioOutTerminal
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint8_t TerminalID;
    uint16_t TerminalType;
    uint8_t AssociatedInputTerminal;
    uint8_t SourceID;
    uint8_t TerminalStrIndex;
} ATTR_PACKED;

struct DescAudioIface
{
    uint8_t size;
    uint8_t type;
    uint8_t Subtype;
    uint16_t ACSpecification;
    uint16_t TotalLength;
    uint8_t InCollection;
    uint8_t InterfaceNumber;
} ATTR_PACKED;

struct SampleFreq
{
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
} ATTR_PACKED;

struct AudioEndpoint
{
    DescEndpoint endpoint;
    uint8_t Refresh;
    uint8_t SyncEndpointNumber;
} ATTR_PACKED;

struct AudioEndpointSpc
{
    uint8_t size;
    uint8_t type;
    uint8_t subtype;
    uint8_t attributes;
    uint8_t lockDelayUnits;
    uint16_t lockDelay;
}
ATTR_PACKED;

struct DescAudioIfaceAS
{
    uint8_t size;
    uint8_t type;
    uint8_t subtype;
    uint8_t terminalLink;
    uint8_t frameDelay;
    uint16_t audioFormat;
}
__attribute__ ((packed));

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

struct USB_Descriptor_Configuration_t
{
    DescConf Config;
    DescIface Audio_ControlInterface;
    DescAudioIface Audio_ControlInterface_SPC;
    DescAudioInTerminal Audio_InputTerminal;
    DescAudioOutTerminal Audio_OutputTerminal;
    DescIface Audio_StreamInterface_Alt0;
    DescIface Audio_StreamInterface_Alt1;
    DescAudioIfaceAS Audio_StreamInterface_SPC;
    AudioFormat Audio_AudioFormat;
    SampleFreq Audio_AudioFormatSampleRates[5];
    AudioEndpoint Audio_StreamEndpoint;
    AudioEndpointSpc Audio_StreamEndpoint_SPC;
} ATTR_PACKED;

#define INTERFACE_ID_AudioControl 0
#define INTERFACE_ID_AudioStream  1



#endif



