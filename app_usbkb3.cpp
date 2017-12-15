#define ARCH ARCH_AVR8
#define F_CPU 16000000UL
#define F_USB 16000000UL

#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>

#ifndef __USBCONTROLLER_H__
#define __USBCONTROLLER_H__

#ifndef __USBMODE_H__
#define __USBMODE_H__

#ifndef __LUFA_COMMON_H__
#define __LUFA_COMMON_H__

#define __INCLUDE_FROM_COMMON_H

#include <stdint.h>
#include <stddef.h>

#ifndef __LUFA_ARCHITECTURES_H__
#define __LUFA_ARCHITECTURES_H__

#define ARCH_AVR8           0
#define ARCH_UC3            1
#define ARCH_XMEGA          2

#if !defined(__DOXYGEN__)
#define ARCH_           ARCH_AVR8

#if !defined(ARCH)
#define ARCH        ARCH_AVR8
#endif
#endif

#endif


#if defined(__GNUC__) || defined(__DOXYGEN__)
#define GCC_FORCE_POINTER_ACCESS(StructPtr)   __asm__ __volatile__("" : "=b" (StructPtr) : "0" (StructPtr))

#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");
#define GCC_IS_COMPILE_CONST(x)               __builtin_constant_p(x)
#else
#define GCC_FORCE_POINTER_ACCESS(StructPtr)
#define GCC_MEMORY_BARRIER()
#define GCC_IS_COMPILE_CONST(x)               0
#endif

#if (__GNUC__ >= 3) || defined(__DOXYGEN__)
#define ATTR_NO_RETURN               __attribute__ ((noreturn))
#define ATTR_WARN_UNUSED_RESULT      __attribute__ ((warn_unused_result))
#define ATTR_NON_NULL_PTR_ARG(...)   __attribute__ ((nonnull (__VA_ARGS__)))
#define ATTR_ALWAYS_INLINE           __attribute__ ((always_inline))
#define ATTR_CONST                   __attribute__ ((const))
#endif


#define ATTR_PACKED                      __attribute__ ((packed))

#if defined(__DOXYGEN__)
typedef MACHINE_REG_t uint_reg_t;
#elif (ARCH == ARCH_AVR8)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <math.h>
#include <util/delay.h>

typedef uint8_t uint_reg_t;

#define ARCH_HAS_EEPROM_ADDRESS_SPACE
#define ARCH_HAS_FLASH_ADDRESS_SPACE
#define ARCH_HAS_MULTI_ADDRESS_SPACE
#define ARCH_LITTLE_ENDIAN

#ifndef __LUFA_ENDIANNESS_H__
#define __LUFA_ENDIANNESS_H__

#define SWAPENDIAN_16(x) (uint16_t)((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

#define SWAPENDIAN_32(x)  (uint32_t)((((x) & 0xFF000000UL) >> 24UL) | (((x) & 0x00FF0000UL) >> 8UL) | \
                         (((x) & 0x0000FF00UL) << 8UL)  | (((x) & 0x000000FFUL) << 24UL))

#if defined(ARCH_BIG_ENDIAN) && !defined(le16_to_cpu)
#define le16_to_cpu(x)           SwapEndian_16(x)
#define be16_to_cpu(x)           (x)
#define be32_to_cpu(x)           (x)
#define cpu_to_le16(x)           SwapEndian_16(x)
#define cpu_to_le32(x)           SwapEndian_32(x)
#define cpu_to_be16(x)           (x)
#define cpu_to_be32(x)           (x)
#define LE16_TO_CPU(x)           SWAPENDIAN_16(x)
#define LE32_TO_CPU(x)           SWAPENDIAN_32(x)
#define BE16_TO_CPU(x)           (x)
#define BE32_TO_CPU(x)           (x)
#define CPU_TO_LE16(x)           SWAPENDIAN_16(x)
#define CPU_TO_LE32(x)           SWAPENDIAN_32(x)
#define CPU_TO_BE16(x)           (x)
#define CPU_TO_BE32(x)           (x)
#elif !defined(le16_to_cpu)
#define le16_to_cpu(x)           (x)
#define be16_to_cpu(x)           SwapEndian_16(x)
#define be32_to_cpu(x)           SwapEndian_32(x)
#define cpu_to_le16(x)           (x)
#define cpu_to_le32(x)           (x)
#define cpu_to_be16(x)           SwapEndian_16(x)
#define cpu_to_be32(x)           SwapEndian_32(x)
#define LE16_TO_CPU(x)           (x)
#define LE32_TO_CPU(x)           (x)
#define BE16_TO_CPU(x)           SWAPENDIAN_16(x)
#define BE32_TO_CPU(x)           SWAPENDIAN_32(x)
#define CPU_TO_LE16(x)           (x)
#define CPU_TO_LE32(x)           (x)
#define CPU_TO_BE16(x)           SWAPENDIAN_16(x)
#define CPU_TO_BE32(x)           SWAPENDIAN_32(x)
#endif

			
static inline uint16_t SwapEndian_16(const uint16_t Word)
    ATTR_WARN_UNUSED_RESULT ATTR_CONST ATTR_ALWAYS_INLINE;
static inline uint16_t SwapEndian_16(const uint16_t Word)
{
    if (GCC_IS_COMPILE_CONST(Word))
        return SWAPENDIAN_16(Word);

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

static inline uint32_t SwapEndian_32(const uint32_t DWord)
    ATTR_WARN_UNUSED_RESULT ATTR_CONST ATTR_ALWAYS_INLINE;
static inline uint32_t SwapEndian_32(const uint32_t DWord)
{
    if (GCC_IS_COMPILE_CONST(DWord))
        return SWAPENDIAN_32(DWord);

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

static inline void SwapEndian_n(void* const Data,
                                uint8_t Length) ATTR_NON_NULL_PTR_ARG(1);
static inline void SwapEndian_n(void* const Data, uint8_t Length)
{
    uint8_t* CurrDataPos = (uint8_t*)Data;

    while (Length > 1)
    {
        uint8_t Temp = *CurrDataPos;
        *CurrDataPos = *(CurrDataPos + Length - 1);
        *(CurrDataPos + Length - 1) = Temp;
        CurrDataPos++;
        Length -= 2;
    }
}
#endif


#elif (ARCH == ARCH_UC3)
#elif (ARCH == ARCH_XMEGA)
#else
#error Unknown device architecture specified.
#endif

#if !defined(__DOXYGEN__)
#define MACROS                  do
#define MACROE                  while (0)
#endif

			
#if !defined(MAX) || defined(__DOXYGEN__)
#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
#endif

	
#if !defined(MIN) || defined(__DOXYGEN__)
#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif

#if !defined(STRINGIFY) || defined(__DOXYGEN__)
#define STRINGIFY(x)            #x
#define STRINGIFY_EXPANDED(x)   STRINGIFY(x)
#endif

#if !defined(CONCAT) || defined(__DOXYGEN__)
#define CONCAT(x, y)            x ## y
#define CONCAT_EXPANDED(x, y)   CONCAT(x, y)
#endif

	

		
			static inline uint8_t BitReverse(uint8_t Byte) ATTR_WARN_UNUSED_RESULT ATTR_CONST;
			static inline uint8_t BitReverse(uint8_t Byte)
			{
				Byte = (((Byte & 0xF0) >> 4) | ((Byte & 0x0F) << 4));
				Byte = (((Byte & 0xCC) >> 2) | ((Byte & 0x33) << 2));
				Byte = (((Byte & 0xAA) >> 1) | ((Byte & 0x55) << 1));

				return Byte;
			}

			
			static inline void Delay_MS(uint16_t Milliseconds) ATTR_ALWAYS_INLINE;
			static inline void Delay_MS(uint16_t Milliseconds)
			{
				if (GCC_IS_COMPILE_CONST(Milliseconds))
				{
					_delay_ms(Milliseconds);
				}
				else
				{
					while (Milliseconds--)
					  _delay_ms(1);
				}
			}


static inline uint_reg_t GetGlobalInterruptMask(void) ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
static inline uint_reg_t GetGlobalInterruptMask(void)
{
    GCC_MEMORY_BARRIER();
    return SREG;
}

	
static inline void SetGlobalInterruptMask(const uint_reg_t GlobalIntState) ATTR_ALWAYS_INLINE;
static inline void SetGlobalInterruptMask(const uint_reg_t GlobalIntState)
{
				GCC_MEMORY_BARRIER();
				SREG = GlobalIntState;
				GCC_MEMORY_BARRIER();
}

#endif



#if defined(__DOXYGEN__)
#define USB_SERIES_2_AVR
#define USB_SERIES_4_AVR
#define USB_SERIES_6_AVR
#define USB_SERIES_7_AVR

#define USB_SERIES_UC3A0_AVR
#define USB_SERIES_UC3A1_AVR
#define USB_SERIES_UC3A3_AVR
#define USB_SERIES_UC3A4_AVR
#define USB_SERIES_UC3B0_AVR
#define USB_SERIES_UC3B1_AVR
#define USB_SERIES_A1U_XMEGA

#define USB_SERIES_A3U_XMEGA

#define USB_SERIES_A4U_XMEGA

#define USB_SERIES_B1_XMEGA

#define USB_SERIES_B3_XMEGA

#define USB_SERIES_C3_XMEGA

#define USB_SERIES_C4_XMEGA

#define USB_CAN_BE_DEVICE

#define USB_CAN_BE_HOST

#define USB_CAN_BE_BOTH
#else
#if (defined(__AVR_AT90USB162__) || defined(__AVR_AT90USB82__)  || \
     defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__))
#define USB_SERIES_2_AVR
#define USB_CAN_BE_DEVICE
#elif (defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__))
#define USB_SERIES_4_AVR
#define USB_CAN_BE_DEVICE
#elif (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))
#define USB_SERIES_6_AVR
#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_AT90USB647__) || defined(__AVR_AT90USB1287__))
				#define USB_SERIES_7_AVR
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3A0512__) || defined(__AVR32_UC3A0256__) || \
			       defined(__AVR32_UC3A0128__) || defined(__AVR32_UC3A064__))
				#define USB_SERIES_UC3A0_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3A1512__) || defined(__AVR32_UC3A1256__) || \
			       defined(__AVR32_UC3A1128__) || defined(__AVR32_UC3A164__))
				#define USB_SERIES_UC3A1_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3A3256__) || defined(__AVR32_UC3A3256S__) || \
			       defined(__AVR32_UC3A3128__) || defined(__AVR32_UC3A3128S__) || \
			       defined(__AVR32_UC3A364__)  || defined(__AVR32_UC3A364S__))
				#define USB_SERIES_UC3A3_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3A4256__) || defined(__AVR32_UC3A4256S__) || \
			       defined(__AVR32_UC3A4128__) || defined(__AVR32_UC3A4128S__) || \
			       defined(__AVR32_UC3A464__)  || defined(__AVR32_UC3A464S__))
				#define USB_SERIES_UC3A4_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3B0512__) || defined(__AVR32_UC3B0256__) || \
			       defined(__AVR32_UC3B0128__) || defined(__AVR32_UC3B064__))
				#define USB_SERIES_UC3B0_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR32_UC3B1512__) || defined(__AVR32_UC3B1256__) || \
			       defined(__AVR32_UC3B1128__) || defined(__AVR32_UC3B164__))
				#define USB_SERIES_UC3B1_AVR32
				#define USB_CAN_BE_DEVICE
				#define USB_CAN_BE_HOST
			#elif (defined(__AVR_ATxmega128A1U__) || defined(__AVR_ATxmega64A1U__))
				#define USB_SERIES_A1U_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega64A3U__) || defined(__AVR_ATxmega128A3U__) || \
			       defined(__AVR_ATxmega192A3U__) || defined(__AVR_ATxmega256A3U__))
				#define USB_SERIES_A3U_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega256A3BU__))
				#define USB_SERIES_A3BU_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega16A4U__) || defined(__AVR_ATxmega32A4U__) || \
			       defined(__AVR_ATxmega64A4U__) || defined(__AVR_ATxmega128A4U__))
				#define USB_SERIES_A4U_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega128B1__) || defined(__AVR_ATxmega64B1__))
				#define USB_SERIES_B1_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega128B3__) || defined(__AVR_ATxmega64B3__))
				#define USB_SERIES_B3_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega128C3__) || defined(__AVR_ATxmega64C3__) || \
			       defined(__AVR_ATxmega192C3__) || defined(__AVR_ATxmega256C3__) || \
				   defined(__AVR_ATxmega384C3__))
				#define USB_SERIES_C3_XMEGA
				#define USB_CAN_BE_DEVICE
			#elif (defined(__AVR_ATxmega16C4__) || defined(__AVR_ATxmega32C4__))
				#define USB_SERIES_C4_XMEGA
				#define USB_CAN_BE_DEVICE
			#endif

			#if (defined(USB_HOST_ONLY) && defined(USB_DEVICE_ONLY))
				#error USB_HOST_ONLY and USB_DEVICE_ONLY are mutually exclusive.
			#elif defined(USB_HOST_ONLY)
				#if !defined(USB_CAN_BE_HOST)
#error USB_HOST_ONLY is not available for the currently selected microcontroller model.
#else
#undef USB_CAN_BE_DEVICE
#endif
#elif defined(USB_DEVICE_ONLY)
#if !defined(USB_CAN_BE_DEVICE)
#error USB_DEVICE_ONLY is not available for the currently selected microcontroller model.
#else
#undef USB_CAN_BE_HOST
#endif
#endif
#endif
#endif



#define ENDPOINT_DIR_MASK                  0x80
#define ENDPOINT_DIR_OUT                   0x00
#define ENDPOINT_DIR_IN                    0x80
#define EP_TYPE_CONTROL                    0x00
#define EP_TYPE_ISOCHRONOUS                0x01
#define EP_TYPE_BULK                       0x02
#define EP_TYPE_INTERRUPT                  0x03

enum USB_Modes_t
{
    USB_MODE_None   = 0,
    USB_MODE_Device = 1,
    USB_MODE_Host   = 2,
    USB_MODE_UID    = 3,
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

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                   const void** const DescriptorAddress
                                   ) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);


#ifndef __USBDEVICE_AVR8_H__
#define __USBDEVICE_AVR8_H__

#ifndef __USBDESCRIPTORS_H__
#define __USBDESCRIPTORS_H__




void EVENT_USB_UIDChange(void);
void EVENT_USB_Host_HostError(const uint8_t ErrorCode);
void EVENT_USB_Host_DeviceAttached(void);
void EVENT_USB_Host_DeviceUnattached(void);

void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
			                                            const uint8_t SubErrorCode);

void EVENT_USB_Host_DeviceEnumerationComplete(void);


void EVENT_USB_Host_StartOfFrame(void);

void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_StartOfFrame(void);


#define NO_DESCRIPTOR    0
#define USB_CONFIG_POWER_MA(mA)           ((mA) >> 1)

#define USB_STRING_LEN(UnicodeChars) (sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))

#define USB_STRING_DESCRIPTOR(String)     { .Header = {.Size = sizeof(USB_Descriptor_Header_t) + (sizeof(String) - 2), .Type = DTYPE_String}, .UnicodeString = String }

#define USB_CONFIG_ATTR_RESERVED          0x80
#define USB_CONFIG_ATTR_SELFPOWERED       0x40
#define USB_CONFIG_ATTR_REMOTEWAKEUP      0x20
#define ENDPOINT_ATTR_NO_SYNC             (0 << 2)
#define ENDPOINT_ATTR_ASYNC               (1 << 2)
#define ENDPOINT_ATTR_ADAPTIVE            (2 << 2)
#define ENDPOINT_ATTR_SYNC                (3 << 2)
#define ENDPOINT_USAGE_DATA               (0 << 4)
#define ENDPOINT_USAGE_FEEDBACK           (1 << 4)
#define ENDPOINT_USAGE_IMPLICIT_FEEDBACK  (2 << 4)

enum USB_DescriptorTypes_t
{
    DTYPE_Device = 0x01,
				DTYPE_Configuration = 0x02,
				DTYPE_String = 0x03, /**< Indicates that the descriptor is a string descriptor. */
				DTYPE_Interface = 0x04, /**< hat the descriptor is an interface descriptor. */
				DTYPE_Endpoint = 0x05, /**< In the descriptor is an endpoint descriptor. */
				DTYPE_DeviceQualifier = 0x06, /**< Ir is a device qualifier descriptor. */
				DTYPE_Other                     = 0x07, /**< Indicates that the descriptor is of other type. */
				DTYPE_InterfacePower            = 0x08, /**< Indicates that the descriptor is an interface power descriptor. */
				DTYPE_InterfaceAssociation      = 0x0B, /**< Indicates that the descriptor is an interface association descriptor. */
				DTYPE_CSInterface               = 0x24, /**< Indicates that the descriptor is a class specific interface descriptor. */
				DTYPE_CSEndpoint                = 0x25, /**< Indicates that the descriptor is a class specific endpoint descriptor. */
			};

			/** Enum for possible Class, Subclass and Protocol values of device and interface descriptors. */
			enum USB_Descriptor_ClassSubclassProtocol_t
			{
				USB_CSCP_NoDeviceClass          = 0x00, /**< Descriptor Class value indicating that the device does not belong
				                                         *   to a particular class at the device level.
				                                         */
				USB_CSCP_NoDeviceSubclass       = 0x00, /**< Descriptor Subclass value indicating that the device does not belong
				                                         *   to a particular subclass at the device level.
				                                         */
				USB_CSCP_NoDeviceProtocol       = 0x00, /**< Descriptor Protocol value indicating that the device does not belong
				                                         *   to a particular protocol at the device level.
				                                         */
				USB_CSCP_VendorSpecificClass    = 0xFF, /**< Descriptor Class value indicating that the device/interface belongs
				                                         *   to a vendor specific class.
				                                         */
				USB_CSCP_VendorSpecificSubclass = 0xFF,
				USB_CSCP_VendorSpecificProtocol = 0xFF, /**< Descriptor Protocol value indicating that the device/interface belongs
				                                         *   to a vendor specific protocol.
				                                         */
				USB_CSCP_IADDeviceClass         = 0xEF, /**< Descriptor Class value indicating that the device belongs to the
				                                         *   Interface Association Descriptor class.
				                                         */
				USB_CSCP_IADDeviceSubclass      = 0x02, /**< Descriptor Subclass value indicating that the device belongs to the
				                                         *   Interface Association Descriptor subclass.
				                                         */
				USB_CSCP_IADDeviceProtocol      = 0x01, /**< Descriptor Protocol value indicating that the device belongs to the
				                                         *   Interface Association Descriptor protocol.
				                                         */
			};

			typedef struct
			{
				uint8_t Size; /**< Size of the descriptor, in bytes. */
				uint8_t Type; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
				               *   given by the specific class.
				               */
			} ATTR_PACKED USB_Descriptor_Header_t;

			
			typedef struct
			{
				uint8_t bLength; /**< Size of the descriptor, in bytes. */
				uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
				                          *   given by the specific class.
				                          */
			} ATTR_PACKED USB_StdDescriptor_Header_t;

			typedef struct
			{
				USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

				uint16_t USBSpecification; /**< BCD of the supported USB specification.
				                            *
				                            *   \see \ref VERSION_BCD() utility macro.
				                            */
				uint8_t  Class; /**< USB device class. */
				uint8_t  SubClass; /**< USB device subclass. */
				uint8_t  Protocol; /**< USB device protocol. */

				uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */

				uint16_t VendorID; /**< Vendor ID for the USB product. */
				uint16_t ProductID; /**< Unique product ID for the USB product. */
				uint16_t ReleaseNumber; /**< Product release (version) number.
				                         *
				                         *   \see \ref VERSION_BCD() utility macro.
				                         */
				uint8_t  ManufacturerStrIndex; /**< String index for the manufacturer's name. The
				                                *   host will request this string via a separate
				                                *   control request for the string descriptor.
				                                *
				                                *   \note If no string supplied, use \ref NO_DESCRIPTOR.
				                                */
				uint8_t  ProductStrIndex; /**< String index for the product name/details.
				                           *
				                           *  \see ManufacturerStrIndex structure entry.
				                           */
				uint8_t  SerialNumStrIndex; /**< String index for the product's globally unique hexadecimal
				                             *   serial number, in uppercase Unicode ASCII.
				                             *
				                             *  \note On some microcontroller models, there is an embedded serial number
				                             *        in the chip which can be used for the device serial number.
				                             *        To use this serial number, set this to \c USE_INTERNAL_SERIAL.
				                             *        On unsupported devices, this will evaluate to \ref NO_DESCRIPTOR
				                             *        and will cause the host to generate a pseudo-unique value for the
				                             *        device upon insertion.
				                             *
				                             *  \see \c ManufacturerStrIndex structure entry.
				                             */
				uint8_t  NumberOfConfigurations; /**< Total number of configurations supported by
				                                  *   the device.
				                                  */
			} ATTR_PACKED USB_Descriptor_Device_t;

			typedef struct
			{
				uint8_t  bLength; /**< Size of the descriptor, in bytes. */
				uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
				                              *   given by the specific class.
				                              */
				uint16_t bcdUSB; /**< BCD of the supported USB specification.
				                  *
				                  *   \see \ref VERSION_BCD() utility macro.
				                  */
				uint8_t  bDeviceClass; /**< USB device class. */
				uint8_t  bDeviceSubClass; /**< USB device subclass. */
				uint8_t  bDeviceProtocol; /**< USB device protocol. */
				uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
				uint16_t idVendor; /**< Vendor ID for the USB product. */
				uint16_t idProduct; /**< Unique product ID for the USB product. */
				uint16_t bcdDevice; /**< Product release (version) number.
				                     *
				                     *   \see \ref VERSION_BCD() utility macro.
				                     */
				uint8_t  iManufacturer; /**< String index for the manufacturer's name. The
				                         *   host will request this string via a separate
				                         *   control request for the string descriptor.
				                         *
				                         *   \note If no string supplied, use \ref NO_DESCRIPTOR.
				                         */
				uint8_t  iProduct; /**< String index for the product name/details.
				                    *
				                    *  \see ManufacturerStrIndex structure entry.
				                    */
				uint8_t iSerialNumber;
				uint8_t  bNumConfigurations; /**< Total number of configurations supported by
				                              *   the device.
				                              */
			} ATTR_PACKED USB_StdDescriptor_Device_t;

			/** \brief Standard USB Device Qualifier Descriptor (LUFA naming conventions).
			 *
			 *  Type define for a standard Device Qualifier Descriptor. This structure uses LUFA-specific element names
			 *  to make each element's purpose clearer.
			 *
			 *  \see \ref USB_StdDescriptor_DeviceQualifier_t for the version of this type with standard element names.
			 */
			typedef struct
			{
				USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

				uint16_t USBSpecification; /**< BCD of the supported USB specification.
				                            *
				                            *   \see \ref VERSION_BCD() utility macro.
				                            */
				uint8_t  Class; /**< USB device class. */
				uint8_t  SubClass; /**< USB device subclass. */
				uint8_t  Protocol; /**< USB device protocol. */

				uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */
				uint8_t  NumberOfConfigurations; /**< Total number of configurations supported by
				                                  *   the device.
				                                  */
				uint8_t  Reserved; /**< Reserved for future use, must be 0. */
			} ATTR_PACKED USB_Descriptor_DeviceQualifier_t;

			typedef struct
			{
				uint8_t  bLength; /**< Size of the descriptor, in bytes. */
				uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
				                           *   given by the specific class.
				                           */
				uint16_t bcdUSB; /**< BCD of the supported USB specification.
				                  *
				                  *   \see \ref VERSION_BCD() utility macro.
				                  */
				uint8_t  bDeviceClass; /**< USB device class. */
				uint8_t  bDeviceSubClass; /**< USB device subclass. */
				uint8_t  bDeviceProtocol; /**< USB device protocol. */
				uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
				uint8_t  bNumConfigurations; /**< Total number of configurations supported by
				                              *   the device.
				                              */
				uint8_t  bReserved; /**< Reserved for future use, must be 0. */
			} ATTR_PACKED USB_StdDescriptor_DeviceQualifier_t;

typedef struct
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    uint16_t TotalConfigurationSize; /**< Size of the configuration descriptor header,
                                  *   and all sub descriptors inside the configuration.
	                                  */
    uint8_t  TotalInterfaces; /**< Total number of interfaces in the configuration. */

				uint8_t  ConfigurationNumber; /**< Configuration index of the current configuration. */
				uint8_t  ConfigurationStrIndex; /**< Index of a string descriptor describing the configuration. */

				uint8_t  ConfigAttributes; /**< Configuration attributes, comprised of a mask of \c USB_CONFIG_ATTR_* masks.
				                            *   On all devices, this should include USB_CONFIG_ATTR_RESERVED at a minimum.
				                            */

				uint8_t  MaxPowerConsumption; /**< Maximum power consumption of the device while in the
				                               *   current configuration, calculated by the \ref USB_CONFIG_POWER_MA()
				                               *   macro.
				                               */
			} ATTR_PACKED USB_Descriptor_Configuration_Header_t;

			/** \brief Standard USB Configuration Descriptor (USB-IF naming conventions).
			 *
			 *  Type define for a standard Configuration Descriptor header. This structure uses the relevant standard's given element names
			 *  to ensure compatibility with the standard.
			 *
			 *  \see \ref USB_Descriptor_Device_t for the version of this type with non-standard LUFA specific element names.
			 *
			 *  \note Regardless of CPU architecture, these values should be stored as little endian.
			 */
			typedef struct
			{
				uint8_t  bLength; /**< Size of the descriptor, in bytes. */
				uint8_t  bDescriptorType;
				uint16_t wTotalLength; /**< Size of the configuration descriptor header,
				                           *   and all sub descriptors inside the configuration.
				                           */
				uint8_t  bNumInterfaces; /**< Total number of interfaces in the configuration. */
				uint8_t  bConfigurationValue; /**< Configuration index of the current configuration. */
				uint8_t  iConfiguration; /**< Index of a string descriptor describing the configuration. */
				uint8_t  bmAttributes; /**< Configuration attributes, comprised of a mask of \c USB_CONFIG_ATTR_* masks.
				                        *   On all devices, this should include USB_CONFIG_ATTR_RESERVED at a minimum.
				                        */
				uint8_t  bMaxPower; /**< Maximum power consumption of the device while in the
				                     *   current configuration, calculated by the \ref USB_CONFIG_POWER_MA()
				                     *   macro.
				                     */
			} ATTR_PACKED USB_StdDescriptor_Configuration_Header_t;

			
			typedef struct
			{
				USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

				uint8_t InterfaceNumber; /**< Index of the interface in the current configuration. */
				uint8_t AlternateSetting; /**< Alternate setting for the interface number. The same
				                           *   interface number can have multiple alternate settings
				                           *   with different endpoint configurations, which can be
				                           *   selected by the host.
				                           */
				uint8_t TotalEndpoints; /**< Total number of endpoints in the interface. */

				uint8_t Class; /**< Interface class ID. */
				uint8_t SubClass; /**< Interface subclass ID. */
				uint8_t Protocol; /**< Interface protocol ID. */

				uint8_t InterfaceStrIndex; /**< Index of the string descriptor describing the interface. */
			} ATTR_PACKED USB_Descriptor_Interface_t;

typedef struct
{
    uint8_t bLength; /**< Size of the descriptor, in bytes. */
    uint8_t bDescriptorType; /**< iptor, either a value in \ref USB_DescriptorTypes_t or a va*/
    uint8_t bInterfaceNumber; /**< Index of the interface in the current configuration. */
    uint8_t bAlternateSetting; /**< Alternate setting for the interface number. The same
				                            *   interface number can have multiple alternate settings
				                            *   with different endpoint configurations, which can be
				                            *   selected by the host.
				                            */
				uint8_t bNumEndpoints; /**< Total number of endpoints in the interface. */
				uint8_t bInterfaceClass; /**< Interface class ID. */
				uint8_t bInterfaceSubClass; /**< Interface subclass ID. */
				uint8_t bInterfaceProtocol; /**< Interface protocol ID. */
				uint8_t iInterface; /**< Index of the string descriptor describing the
				                     *   interface.
				                     */
			} ATTR_PACKED USB_StdDescriptor_Interface_t;

			typedef struct
			{
				USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

				uint8_t FirstInterfaceIndex; /**< Index of the first associated interface. */
				uint8_t TotalInterfaces; /**< Total number of associated interfaces. */

				uint8_t Class; /**< Interface class ID. */
				uint8_t SubClass; /**< Interface subclass ID. */
				uint8_t Protocol; /**< Interface protocol ID. */

				uint8_t IADStrIndex; /**< Index of the string descriptor describing the
				                      *   interface association.
				                      */
			} ATTR_PACKED USB_Descriptor_Interface_Association_t;

			typedef struct
			{
				uint8_t bLength; /**< Size of the descriptor, in bytes. */
				uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
				                          *   given by the specific class.
				                          */
				uint8_t bFirstInterface; /**< Index of the first associated interface. */
				uint8_t bInterfaceCount; /**< Total number of associated interfaces. */
				uint8_t bFunctionClass; /**< Interface class ID. */
				uint8_t bFunctionSubClass; /**< Interface subclass ID. */
				uint8_t bFunctionProtocol; /**< Interface protocol ID. */
				uint8_t iFunction; /**< Index of the string descriptor describing the
				                    *   interface association.
				                    */
			} ATTR_PACKED USB_StdDescriptor_Interface_Association_t;

			typedef struct
			{
				USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

				uint8_t  EndpointAddress; /**< Logical address of the endpoint within the device for the current
				                           *   configuration, including direction mask.
				                           */
				uint8_t  Attributes; /**< Endpoint attributes, comprised of a mask of the endpoint type (EP_TYPE_*)
				                      *   and attributes (ENDPOINT_ATTR_*) masks.
				                      */
				uint16_t EndpointSize; /**< Size of the endpoint bank, in bytes. This indicates the maximum packet
				                        *   size that the endpoint can receive at a time.
				                        */
				uint8_t  PollingIntervalMS; /**< Polling interval in milliseconds for the endpoint if it is an INTERRUPT
				                             *   or ISOCHRONOUS type.
				                             */
			} ATTR_PACKED USB_Descriptor_Endpoint_t;

typedef struct
{
uint8_t  bLength; /**< Size of the descriptor, in bytes. */
uint8_t  bDescriptorType; 
uint8_t  bEndpointAddress; /**< Logical address of the endpoint within the device for the curr*/
uint8_t  bmAttributes;
uint16_t wMaxPacketSize;
uint8_t  bInterval;
} ATTR_PACKED USB_StdDescriptor_Endpoint_t;
			
template <size_t S> struct USB_Descriptor_String_t
{
    USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */
    wchar_t UnicodeString[S];
} ATTR_PACKED;

typedef struct
{
    uint8_t bLength; /**< Size of the descriptor, in bytes. */
    uint8_t bDescriptorType;
    uint16_t bString[];
} ATTR_PACKED USB_StdDescriptor_String_t;
#endif


enum USB_Interrupts_t
{
    USB_INT_VBUSTI  = 0,
    USB_INT_WAKEUPI = 2,
    USB_INT_SUSPI   = 3,
    USB_INT_EORSTI  = 4,
    USB_INT_SOFI    = 5,
    USB_INT_RXSTPI  = 6,
};

static inline void USB_INT_Disable(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
        case USB_INT_VBUSTI:
						USBCON &= ~(1 << VBUSTE);
						break;
					case USB_INT_WAKEUPI:
						UDIEN  &= ~(1 << WAKEUPE);
						break;
					case USB_INT_SUSPI:
						UDIEN  &= ~(1 << SUSPE);
						break;
					case USB_INT_EORSTI:
						UDIEN  &= ~(1 << EORSTE);
						break;
					case USB_INT_SOFI:
						UDIEN  &= ~(1 << SOFE);
						break;
					case USB_INT_RXSTPI:
						UEIENX &= ~(1 << RXSTPE);
						break;
        default:
            break;
    }
}

static inline bool USB_INT_IsEnabled(const uint8_t Interrupt)
    ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
static inline bool USB_INT_IsEnabled(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
        case USB_INT_VBUSTI:
						return (USBCON & (1 << VBUSTE));
					case USB_INT_WAKEUPI:
						return (UDIEN  & (1 << WAKEUPE));
					case USB_INT_SUSPI:
						return (UDIEN  & (1 << SUSPE));
					case USB_INT_EORSTI:
						return (UDIEN  & (1 << EORSTE));
					case USB_INT_SOFI:
						return (UDIEN  & (1 << SOFE));
					case USB_INT_RXSTPI:
						return (UEIENX & (1 << RXSTPE));
					default:
						return false;
				}
			}

static inline bool USB_INT_HasOccurred(const uint8_t Interrupt)
    ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
static inline bool USB_INT_HasOccurred(const uint8_t Interrupt)
{
    switch (Interrupt)
    {
					case USB_INT_VBUSTI:
						return (USBINT & (1 << VBUSTI));
					case USB_INT_WAKEUPI:
						return (UDINT  & (1 << WAKEUPI));
					case USB_INT_SUSPI:
						return (UDINT  & (1 << SUSPI));
					case USB_INT_EORSTI:
						return (UDINT  & (1 << EORSTI));
					case USB_INT_SOFI:
						return (UDINT  & (1 << SOFI));
					case USB_INT_RXSTPI:
						return (UEINTX & (1 << RXSTPI));
					default:
						return false;
				}
			}



typedef struct
{
    uint8_t  Address; /**< Addresto configure, or zero if the table entry is to be unused. */
    uint16_t Size; /**< Size of the endpoint bank, in bytes. */
    uint8_t  Type; /**< Type of the endpoint, a \c EP_TYPE_* mask. */
    uint8_t  Banks; /**< Number of hardware banks to use for the endpoint. */
} USB_Endpoint_Table_t;

#define ENDPOINT_EPNUM_MASK                     0x0F
#define ENDPOINT_CONTROLEP                      0

static inline uint8_t Endpoint_BytesToEPSizeMask(const uint16_t Bytes)
    ATTR_WARN_UNUSED_RESULT ATTR_CONST
                                                ATTR_ALWAYS_INLINE;

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

void Endpoint_ClearEndpoints(void);
bool Endpoint_ConfigureEndpoint_Prv(const uint8_t Number,
                                                const uint8_t UECFG0XData,
                                                const uint8_t UECFG1XData);

static constexpr uint8_t ENDPOINT_TOTAL_ENDPOINTS = 7;

enum Endpoint_WaitUntilReady_ErrorCodes_t
{
    ENDPOINT_READYWAIT_NoError = 0, /**< Endpoint is ready for next packet, no error. */
    ENDPOINT_READYWAIT_EndpointStalled = 1, /**< The endpoint was stalled during the stream*/
    ENDPOINT_READYWAIT_DeviceDisconnected  = 2,
    ENDPOINT_READYWAIT_BusSuspended = 3,
    ENDPOINT_READYWAIT_Timeout = 4,
};

static inline bool Endpoint_ConfigureEndpoint(const uint8_t Address,
                                         const uint8_t Type,
                                        const uint16_t Size,
                                    const uint8_t Banks) ATTR_ALWAYS_INLINE;

static inline bool Endpoint_ConfigureEndpoint(uint8_t Address, uint8_t Type,
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

static inline uint16_t Endpoint_BytesInEndpoint(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;

static inline uint16_t Endpoint_BytesInEndpoint(void)
{
    return (((uint16_t)UEBCHX << 8) | UEBCLX);
}

static inline uint8_t Endpoint_GetEndpointDirection(void)
    ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;

static inline uint8_t Endpoint_GetEndpointDirection(void)
{
    return (UECFG0X & (1 << EPDIR)) ? ENDPOINT_DIR_IN : ENDPOINT_DIR_OUT;
}

static inline uint8_t Endpoint_GetCurrentEndpoint(void)
ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;

static inline uint8_t Endpoint_GetCurrentEndpoint(void)
{
#if !defined(CONTROL_ONLY_DEVICE)
    return ((UENUM & ENDPOINT_EPNUM_MASK) | Endpoint_GetEndpointDirection());
#else
    return ENDPOINT_CONTROLEP;
#endif
}

static inline void Endpoint_SelectEndpoint(const uint8_t Address)
{
    UENUM = (Address & ENDPOINT_EPNUM_MASK);
}

static inline void Endpoint_ResetEndpoint(const uint8_t Address) ATTR_ALWAYS_INLINE;
static inline void Endpoint_ResetEndpoint(const uint8_t Address)
{
    UERST = (1 << (Address & ENDPOINT_EPNUM_MASK));
    UERST = 0;
}

static inline uint8_t Endpoint_GetBusyBanks(void) ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
static inline uint8_t Endpoint_GetBusyBanks(void)
{
    return (UESTA0X & (0x03 << NBUSYBK0));
}

static inline bool Endpoint_IsReadWriteAllowed(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsReadWriteAllowed(void)
{   
    return ((UEINTX & (1 << RWAL)) ? true : false);
}   

static inline bool Endpoint_IsConfigured(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsConfigured(void)
{   
    return ((UESTA0X & (1 << CFGOK)) ? true : false);
} 

static inline uint8_t Endpoint_GetEndpointInterrupts(void)
ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;

static inline uint8_t Endpoint_GetEndpointInterrupts(void)
{
    return UEINT;
}

static inline bool Endpoint_HasEndpointInterrupted(const uint8_t Address)
ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;

static inline bool Endpoint_HasEndpointInterrupted(const uint8_t Address)
{
    return ((Endpoint_GetEndpointInterrupts() &
        (1 << (Address & ENDPOINT_EPNUM_MASK))) ? true : false);
}

static inline bool Endpoint_IsINReady(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsINReady(void)
{
    return ((UEINTX & (1 << TXINI)) ? true : false);
}

static inline bool Endpoint_IsOUTReceived(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsOUTReceived(void)
{
    return ((UEINTX & (1 << RXOUTI)) ? true : false);
}

static inline bool Endpoint_IsSETUPReceived(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsSETUPReceived(void)
{
    return ((UEINTX & (1 << RXSTPI)) ? true : false);
}

static inline void Endpoint_ClearOUT(void) ATTR_ALWAYS_INLINE;
static inline void Endpoint_ClearOUT(void)
{   
    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
}

static inline bool Endpoint_IsStalled(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool Endpoint_IsStalled(void)
{
    return ((UECONX & (1 << STALLRQ)) ? true : false);
}

static inline void Endpoint_ResetDataToggle(void) ATTR_ALWAYS_INLINE;
static inline void Endpoint_ResetDataToggle(void)
{ 
    UECONX |= (1 << RSTDT);
} 

static inline void Endpoint_SetEndpointDirection(const uint8_t DirectionMask)
    ATTR_ALWAYS_INLINE;

static inline void Endpoint_SetEndpointDirection(const uint8_t DirectionMask)
{   
    UECFG0X = ((UECFG0X & ~(1 << EPDIR)) | (DirectionMask ? (1 << EPDIR) : 0));
}   

static inline uint8_t Endpoint_Read_8(void)
{   
    return UEDATX;
}

static inline void Endpoint_Write_8(const uint8_t Data) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Write_8(const uint8_t Data)
{   
    UEDATX = Data;
}

static inline void Endpoint_Discard_8(void) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Discard_8(void)
{   
    uint8_t Dummy;
    Dummy = UEDATX;
    (void)Dummy;
}   

static inline uint16_t Endpoint_Read_16_LE(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline uint16_t Endpoint_Read_16_LE(void)
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

static inline uint16_t Endpoint_Read_16_BE(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline uint16_t Endpoint_Read_16_BE(void)
{
    union
    {
        uint16_t Value;
        uint8_t  Bytes[2];
    } Data;

    Data.Bytes[1] = UEDATX;
    Data.Bytes[0] = UEDATX;
    return Data.Value;
}

static inline void Endpoint_Write_16_LE(const uint16_t Data) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Write_16_LE(const uint16_t Data)
{   
    UEDATX = (Data & 0xFF);
    UEDATX = (Data >> 8);
}

static inline void Endpoint_Write_16_BE(const uint16_t Data) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Write_16_BE(const uint16_t Data)
{   
    UEDATX = (Data >> 8);
    UEDATX = (Data & 0xFF);
}

static inline void Endpoint_Discard_16(void) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Discard_16(void)
{
    uint8_t Dummy;
    Dummy = UEDATX;
    Dummy = UEDATX;
    (void)Dummy;
}

static inline uint32_t Endpoint_Read_32_LE(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline uint32_t Endpoint_Read_32_LE(void)
{
    union
    {
        uint32_t Value;
        uint8_t  Bytes[4];
    } Data;

    Data.Bytes[0] = UEDATX;
    Data.Bytes[1] = UEDATX;
    Data.Bytes[2] = UEDATX;
    Data.Bytes[3] = UEDATX;
    return Data.Value;
}

static inline void Endpoint_Write_32_LE(const uint32_t Data) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Write_32_LE(const uint32_t Data)
{
    UEDATX = (Data &  0xFF);
    UEDATX = (Data >> 8);
    UEDATX = (Data >> 16);
    UEDATX = (Data >> 24);
}

static inline void Endpoint_Write_32_BE(const uint32_t Data) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Write_32_BE(const uint32_t Data)
{
    UEDATX = (Data >> 24);
    UEDATX = (Data >> 16);
    UEDATX = (Data >> 8);
    UEDATX = (Data &  0xFF);
}
    
static inline void Endpoint_Discard_32(void) ATTR_ALWAYS_INLINE;
static inline void Endpoint_Discard_32(void)
{       
    uint8_t Dummy;
    Dummy = UEDATX;
    Dummy = UEDATX; 
    Dummy = UEDATX; 
    Dummy = UEDATX; 
    (void)Dummy;
}

bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                                 const uint8_t Entries);


#if (defined(USE_RAM_DESCRIPTORS) && defined(USE_EEPROM_DESCRIPTORS))
#error USE_RAM_DESCRIPTORS and USE_EEPROM_DESCRIPTORS are mutually exclusive.
#endif

#if (defined(USE_FLASH_DESCRIPTORS) && defined(USE_EEPROM_DESCRIPTORS))
#error USE_FLASH_DESCRIPTORS and USE_EEPROM_DESCRIPTORS are mutually exclusive.
#endif

#if (defined(USE_FLASH_DESCRIPTORS) && defined(USE_RAM_DESCRIPTORS))
#error USE_FLASH_DESCRIPTORS and USE_RAM_DESCRIPTORS are mutually exclusive.
#endif

#define USB_DEVICE_OPT_LOWSPEED            (1 << 0)

#define USB_DEVICE_OPT_FULLSPEED               (0 << 0)

#if (!defined(NO_INTERNAL_SERIAL) && \
	     (defined(USB_SERIES_7_AVR) || defined(USB_SERIES_6_AVR) || \
	      defined(USB_SERIES_4_AVR) || defined(USB_SERIES_2_AVR) || \
		  defined(__DOXYGEN__)))

				#define USE_INTERNAL_SERIAL            0xDC

				#define INTERNAL_SERIAL_LENGTH_BITS    80

				#define INTERNAL_SERIAL_START_ADDRESS  0x0E
			#else
				#define USE_INTERNAL_SERIAL            NO_DESCRIPTOR

				#define INTERNAL_SERIAL_LENGTH_BITS    0
				#define INTERNAL_SERIAL_START_ADDRESS  0
			#endif

void USB_Device_SendRemoteWakeup(void);

static inline uint16_t USB_Device_GetFrameNumber(void)
    ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
static inline uint16_t USB_Device_GetFrameNumber(void)
{
    return UDFNUM;
}

static inline void USB_Device_SetDeviceAddress(const uint8_t Address) ATTR_ALWAYS_INLINE;
static inline void USB_Device_SetDeviceAddress(const uint8_t Address)
{
    UDADDR = (UDADDR & (1 << ADDEN)) | (Address & 0x7F);
}

static inline void USB_Device_EnableDeviceAddress(const uint8_t Address) ATTR_ALWAYS_INLINE;
static inline void USB_Device_EnableDeviceAddress(const uint8_t Address)
{
    (void)Address;
    UDADDR |= (1 << ADDEN);
}

static inline void USB_Device_GetSerialString(uint16_t* const UnicodeString)
    ATTR_NON_NULL_PTR_ARG(1);

static inline void USB_Device_GetSerialString(uint16_t* const UnicodeString)
{
    uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
    cli();

    uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

    for (uint8_t SerialCharNum = 0;
        SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4);
        SerialCharNum++)
    {
        uint8_t SerialByte = boot_signature_byte_get(SigReadAddress);

        if (SerialCharNum & 0x01)
        {
            SerialByte >>= 4;
            SigReadAddress++;
        }

        SerialByte &= 0x0F;

        UnicodeString[SerialCharNum] = cpu_to_le16((SerialByte >= 10) ?
            (('A' - 10) + SerialByte) : ('0' + SerialByte));
    }

    SetGlobalInterruptMask(CurrentGlobalInt);
}

#endif




#ifndef __STDREQTYPE_H__
#define __STDREQTYPE_H__



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

			
typedef struct
{
    uint8_t  bmRequestType; /**< Type of the request. */
    uint8_t  bRequest; /**< Request command code. */
    uint16_t wValue; /**< wValue parameter of the request. */
    uint16_t wIndex; /**< wIndex parameter of the request. */
    uint16_t wLength; /**< Length of the data to transfer in bytes. */
} ATTR_PACKED USB_Request_Header_t;

enum USB_Control_Request_t
{
REQ_GetStatus           = 0,
REQ_ClearFeature        = 1,
REQ_SetFeature          = 3,
REQ_SetAddress          = 5,
REQ_GetDescriptor       = 6, /**< Implemented in the library for device and interface recipients. Passed to the
				                              *   user application for other recipients via the
				                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_SetDescriptor       = 7, /**< Not implemented in the library, passed to the user application
				                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_GetConfiguration    = 8, /**< Implemented in the library for the device recipient. Passed
				                              *   to the user application for other recipients via the
				                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_SetConfiguration    = 9, /**< Implemented in the library for the device recipient. Passed
				                              *   to the user application for other recipients via the
				                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_GetInterface        = 10, /**< Not implemented in the library, passed to the user application
				                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_SetInterface        = 11, /**< Not implemented in the library, passed to the user application
				                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
				REQ_SynchFrame          = 12, /**< Not implemented in the library, passed to the user application
				                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
				                              *   device mode. */
			};

			/** Feature Selector values for Set Feature and Clear Feature standard control requests directed to the device, interface
			 *  and endpoint recipients.
			 */
			enum USB_Feature_Selectors_t
			{
				FEATURE_SEL_EndpointHalt       = 0x00,
				FEATURE_SEL_DeviceRemoteWakeup = 0x01, /**< Feature selector for Device level Remote Wakeup enable set or clear.
			                                            *   This feature can be controlled by the host on devices which indicate
			                                            *   remote wakeup support in their descriptors to selectively disable or
			                                            *   enable remote wakeup.
			                                            */
				FEATURE_SEL_TestMode           = 0x02, /**< Feature selector for Test Mode features, used to test the USB controller
			                                            *   to check for incorrect operation.
			                                            */
			};

		#if !defined(__DOXYGEN__)
				#define FEATURE_SELFPOWERED_ENABLED     (1 << 0)
				#define FEATURE_REMOTE_WAKEUP_ENABLED   (1 << 1)
		#endif


#endif


#if defined(ARCH_HAS_MULTI_ADDRESS_SPACE) || defined(__DOXYGEN__)
enum USB_DescriptorMemorySpaces_t
{
#if defined(ARCH_HAS_FLASH_ADDRESS_SPACE) || defined(__DOXYGEN__)
    MEMSPACE_FLASH    = 0, /**< Indiescriptor is located in FLASH memory. */
#endif
#if defined(ARCH_HAS_EEPROM_ADDRESS_SPACE) || defined(__DOXYGEN__)
    MEMSPACE_EEPROM   = 1, /**< Indicates scriptor is located in EEPROM memory. */
#endif
    MEMSPACE_RAM      = 2, /**< Indicates descriptor is located in RAM memory. */
};
#endif

extern uint8_t USB_Device_ConfigurationNumber;

#if !defined(NO_DEVICE_REMOTE_WAKEUP)
extern bool USB_Device_RemoteWakeupEnabled;
#endif

#if !defined(NO_DEVICE_SELF_POWER)
extern bool USB_Device_CurrentlySelfPowered;
#endif


enum Endpoint_Stream_RW_ErrorCodes_t
{
    ENDPOINT_RWSTREAM_NoError            = 0,
    ENDPOINT_RWSTREAM_EndpointStalled    = 1,
    ENDPOINT_RWSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWSTREAM_BusSuspended       = 3,
    ENDPOINT_RWSTREAM_Timeout            = 4,
    ENDPOINT_RWSTREAM_IncompleteTransfer = 5,
};

enum Endpoint_ControlStream_RW_ErrorCodes_t
{
    ENDPOINT_RWCSTREAM_NoError = 0, /**< Command completed successfully, no error. */
    ENDPOINT_RWCSTREAM_HostAborted = 1, /**< The aborted the transfer prematurely. */
    ENDPOINT_RWCSTREAM_DeviceDisconnected = 2,
    ENDPOINT_RWCSTREAM_BusSuspended = 3,
};



uint8_t Endpoint_Discard_Stream(uint16_t Length, uint16_t* const BytesProcessed);

uint8_t Endpoint_Write_Stream_LE(const void* const Buffer,
			                      uint16_t Length,
			                      uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

uint8_t Endpoint_Write_Stream_BE(const void* const Buffer,
			                                 uint16_t Length,
			                     uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

			
			uint8_t Endpoint_Read_Stream_LE(void* const Buffer,
			                                uint16_t Length,
		                  uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

		
			uint8_t Endpoint_Read_Stream_BE(void* const Buffer,
			                                uint16_t Length,
	                          uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);



			uint8_t Endpoint_Write_Control_Stream_BE(const void* const Buffer,
                                 uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Read_Control_Stream_LE(void* const Buffer,
                                uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Read_Control_Stream_BE(void* const Buffer,
                                 uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Write_EStream_LE(const void* const Buffer,
                            uint16_t Length,
                             uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Write_EStream_BE(const void* const Buffer,
                           uint16_t Length,
                         uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);


			uint8_t Endpoint_Read_EStream_LE(void* const Buffer,
                            uint16_t Length,
                           uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Read_EStream_BE(void* const Buffer,
                              uint16_t Length,
                              uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

	
			uint8_t Endpoint_Write_Control_EStream_LE(const void* const Buffer,
			                                          uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Write_Control_EStream_BE(const void* const Buffer,
                                        uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Read_Control_EStream_LE(void* const Buffer,
                                       uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			
			uint8_t Endpoint_Read_Control_EStream_BE(void* const Buffer,
                                       uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);
		
			uint8_t Endpoint_Write_PStream_LE(const void* const Buffer,
                                 uint16_t Length,
                                 uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

	
			uint8_t Endpoint_Write_PStream_BE(const void* const Buffer,
                    uint16_t Length,
                    uint16_t* const BytesProcessed) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Write_Control_PStream_LE(const void* const Buffer,
                           uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			uint8_t Endpoint_Write_Control_PStream_BE(const void* const Buffer,
                                uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);


#if (F_USB == 8000000)
#if (defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || \
defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
defined(__AVR_ATmega32U2__))
#define USB_PLL_PSC                0
#elif (defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__))
#define USB_PLL_PSC                0
#elif (defined(__AVR_AT90USB646__)  || defined(__AVR_AT90USB1286__))
#define USB_PLL_PSC                ((1 << PLLP1) | (1 << PLLP0))
#elif (defined(__AVR_AT90USB647__)  || defined(__AVR_AT90USB1287__))
#define USB_PLL_PSC                ((1 << PLLP1) | (1 << PLLP0))
#endif
#elif (F_USB == 16000000)
#if (defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || \
defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
defined(__AVR_ATmega32U2__))
#define USB_PLL_PSC                (1 << PLLP0)
#elif (defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__))
#define USB_PLL_PSC                (1 << PINDIV)
#elif (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__))
#define USB_PLL_PSC                ((1 << PLLP2) | (1 << PLLP1))
#elif (defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__))
#define USB_PLL_PSC                ((1 << PLLP2) | (1 << PLLP0))
#endif
#endif

#if !defined(USB_STREAM_TIMEOUT_MS) || defined(__DOXYGEN__)
#define USB_STREAM_TIMEOUT_MS       100
#endif

static inline bool USB_PLL_IsReady(void) ATTR_WARN_UNUSED_RESULT ATTR_ALWAYS_INLINE;
static inline bool USB_PLL_IsReady(void)
{
    return ((PLLCSR & (1 << PLOCK)) ? true : false);
}


#endif


#ifndef _HID_CLASS_COMMON_H_
#define _HID_CLASS_COMMON_H_

#ifndef __HIDREPORTDATA_H__
#define __HIDREPORTDATA_H__

#if !defined(__DOXYGEN__)
#define HID_RI_DATA_SIZE_MASK                   0x03
#define HID_RI_TYPE_MASK                        0x0C
#define HID_RI_TAG_MASK                         0xF0

#define HID_RI_TYPE_MAIN                        0x00
#define HID_RI_TYPE_GLOBAL                      0x04
#define HID_RI_TYPE_LOCAL                       0x08

#define HID_RI_DATA_BITS_0                      0x00
#define HID_RI_DATA_BITS_8                      0x01
#define HID_RI_DATA_BITS_16                     0x02
#define HID_RI_DATA_BITS_32                     0x03
#define HID_RI_DATA_BITS(DataBits)              CONCAT_EXPANDED(HID_RI_DATA_BITS_, DataBits)

#define _HID_RI_ENCODE_0(Data)
#define _HID_RI_ENCODE_8(Data)                  , (Data & 0xFF)
#define _HID_RI_ENCODE_16(Data)        _HID_RI_ENCODE_8(Data)  _HID_RI_ENCODE_8(Data >> 8)
#define _HID_RI_ENCODE_32(Data)        _HID_RI_ENCODE_16(Data) _HID_RI_ENCODE_16(Data >> 16)
#define _HID_RI_ENCODE(DataBits, ...) CONCAT_EXPANDED(_HID_RI_ENCODE_, DataBits(__VA_ARGS__))

			#define _HID_RI_ENTRY(Type, Tag, DataBits, ...) (Type | Tag | HID_RI_DATA_BITS(DataBits)) _HID_RI_ENCODE(DataBits, (__VA_ARGS__))
	#endif

			#define HID_IOF_CONSTANT                        (1 << 0)
			#define HID_IOF_DATA                            (0 << 0)
			#define HID_IOF_VARIABLE                        (1 << 1)
			#define HID_IOF_ARRAY                           (0 << 1)
			#define HID_IOF_RELATIVE                        (1 << 2)
			#define HID_IOF_ABSOLUTE                        (0 << 2)
			#define HID_IOF_WRAP                            (1 << 3)
			#define HID_IOF_NO_WRAP                         (0 << 3)
			#define HID_IOF_NON_LINEAR                      (1 << 4)
			#define HID_IOF_LINEAR                          (0 << 4)
			#define HID_IOF_NO_PREFERRED_STATE              (1 << 5)
			#define HID_IOF_PREFERRED_STATE                 (0 << 5)
			#define HID_IOF_NULLSTATE                       (1 << 6)
			#define HID_IOF_NO_NULL_POSITION                (0 << 6)
			#define HID_IOF_VOLATILE                        (1 << 7)
			#define HID_IOF_NON_VOLATILE                    (0 << 7)
			#define HID_IOF_BUFFERED_BYTES                  (1 << 8)
			#define HID_IOF_BITFIELD                        (0 << 8)

#define HID_RI_INPUT(DataBits, ...) _HID_RI_ENTRY(HID_RI_TYPE_MAIN  , 0x80, DataBits, __VA_ARGS__)
#define HID_RI_OUTPUT(DataBits, ...) _HID_RI_ENTRY(HID_RI_TYPE_MAIN  , 0x90, DataBits, __VA_ARGS__)
#define HID_RI_COLLECTION(DataBits, ...)        _HID_RI_ENTRY(HID_RI_TYPE_MAIN  , 0xA0, DataBits, __VA_ARGS__)
#define HID_RI_FEATURE(DataBits, ...)           _HID_RI_ENTRY(HID_RI_TYPE_MAIN  , 0xB0, DataBits, __VA_ARGS__)
#define HID_RI_END_COLLECTION(DataBits, ...)    _HID_RI_ENTRY(HID_RI_TYPE_MAIN  , 0xC0, DataBits, __VA_ARGS__)
#define HID_RI_USAGE_PAGE(DataBits, ...)        _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x00, DataBits, __VA_ARGS__)
#define HID_RI_LOGICAL_MINIMUM(DataBits, ...)   _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x10, DataBits, __VA_ARGS__)
			#define HID_RI_LOGICAL_MAXIMUM(DataBits, ...)   _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x20, DataBits, __VA_ARGS__)
			#define HID_RI_PHYSICAL_MINIMUM(DataBits, ...)  _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x30, DataBits, __VA_ARGS__)
			#define HID_RI_PHYSICAL_MAXIMUM(DataBits, ...)  _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x40, DataBits, __VA_ARGS__)
			#define HID_RI_UNIT_EXPONENT(DataBits, ...)     _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x50, DataBits, __VA_ARGS__)
			#define HID_RI_UNIT(DataBits, ...)              _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x60, DataBits, __VA_ARGS__)
			#define HID_RI_REPORT_SIZE(DataBits, ...)       _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x70, DataBits, __VA_ARGS__)
			#define HID_RI_REPORT_ID(DataBits, ...)         _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x80, DataBits, __VA_ARGS__)
			#define HID_RI_REPORT_COUNT(DataBits, ...)      _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0x90, DataBits, __VA_ARGS__)
			#define HID_RI_PUSH(DataBits, ...)              _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0xA0, DataBits, __VA_ARGS__)
			#define HID_RI_POP(DataBits, ...)               _HID_RI_ENTRY(HID_RI_TYPE_GLOBAL, 0xB0, DataBits, __VA_ARGS__)
			#define HID_RI_USAGE(DataBits, ...)             _HID_RI_ENTRY(HID_RI_TYPE_LOCAL , 0x00, DataBits, __VA_ARGS__)
			#define HID_RI_USAGE_MINIMUM(DataBits, ...)     _HID_RI_ENTRY(HID_RI_TYPE_LOCAL , 0x10, DataBits, __VA_ARGS__)
			#define HID_RI_USAGE_MAXIMUM(DataBits, ...)     _HID_RI_ENTRY(HID_RI_TYPE_LOCAL , 0x20, DataBits, __VA_ARGS__)
		//@}

/** @} */

#endif


#if !defined(HID_STATETABLE_STACK_DEPTH) || defined(__DOXYGEN__)
#define HID_STATETABLE_STACK_DEPTH    2
#endif

#if !defined(HID_USAGE_STACK_DEPTH) || defined(__DOXYGEN__)
#define HID_USAGE_STACK_DEPTH         8
#endif

#if !defined(HID_MAX_COLLECTIONS) || defined(__DOXYGEN__)
#define HID_MAX_COLLECTIONS           10
#endif

#if !defined(HID_MAX_REPORTITEMS) || defined(__DOXYGEN__)
#define HID_MAX_REPORTITEMS           20
#endif

#if !defined(HID_MAX_REPORT_IDS) || defined(__DOXYGEN__)
#define HID_MAX_REPORT_IDS            10
#endif

#define HID_ALIGN_DATA(ReportItem, Type) ((Type)(ReportItem->Value << ((8 * sizeof(Type)) - ReportItem->Attributes.BitSize)))

enum HID_Parse_ErrorCodes_t
{
    HID_PARSE_Successful = 0, /**< Successful parse of the HID report descriptor, no error. */
    HID_PARSE_HIDStackOverflow = 1,
    HID_PARSE_HIDStackUnderflow = 2,
    HID_PARSE_InsufficientReportItems = 3,
    HID_PARSE_UnexpectedEndCollection = 4,
    HID_PARSE_InsufficientCollectionPaths = 5, /**< MOLLECTIONS collections in the report. */
    HID_PARSE_UsageListOverflow           = 6, /**< TH usages listed in a row. */
    HID_PARSE_InsufficientReportIDItems   = 7, /**< eport IDs in the device. */
    HID_PARSE_NoUnfilteredReportItems     = 8, /**< iltering callback routine. */
};

typedef struct
{
    uint32_t Minimum; /**< Minimum value for the attribute. */
    uint32_t Maximum; /**< Maximum value for the attribute. */
} HID_MinMax_t;

typedef struct
{
    uint32_t Type;     /**< Unit type (refer to HID specifications for details). */
    uint8_t  Exponent; /**< Unit exponent (refer to HID specifications for details). */
} HID_Unit_t;

typedef struct
{
    uint16_t Page;  /**< Usage page of the report item. */
    uint16_t Usage; /**< Usage of the report item. */
} HID_Usage_t;

typedef struct HID_CollectionPath
{
    uint8_t                    Type;   /**< Collection type (e.g. "Generic Desktop"). */
    HID_Usage_t                Usage;  /**< Collection usage. */
    struct HID_CollectionPath* Parent; /**< Reference ection, or \c NULL if root collection. */
} HID_CollectionPath_t;

typedef struct
{
    uint8_t      BitSize;  /**< Size in bits of the report item's data. */
    HID_Usage_t  Usage;    /**< Usage of the report item. */
    HID_Unit_t   Unit;     /**< Unit type and exponent of the report item. */
    HID_MinMax_t Logical;  /**< Logical minimum and maximum of the report item. */
    HID_MinMax_t Physical; /**< Physical minimum and maximum of the report item. */
} HID_ReportItem_Attributes_t;

typedef struct
{
    uint16_t BitOffset;   /**< Bit offset in the IN, OUT or FEATURE report of the item. */
    uint8_t                     ItemType;       /**< Report item type, a valItemTypes_t. */
    uint16_t                    ItemFlags;      /**< Item data fOF_* constants. */
    uint8_t                     ReportID;       /**< Report ID tnly one report */
    HID_CollectionPath_t*       CollectionPath; /**< Collection path of the item. */
    HID_ReportItem_Attributes_t Attributes;     /**< Report item attributes. */
    uint32_t                    Value;
    uint32_t                    PreviousValue;  /**< Previous value of the report item. */
            } HID_ReportItem_t;

typedef struct
{
    uint8_t  ReportID; /**< Report ID of the report within the HID interface. */
    uint16_t ReportSizeBits[3];
} HID_ReportSizeInfo_t;

typedef struct
{   
    uint8_t TotalReportItems; /**< Total rt items stored in the \c ReportItems array. */
    HID_ReportItem_t     ReportItems[HID_MAX_REPORTITEMS];
    HID_CollectionPath_t CollectionPaths[HID_MAX_COLLECTIONS];
    uint8_t              TotalDeviceReports;
    HID_ReportSizeInfo_t ReportIDSizes[HID_MAX_REPORT_IDS];
    uint16_t             LargestReportSizeBits;
    bool                 UsingReportIDs;              
} HID_ReportInfo_t;

uint8_t USB_ProcessHIDReport(const uint8_t* ReportData,
         uint16_t ReportSize,
         HID_ReportInfo_t* const ParserData) ATTR_NON_NULL_PTR_ARG(1) ATTR_NON_NULL_PTR_ARG(3);

bool USB_GetHIDReportItemInfo(const uint8_t* ReportData,
                            HID_ReportItem_t* const ReportItem) ATTR_NON_NULL_PTR_ARG(1);


void USB_SetHIDReportItemInfo(uint8_t* ReportData,
                             HID_ReportItem_t* const ReportItem) ATTR_NON_NULL_PTR_ARG(1);


uint16_t USB_GetHIDReportSize(HID_ReportInfo_t* const ParserData,
                              const uint8_t ReportID,
                              const uint8_t ReportType) ATTR_CONST ATTR_NON_NULL_PTR_ARG(1);


bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem);

typedef struct
{
    HID_ReportItem_Attributes_t Attributes;
    uint8_t                     ReportCount;
    uint8_t                     ReportID;
} HID_StateTable_t;


#define HID_KEYBOARD_MODIFIER_LEFTCTRL                    (1 << 0)
#define HID_KEYBOARD_MODIFIER_LEFTSHIFT                   (1 << 1)
#define HID_KEYBOARD_MODIFIER_LEFTALT                     (1 << 2)
#define HID_KEYBOARD_MODIFIER_LEFTGUI                     (1 << 3)
#define HID_KEYBOARD_MODIFIER_RIGHTCTRL                   (1 << 4)
#define HID_KEYBOARD_MODIFIER_RIGHTSHIFT                  (1 << 5)
#define HID_KEYBOARD_MODIFIER_RIGHTALT                    (1 << 6)
#define HID_KEYBOARD_MODIFIER_RIGHTGUI                    (1 << 7)
#define HID_KEYBOARD_LED_NUMLOCK                          (1 << 0)
#define HID_KEYBOARD_LED_CAPSLOCK                         (1 << 1)
#define HID_KEYBOARD_LED_SCROLLLOCK                       (1 << 2)
#define HID_KEYBOARD_LED_COMPOSE                          (1 << 3)
#define HID_KEYBOARD_LED_KANA                             (1 << 4)

#define HID_KEYBOARD_SC_ERROR_ROLLOVER                    0x01
#define HID_KEYBOARD_SC_POST_FAIL                         0x02
#define HID_KEYBOARD_SC_ERROR_UNDEFINED                   0x03
#define HID_KEYBOARD_SC_A                                 0x04
#define HID_KEYBOARD_SC_B                                 0x05
#define HID_KEYBOARD_SC_C                                 0x06
#define HID_KEYBOARD_SC_D                                 0x07
#define HID_KEYBOARD_SC_E                                 0x08
#define HID_KEYBOARD_SC_F                                 0x09
#define HID_KEYBOARD_SC_G                                 0x0A
#define HID_KEYBOARD_SC_H                                 0x0B
#define HID_KEYBOARD_SC_I                                 0x0C
#define HID_KEYBOARD_SC_J                                 0x0D
#define HID_KEYBOARD_SC_K                                 0x0E
#define HID_KEYBOARD_SC_L                                 0x0F
#define HID_KEYBOARD_SC_M                                 0x10
#define HID_KEYBOARD_SC_N                                 0x11
#define HID_KEYBOARD_SC_O                                 0x12
#define HID_KEYBOARD_SC_P                                 0x13
#define HID_KEYBOARD_SC_Q                                 0x14
#define HID_KEYBOARD_SC_R                                 0x15
#define HID_KEYBOARD_SC_S                                 0x16
#define HID_KEYBOARD_SC_T                                 0x17
#define HID_KEYBOARD_SC_U                                 0x18
#define HID_KEYBOARD_SC_V                                 0x19
#define HID_KEYBOARD_SC_W                                 0x1A
#define HID_KEYBOARD_SC_X                                 0x1B
#define HID_KEYBOARD_SC_Y                                 0x1C
#define HID_KEYBOARD_SC_Z                                 0x1D
#define HID_KEYBOARD_SC_1_AND_EXCLAMATION                 0x1E
#define HID_KEYBOARD_SC_2_AND_AT                          0x1F
#define HID_KEYBOARD_SC_3_AND_HASHMARK                    0x20
#define HID_KEYBOARD_SC_4_AND_DOLLAR                      0x21
#define HID_KEYBOARD_SC_5_AND_PERCENTAGE                  0x22
#define HID_KEYBOARD_SC_6_AND_CARET                       0x23
#define HID_KEYBOARD_SC_7_AND_AMPERSAND                   0x24
#define HID_KEYBOARD_SC_8_AND_ASTERISK                    0x25
#define HID_KEYBOARD_SC_9_AND_OPENING_PARENTHESIS         0x26
#define HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS         0x27
#define HID_KEYBOARD_SC_ENTER                             0x28
#define HID_KEYBOARD_SC_ESCAPE                            0x29
#define HID_KEYBOARD_SC_BACKSPACE                         0x2A
#define HID_KEYBOARD_SC_TAB                               0x2B
#define HID_KEYBOARD_SC_SPACE                             0x2C
#define HID_KEYBOARD_SC_MINUS_AND_UNDERSCORE              0x2D
#define HID_KEYBOARD_SC_EQUAL_AND_PLUS                    0x2E
#define HID_KEYBOARD_SC_OPENING_BRACKET_AND_OPENING_BRACE 0x2F
#define HID_KEYBOARD_SC_CLOSING_BRACKET_AND_CLOSING_BRACE 0x30
#define HID_KEYBOARD_SC_BACKSLASH_AND_PIPE                0x31
#define HID_KEYBOARD_SC_NON_US_HASHMARK_AND_TILDE         0x32
#define HID_KEYBOARD_SC_SEMICOLON_AND_COLON               0x33
#define HID_KEYBOARD_SC_APOSTROPHE_AND_QUOTE              0x34
#define HID_KEYBOARD_SC_GRAVE_ACCENT_AND_TILDE            0x35
#define HID_KEYBOARD_SC_COMMA_AND_LESS_THAN_SIGN          0x36
#define HID_KEYBOARD_SC_DOT_AND_GREATER_THAN_SIGN         0x37
#define HID_KEYBOARD_SC_SLASH_AND_QUESTION_MARK           0x38
#define HID_KEYBOARD_SC_CAPS_LOCK                         0x39
#define HID_KEYBOARD_SC_F1                                0x3A
#define HID_KEYBOARD_SC_F2                                0x3B
#define HID_KEYBOARD_SC_F3                                0x3C
#define HID_KEYBOARD_SC_F4                                0x3D
#define HID_KEYBOARD_SC_F5                                0x3E
#define HID_KEYBOARD_SC_F6                                0x3F
#define HID_KEYBOARD_SC_F7                                0x40
#define HID_KEYBOARD_SC_F8                                0x41
#define HID_KEYBOARD_SC_F9                                0x42
#define HID_KEYBOARD_SC_F10                               0x43
#define HID_KEYBOARD_SC_F11                               0x44
#define HID_KEYBOARD_SC_F12                               0x45
#define HID_KEYBOARD_SC_PRINT_SCREEN                      0x46
#define HID_KEYBOARD_SC_SCROLL_LOCK                       0x47
#define HID_KEYBOARD_SC_PAUSE                             0x48
#define HID_KEYBOARD_SC_INSERT                            0x49
#define HID_KEYBOARD_SC_HOME                              0x4A
#define HID_KEYBOARD_SC_PAGE_UP                           0x4B
#define HID_KEYBOARD_SC_DELETE                            0x4C
#define HID_KEYBOARD_SC_END                               0x4D
#define HID_KEYBOARD_SC_PAGE_DOWN                         0x4E
#define HID_KEYBOARD_SC_RIGHT_ARROW                       0x4F
#define HID_KEYBOARD_SC_LEFT_ARROW                        0x50
#define HID_KEYBOARD_SC_DOWN_ARROW                        0x51
#define HID_KEYBOARD_SC_UP_ARROW                          0x52
#define HID_KEYBOARD_SC_NUM_LOCK                          0x53
#define HID_KEYBOARD_SC_KEYPAD_SLASH                      0x54
#define HID_KEYBOARD_SC_KEYPAD_ASTERISK                   0x55
#define HID_KEYBOARD_SC_KEYPAD_MINUS                      0x56
#define HID_KEYBOARD_SC_KEYPAD_PLUS                       0x57
#define HID_KEYBOARD_SC_KEYPAD_ENTER                      0x58
#define HID_KEYBOARD_SC_KEYPAD_1_AND_END                  0x59
#define HID_KEYBOARD_SC_KEYPAD_2_AND_DOWN_ARROW           0x5A
#define HID_KEYBOARD_SC_KEYPAD_3_AND_PAGE_DOWN            0x5B
#define HID_KEYBOARD_SC_KEYPAD_4_AND_LEFT_ARROW           0x5C
#define HID_KEYBOARD_SC_KEYPAD_5                          0x5D
#define HID_KEYBOARD_SC_KEYPAD_6_AND_RIGHT_ARROW          0x5E
#define HID_KEYBOARD_SC_KEYPAD_7_AND_HOME                 0x5F
#define HID_KEYBOARD_SC_KEYPAD_8_AND_UP_ARROW             0x60
#define HID_KEYBOARD_SC_KEYPAD_9_AND_PAGE_UP              0x61
#define HID_KEYBOARD_SC_KEYPAD_0_AND_INSERT               0x62
#define HID_KEYBOARD_SC_KEYPAD_DOT_AND_DELETE             0x63
#define HID_KEYBOARD_SC_NON_US_BACKSLASH_AND_PIPE         0x64
#define HID_KEYBOARD_SC_APPLICATION                       0x65
#define HID_KEYBOARD_SC_POWER                             0x66
#define HID_KEYBOARD_SC_KEYPAD_EQUAL_SIGN                 0x67
#define HID_KEYBOARD_SC_F13                               0x68
#define HID_KEYBOARD_SC_F14                               0x69
#define HID_KEYBOARD_SC_F15                               0x6A
#define HID_KEYBOARD_SC_F16                               0x6B
#define HID_KEYBOARD_SC_F17                               0x6C
#define HID_KEYBOARD_SC_F18                               0x6D
#define HID_KEYBOARD_SC_F19                               0x6E
#define HID_KEYBOARD_SC_F20                               0x6F
#define HID_KEYBOARD_SC_F21                               0x70
#define HID_KEYBOARD_SC_F22                               0x71
#define HID_KEYBOARD_SC_F23                               0x72
#define HID_KEYBOARD_SC_F24                               0x73
#define HID_KEYBOARD_SC_EXECUTE                           0x74
#define HID_KEYBOARD_SC_HELP                              0x75
#define HID_KEYBOARD_SC_MENU                              0x76
#define HID_KEYBOARD_SC_SELECT                            0x77
#define HID_KEYBOARD_SC_STOP                              0x78
#define HID_KEYBOARD_SC_AGAIN                             0x79
#define HID_KEYBOARD_SC_UNDO                              0x7A
#define HID_KEYBOARD_SC_CUT                               0x7B
#define HID_KEYBOARD_SC_COPY                              0x7C
#define HID_KEYBOARD_SC_PASTE                             0x7D
#define HID_KEYBOARD_SC_FIND                              0x7E
#define HID_KEYBOARD_SC_MUTE                              0x7F
#define HID_KEYBOARD_SC_VOLUME_UP                         0x80
#define HID_KEYBOARD_SC_VOLUME_DOWN                       0x81
#define HID_KEYBOARD_SC_LOCKING_CAPS_LOCK                 0x82
#define HID_KEYBOARD_SC_LOCKING_NUM_LOCK                  0x83
#define HID_KEYBOARD_SC_LOCKING_SCROLL_LOCK               0x84
#define HID_KEYBOARD_SC_KEYPAD_COMMA                      0x85
#define HID_KEYBOARD_SC_KEYPAD_EQUAL_SIGN_AS400           0x86
#define HID_KEYBOARD_SC_INTERNATIONAL1                    0x87
#define HID_KEYBOARD_SC_INTERNATIONAL2                    0x88
#define HID_KEYBOARD_SC_INTERNATIONAL3                    0x89
#define HID_KEYBOARD_SC_INTERNATIONAL4                    0x8A
#define HID_KEYBOARD_SC_INTERNATIONAL5                    0x8B
#define HID_KEYBOARD_SC_INTERNATIONAL6                    0x8C
#define HID_KEYBOARD_SC_INTERNATIONAL7                    0x8D
#define HID_KEYBOARD_SC_INTERNATIONAL8                    0x8E
#define HID_KEYBOARD_SC_INTERNATIONAL9                    0x8F
#define HID_KEYBOARD_SC_LANG1                             0x90
#define HID_KEYBOARD_SC_LANG2                             0x91
#define HID_KEYBOARD_SC_LANG3                             0x92
#define HID_KEYBOARD_SC_LANG4                             0x93
#define HID_KEYBOARD_SC_LANG5                             0x94
#define HID_KEYBOARD_SC_LANG6                             0x95
#define HID_KEYBOARD_SC_LANG7                             0x96
#define HID_KEYBOARD_SC_LANG8                             0x97
#define HID_KEYBOARD_SC_LANG9                             0x98
#define HID_KEYBOARD_SC_ALTERNATE_ERASE                   0x99
#define HID_KEYBOARD_SC_SYSREQ                            0x9A
#define HID_KEYBOARD_SC_CANCEL                            0x9B
#define HID_KEYBOARD_SC_CLEAR                             0x9C
#define HID_KEYBOARD_SC_PRIOR                             0x9D
#define HID_KEYBOARD_SC_RETURN                            0x9E
#define HID_KEYBOARD_SC_SEPARATOR                         0x9F
#define HID_KEYBOARD_SC_OUT                               0xA0
#define HID_KEYBOARD_SC_OPER                              0xA1
#define HID_KEYBOARD_SC_CLEAR_AND_AGAIN                   0xA2
#define HID_KEYBOARD_SC_CRSEL_AND_PROPS                   0xA3
#define HID_KEYBOARD_SC_EXSEL                             0xA4
#define HID_KEYBOARD_SC_KEYPAD_00                         0xB0
#define HID_KEYBOARD_SC_KEYPAD_000                        0xB1
#define HID_KEYBOARD_SC_THOUSANDS_SEPARATOR               0xB2
#define HID_KEYBOARD_SC_DECIMAL_SEPARATOR                 0xB3
#define HID_KEYBOARD_SC_CURRENCY_UNIT                     0xB4
#define HID_KEYBOARD_SC_CURRENCY_SUB_UNIT                 0xB5
#define HID_KEYBOARD_SC_KEYPAD_OPENING_PARENTHESIS        0xB6
#define HID_KEYBOARD_SC_KEYPAD_CLOSING_PARENTHESIS        0xB7
#define HID_KEYBOARD_SC_KEYPAD_OPENING_BRACE              0xB8
#define HID_KEYBOARD_SC_KEYPAD_CLOSING_BRACE              0xB9
#define HID_KEYBOARD_SC_KEYPAD_TAB                        0xBA
#define HID_KEYBOARD_SC_KEYPAD_BACKSPACE                  0xBB
#define HID_KEYBOARD_SC_KEYPAD_A                          0xBC
#define HID_KEYBOARD_SC_KEYPAD_B                          0xBD
#define HID_KEYBOARD_SC_KEYPAD_C                          0xBE
#define HID_KEYBOARD_SC_KEYPAD_D                          0xBF
#define HID_KEYBOARD_SC_KEYPAD_E                          0xC0
#define HID_KEYBOARD_SC_KEYPAD_F                          0xC1
#define HID_KEYBOARD_SC_KEYPAD_XOR                        0xC2
#define HID_KEYBOARD_SC_KEYPAD_CARET                      0xC3
#define HID_KEYBOARD_SC_KEYPAD_PERCENTAGE                 0xC4
#define HID_KEYBOARD_SC_KEYPAD_LESS_THAN_SIGN             0xC5
#define HID_KEYBOARD_SC_KEYPAD_GREATER_THAN_SIGN          0xC6
#define HID_KEYBOARD_SC_KEYPAD_AMP                        0xC7
#define HID_KEYBOARD_SC_KEYPAD_AMP_AMP                    0xC8
#define HID_KEYBOARD_SC_KEYPAD_PIPE                       0xC9
#define HID_KEYBOARD_SC_KEYPAD_PIPE_PIPE                  0xCA
#define HID_KEYBOARD_SC_KEYPAD_COLON                      0xCB
#define HID_KEYBOARD_SC_KEYPAD_HASHMARK                   0xCC
#define HID_KEYBOARD_SC_KEYPAD_SPACE                      0xCD
#define HID_KEYBOARD_SC_KEYPAD_AT                         0xCE
#define HID_KEYBOARD_SC_KEYPAD_EXCLAMATION_SIGN           0xCF
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_STORE               0xD0
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_RECALL              0xD1
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_CLEAR               0xD2
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_ADD                 0xD3
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_SUBTRACT            0xD4
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_MULTIPLY            0xD5
#define HID_KEYBOARD_SC_KEYPAD_MEMORY_DIVIDE              0xD6
#define HID_KEYBOARD_SC_KEYPAD_PLUS_AND_MINUS             0xD7
#define HID_KEYBOARD_SC_KEYPAD_CLEAR                      0xD8
#define HID_KEYBOARD_SC_KEYPAD_CLEAR_ENTRY                0xD9
#define HID_KEYBOARD_SC_KEYPAD_BINARY                     0xDA
#define HID_KEYBOARD_SC_KEYPAD_OCTAL                      0xDB
#define HID_KEYBOARD_SC_KEYPAD_DECIMAL                    0xDC
#define HID_KEYBOARD_SC_KEYPAD_HEXADECIMAL                0xDD
#define HID_KEYBOARD_SC_LEFT_CONTROL                      0xE0
#define HID_KEYBOARD_SC_LEFT_SHIFT                        0xE1
#define HID_KEYBOARD_SC_LEFT_ALT                          0xE2
#define HID_KEYBOARD_SC_LEFT_GUI                          0xE3
#define HID_KEYBOARD_SC_RIGHT_CONTROL                     0xE4
#define HID_KEYBOARD_SC_RIGHT_SHIFT                       0xE5
#define HID_KEYBOARD_SC_RIGHT_ALT                         0xE6
#define HID_KEYBOARD_SC_RIGHT_GUI                         0xE7
#define HID_KEYBOARD_SC_MEDIA_PLAY                        0xE8
#define HID_KEYBOARD_SC_MEDIA_STOP                        0xE9
#define HID_KEYBOARD_SC_MEDIA_PREVIOUS_TRACK              0xEA
#define HID_KEYBOARD_SC_MEDIA_NEXT_TRACK                  0xEB
#define HID_KEYBOARD_SC_MEDIA_EJECT                       0xEC
#define HID_KEYBOARD_SC_MEDIA_VOLUME_UP                   0xED
#define HID_KEYBOARD_SC_MEDIA_VOLUME_DOWN                 0xEE
#define HID_KEYBOARD_SC_MEDIA_MUTE                        0xEF
#define HID_KEYBOARD_SC_MEDIA_WWW                         0xF0
#define HID_KEYBOARD_SC_MEDIA_BACKWARD                    0xF1
#define HID_KEYBOARD_SC_MEDIA_FORWARD                     0xF2
#define HID_KEYBOARD_SC_MEDIA_CANCEL                      0xF3
#define HID_KEYBOARD_SC_MEDIA_SEARCH                      0xF4
#define HID_KEYBOARD_SC_MEDIA_SLEEP                       0xF8
#define HID_KEYBOARD_SC_MEDIA_LOCK                        0xF9
#define HID_KEYBOARD_SC_MEDIA_RELOAD                      0xFA
#define HID_KEYBOARD_SC_MEDIA_CALCULATOR                  0xFB

enum HID_Descriptor_ClassSubclassProtocol_t
{
    HID_CSCP_HIDClass             = 0x03,
    HID_CSCP_NonBootSubclass      = 0x00,
    HID_CSCP_BootSubclass         = 0x01,
         HID_CSCP_NonBootProtocol      = 0x00,
			HID_CSCP_KeyboardBootProtocol = 0x01, /**< Descriptor Protocol value indicating that the device or interface
			                                       *   belongs to the Keyboard HID boot protocol.
			                                       */
			HID_CSCP_MouseBootProtocol    = 0x02, /**< Descriptor Protocol value indicating that the device or interface
			                                       *   belongs to the Mouse HID boot protocol.
			                                       */
		};

		enum HID_ClassRequests_t
		{
			HID_REQ_GetReport       = 0x01, /**< HID class-specific Request to get the current HID report from the device. */
			HID_REQ_GetIdle         = 0x02, /**< HID class-specific Request to get the current device idle count. */
			HID_REQ_GetProtocol     = 0x03, /**< HID class-specific Request to get the current HID report protocol mode. */
			HID_REQ_SetReport       = 0x09, /**< HID class-specific Request to set the current HID report to the device. */
			HID_REQ_SetIdle         = 0x0A, /**< HID class-specific Request to set the device's idle count. */
			HID_REQ_SetProtocol     = 0x0B, /**< HID class-specific Request to set the current HID report protocol mode. */
		};

		/** Enum for the HID class specific descriptor types. */
		enum HID_DescriptorTypes_t
		{
			HID_DTYPE_HID           = 0x21, /**< Descriptor header type value, to indicate a HID class HID descriptor. */
			HID_DTYPE_Report        = 0x22, /**< Descriptor header type value, to indicate a HID class HID report descriptor. */
		};

		/** Enum for the different types of HID reports. */
		enum HID_ReportItemTypes_t
		{
			HID_REPORT_ITEM_In      = 0, /**< Indicates that the item is an IN report type. */
			HID_REPORT_ITEM_Out     = 1, /**< Indicates that the item is an OUT report type. */
			HID_REPORT_ITEM_Feature = 2, /**< Indicates that the item is a FEATURE report type. */
		};

		typedef struct
		{
			USB_Descriptor_Header_t Header; /**< Regular  descriptor's type and length. */

			uint16_t                HIDSpec;
            uint8_t CountryCode; /**< Countcalized device, or zero if universal. */

			uint8_t                 TotalReportDescriptors; /**< Total number of HID report descriptors for the interface. */

			uint8_t                 HIDReportType; /**< Type of HID report, set to \ref HID_DTYPE_Report. */
			uint16_t                HIDReportLength; /**< Length of the associated HID report descriptor, in bytes. */
		} ATTR_PACKED USB_HID_Descriptor_HID_t;

		
		typedef struct
		{
			uint8_t  bLength; /**< Size of the descriptor, in bytes. */
			uint8_t  bDescriptorType;

			uint16_t bcdHID; /**< BCD encoded version that the HID descriptor and device complies to.
			                  *
			                  *   \see \ref VERSION_BCD() utility macro.
			                  */
			uint8_t  bCountryCode; /**< Country code of the localized device, or zero if universal. */

			uint8_t  bNumDescriptors; /**< Total number of HID report descriptors for the interface. */

			uint8_t  bDescriptorType2; /**< Type of HID report, set to \ref HID_DTYPE_Report. */
			uint16_t wDescriptorLength; /**< Length of the associated HID report descriptor, in bytes. */
		} ATTR_PACKED USB_HID_StdDescriptor_HID_t;

		typedef struct
		{
			uint8_t Button; /**< Button mask for currently pressed buttons in the mouse. */
			int8_t  X; /**< Current delta X movement of the mouse. */
			int8_t  Y; /**< Current delta Y movement on the mouse. */
		} ATTR_PACKED USB_MouseReport_Data_t;

		typedef struct
		{
			uint8_t Modifier; /**< Keyboard modifier byte, indicating pressed modifier keys (a combination of
			                   *   \c HID_KEYBOARD_MODIFER_* masks).
			                   */
			uint8_t Reserved; /**< Reserved for OEM use, always set to 0. */
			uint8_t KeyCode[6]; /**< Key codes of the currently pressed keys. */
		} ATTR_PACKED USB_KeyboardReport_Data_t;

		/** Type define for the data type used to store HID report descriptor elements. */
		typedef uint8_t USB_Descriptor_HIDReport_Datatype_t;


#endif



#define DESCRIPTOR_PCAST(DescriptorPtr, Type) ((Type*)(DescriptorPtr))
#define DESCRIPTOR_CAST(DescriptorPtr, Type)  (*DESCRIPTOR_PCAST(DescriptorPtr, Type))

#define DESCRIPTOR_TYPE(DescriptorPtr)    DESCRIPTOR_PCAST(DescriptorPtr, USB_Descriptor_Header_t)->Type

#define DESCRIPTOR_SIZE(DescriptorPtr)    DESCRIPTOR_PCAST(DescriptorPtr, USB_Descriptor_Header_t)->Size

typedef uint8_t (* ConfigComparatorPtr_t)(void*);

enum USB_Host_GetConfigDescriptor_ErrorCodes_t
{
    HOST_GETCONFIG_Successful = 0,
    HOST_GETCONFIG_DeviceDisconnect = 1,
    HOST_GETCONFIG_PipeError        = 2,
    HOST_GETCONFIG_SetupStalled     = 3,
    HOST_GETCONFIG_SoftwareTimeOut  = 4, /**< The request or data transfer timed out. */
    HOST_GETCONFIG_BuffOverflow     = 5,
    HOST_GETCONFIG_InvalidData      = 6,
};

enum DSearch_Return_ErrorCodes_t
{
    DESCRIPTOR_SEARCH_Found = 0,
    DESCRIPTOR_SEARCH_Fail = 1,
    DESCRIPTOR_SEARCH_NotFound = 2,
};

enum DSearch_Comp_Return_ErrorCodes_t
{
    DESCRIPTOR_SEARCH_COMP_Found = 0,
    DESCRIPTOR_SEARCH_COMP_Fail = 1,
    DESCRIPTOR_SEARCH_COMP_EndOfDescriptor = 2,
};

uint8_t USB_Host_GetDeviceConfigDescriptor(const uint8_t ConfigNumber,
                                        uint16_t* const ConfigSizePtr,
                                        void* const BufferPtr,
                const uint16_t BufferSize) ATTR_NON_NULL_PTR_ARG(2) ATTR_NON_NULL_PTR_ARG(3);




typedef struct
{
    struct
    {
        uint8_t  InterfaceNumber;
        USB_Endpoint_Table_t ReportINEndpoint;
        void*    PrevReportINBuffer;
        uint8_t  PrevReportINBufferSize;
    } Config;

    struct
    {
        bool UsingReportProtocol;
        uint16_t PrevFrameNum;
        uint16_t IdleCount;
        uint16_t IdleMSRemaining;
    } State;
} USB_ClassInfo_HID_Device_t;

bool HID_Device_ConfigureEndpoints(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo)
    ATTR_NON_NULL_PTR_ARG(1);


void HID_Device_ProcessControlRequest(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo)
    ATTR_NON_NULL_PTR_ARG(1);


void HID_Device_USBTask(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo)
    ATTR_NON_NULL_PTR_ARG(1);

bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                        void* ReportData,
                                        uint16_t* const ReportSize) ATTR_NON_NULL_PTR_ARG(1)
                 ATTR_NON_NULL_PTR_ARG(2) ATTR_NON_NULL_PTR_ARG(4) ATTR_NON_NULL_PTR_ARG(5);


void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                       const uint8_t ReportID,
                                       const uint8_t ReportType,
                                       const void* ReportData,
               const uint16_t ReportSize) ATTR_NON_NULL_PTR_ARG(1) ATTR_NON_NULL_PTR_ARG(4);

static inline void HID_Device_MillisecondElapsed(
    USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo)
    ATTR_ALWAYS_INLINE ATTR_NON_NULL_PTR_ARG(1);

static inline void HID_Device_MillisecondElapsed(USB_ClassInfo_HID_Device_t*
    const HIDInterfaceInfo)
{
    if (HIDInterfaceInfo->State.IdleMSRemaining)
        HIDInterfaceInfo->State.IdleMSRemaining--;
}





typedef struct
{
    USB_Descriptor_Configuration_Header_t Config;
    USB_Descriptor_Interface_t            HID_Interface;
    USB_HID_Descriptor_HID_t              HID_KeyboardHID;
    USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;
    USB_Descriptor_Endpoint_t             HID_ReportOUTEndpoint;
} USB_Descriptor_Configuration_t;

enum InterfaceDescriptors_t
{
    INTERFACE_ID_Keyboard = 0, /**< Keyboard interface descriptor ID */
};

enum StringDescriptors_t
{
    STRING_ID_Language     = 0, /**< Supported Languages string descriptor ID (must be zero) */
    STRING_ID_Manufacturer = 1, /**< Manufacturer string ID */
    STRING_ID_Product      = 2, /**< Product string ID */
};

#define KEYBOARD_IN_EPADDR        (ENDPOINT_DIR_IN  | 1)
#define KEYBOARD_OUT_EPADDR       (ENDPOINT_DIR_OUT | 2)

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                     const uint16_t wIndex, const void** const DescriptorAddress)
                                            ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_StartOfFrame(void);
void CreateKeyboardReport(USB_KeyboardReport_Data_t* const ReportData);

static uint16_t IdleCount = 500;
static uint16_t IdleMSRemaining = 0;
volatile bool USB_IsInitialized;

class USBKB
{
private:
    USB_Request_Header_t USB_ControlRequest;
    volatile uint8_t USB_DeviceState;
    uint8_t Endpoint_Write_Stream_LE(const void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Stream_LE (void * const buf, uint16_t len, uint16_t* const bytes);
    uint8_t Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length);
    uint8_t Endpoint_WaitUntilReady();
    uint8_t Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length);
    void USB_Device_GetDescriptor();
    void USB_Device_ProcessControlRequest();
    void Endpoint_ClearStatusStage();
    void Device_ClearSetFeature();
    uint8_t Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length);
    void USB_Device_GetInternalSerialDescriptor();
public:
    static USBKB *instance;
    void com();
    void gen();
    USBKB();
    void USBTask();
    void hidtask();
};

USBKB *USBKB::instance;

void USBKB::USBTask()
{
    uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

    if (UEINTX & 1<<RXSTPI)     // is setup received?
        USB_Device_ProcessControlRequest();

    Endpoint_SelectEndpoint(PrevEndpoint);
}

uint16_t USB_GetHIDReportSize(HID_ReportInfo_t* const ParserData,
                              const uint8_t ReportID,
                              const uint8_t ReportType)
{
    for (uint8_t i = 0; i < HID_MAX_REPORT_IDS; i++)
    {
        uint16_t ReportSizeBits = ParserData->ReportIDSizes[i].ReportSizeBits[ReportType];

        if (ParserData->ReportIDSizes[i].ReportID == ReportID)
            return (ReportSizeBits / 8) + ((ReportSizeBits % 8) ? 1 : 0);
    }

    return 0;
}


void USB_SetHIDReportItemInfo(uint8_t* ReportData, HID_ReportItem_t* const ReportItem)
{
    if (ReportItem == NULL)
        return;

    uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
    uint16_t CurrentBit   = ReportItem->BitOffset;
    uint32_t BitMask      = (1 << 0);

    if (ReportItem->ReportID)
    {
        ReportData[0] = ReportItem->ReportID;
        ReportData++;
    }

    ReportItem->PreviousValue = ReportItem->Value;

    while (DataBitsRem--)
    {
        if (ReportItem->Value & BitMask)
            ReportData[CurrentBit / 8] |= (1 << (CurrentBit % 8));

        CurrentBit++;
        BitMask <<= 1;
    }
}


void USBKB::hidtask()
{
    if (USB_DeviceState != DEVICE_STATE_Configured)
        return;

    static USB_KeyboardReport_Data_t PrevKeyboardReportData;
    USB_KeyboardReport_Data_t        KeyboardReportData;
    bool                             SendReport = false;

    CreateKeyboardReport(&KeyboardReportData);

    if (IdleCount && (!(IdleMSRemaining)))
    {
        IdleMSRemaining = IdleCount;
        SendReport = true;
    }
    else
    {
        SendReport = (memcmp(&PrevKeyboardReportData, &KeyboardReportData,
            sizeof(USB_KeyboardReport_Data_t)) != 0);
    }

    Endpoint_SelectEndpoint(KEYBOARD_IN_EPADDR);

    if (Endpoint_IsReadWriteAllowed() && SendReport)
    {
        PrevKeyboardReportData = KeyboardReportData;
        Endpoint_Write_Stream_LE(&KeyboardReportData, sizeof(KeyboardReportData), NULL);
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
    }

    Endpoint_SelectEndpoint(KEYBOARD_OUT_EPADDR);

    if (Endpoint_IsOUTReceived())
    {
        if (Endpoint_IsReadWriteAllowed())
        {
            (void)Endpoint_Read_8();
        }

        UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
    }
}

USBKB::USBKB()
{
    instance = this;
	clock_prescale_set(clock_div_2);
    UHWCON |= 1<<UVREGE;
    USB_IsInitialized = true;
    USBCON &= ~(1<<VBUSTE);
    UDIEN = 0;
    USBINT = 0;
    UDINT = 0;
    USBCON &= ~(1<<USBE);       // disable usb controller
    USBCON |= 1<<USBE;          // enable usb controller
    USBCON &= ~(1<<FRZCLK);
    PLLCSR = 0;
    USB_DeviceState                 = DEVICE_STATE_Unattached;
    USB_Device_ConfigurationNumber  = 0;
    USB_Device_RemoteWakeupEnabled  = false;
    USB_Device_CurrentlySelfPowered = false;
    UDCON &= ~(1<<LSM);         // full speed
    USBCON |= 1<<VBUSTE;        // enable vbus int
    Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, 8, 1);
    UDINT &= ~(1<<SUSPI);
    UDIEN |= 1<<SUSPE;
    UDIEN |= 1<<EORSTE;
    UDCON &= ~(1<<DETACH);
    USBCON |= 1<<OTGPADE;
	sei();
}

int main(void)
{
    DDRF &= ~(1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    PORTF |= 1<<0 | 1<<1 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    USBKB kb;

	while (true)
	{
        kb.hidtask();
        kb.USBTask();
	}
}

void USBKB::Endpoint_ClearStatusStage()
{
    if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
    {
        while (!(Endpoint_IsOUTReceived()))
            if (USB_DeviceState == DEVICE_STATE_Unattached)
                return;

        Endpoint_ClearOUT();
    }
    else
    {
        while (!(Endpoint_IsINReady()))
        {
            if (USB_DeviceState == DEVICE_STATE_Unattached)
              return;
        }

        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
    }
}

uint8_t USBKB::Endpoint_WaitUntilReady()
{
    uint8_t  TimeoutMSRem = USB_STREAM_TIMEOUT_MS;
    uint16_t PreviousFrameNumber = USB_Device_GetFrameNumber();

    while (true)
    {
        if (Endpoint_GetEndpointDirection() == ENDPOINT_DIR_IN)
        {
            if (Endpoint_IsINReady())
                return ENDPOINT_READYWAIT_NoError;
        }
        else
        {
            if (Endpoint_IsOUTReceived())
                return ENDPOINT_READYWAIT_NoError;
        }

        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
            return ENDPOINT_READYWAIT_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
            return ENDPOINT_READYWAIT_BusSuspended;
        else if (Endpoint_IsStalled())
            return ENDPOINT_READYWAIT_EndpointStalled;

        uint16_t CurrentFrameNumber = USB_Device_GetFrameNumber();

        if (CurrentFrameNumber != PreviousFrameNumber)
        {
            PreviousFrameNumber = CurrentFrameNumber;

            if (!(TimeoutMSRem--))
                return ENDPOINT_READYWAIT_Timeout;
        }
    }
}

bool Endpoint_ConfigureEndpoint_Prv(const uint8_t Number,
                                    const uint8_t UECFG0XData,
                                    const uint8_t UECFG1XData)
{
    for (uint8_t EPNum = Number; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        uint8_t UECFG0XTemp;
        uint8_t UECFG1XTemp;
        uint8_t UEIENXTemp;

        Endpoint_SelectEndpoint(EPNum);

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

        if (!(UECFG1XTemp & 1<<ALLOC))
            continue;

        UECONX &= ~(1<<EPEN);
        UECFG1X &= ~(1<<ALLOC);
        UECONX |= 1<<EPEN;
        UECFG0X = UECFG0XTemp;
        UECFG1X = UECFG1XTemp;
        UEIENX  = UEIENXTemp;

        if (!(Endpoint_IsConfigured()))
            return false;
    }

    Endpoint_SelectEndpoint(Number);
    return true;
}

void USBKB::USB_Device_GetInternalSerialDescriptor()
{
    struct
    {
        USB_Descriptor_Header_t Header;
        uint16_t                UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
    } SignatureDescriptor;

    SignatureDescriptor.Header.Type = DTYPE_String;
    SignatureDescriptor.Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
    USB_Device_GetSerialString(SignatureDescriptor.UnicodeString);
    UEINTX &= ~(1<<RXSTPI);
    Endpoint_Write_Control_Stream_LE(&SignatureDescriptor, sizeof(SignatureDescriptor));
    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
}

bool Endpoint_ConfigureEndpointTable(const USB_Endpoint_Table_t* const Table,
                                     const uint8_t Entries)
{
    for (uint8_t i = 0; i < Entries; i++)
    {
        if (!(Table[i].Address))
          continue;

        if (!(Endpoint_ConfigureEndpoint(Table[i].Address, Table[i].Type, Table[i].Size, Table[i].Banks)))
          return false;
    }

    return true;
}

void USBKB::USB_Device_GetDescriptor()
{
    const void* DescriptorPointer;
    uint16_t    DescriptorSize;

    if (USB_ControlRequest.wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL))
    {
        USB_Device_GetInternalSerialDescriptor();
        return;
    }

    if ((DescriptorSize = CALLBACK_USB_GetDescriptor(USB_ControlRequest.wValue,
        USB_ControlRequest.wIndex, &DescriptorPointer)) == NO_DESCRIPTOR)
    {
        return;
    }

    UEINTX &= ~(1<<RXSTPI);
    Endpoint_Write_Control_PStream_LE(DescriptorPointer, DescriptorSize);
    UEINTX &= ~(1<<RXOUTI | 1<<FIFOCON);
}

uint8_t USB_Device_ConfigurationNumber;

bool    USB_Device_CurrentlySelfPowered;

bool    USB_Device_RemoteWakeupEnabled;

void USBKB::Device_ClearSetFeature()
{
    switch (USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_RECIPIENT)
    {
        case REQREC_DEVICE:
            if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_DeviceRemoteWakeup)
                USB_Device_RemoteWakeupEnabled = (USB_ControlRequest.bRequest == REQ_SetFeature);
            else
                return;

            break;
        case REQREC_ENDPOINT:
            if ((uint8_t)USB_ControlRequest.wValue == FEATURE_SEL_EndpointHalt)
            {
                uint8_t index = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

                if (index == ENDPOINT_CONTROLEP || index >= ENDPOINT_TOTAL_ENDPOINTS)
                    return;

                Endpoint_SelectEndpoint(index);

                if (UECONX & 1<<EPEN)
                {
                    if (USB_ControlRequest.bRequest == REQ_SetFeature)
                    {
                        UECONX |= 1<<STALLRQ;   // stall transaction
                    }
                    else
                    {
                        UECONX |= 1<<STALLRQC;  // clear stall
                        Endpoint_ResetEndpoint(index);
                        Endpoint_ResetDataToggle();
                    }
                }
            }
            break;
        default:
            return;
    }

    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEINTX &= ~(1<<RXSTPI);
    Endpoint_ClearStatusStage();
}

void USBKB::USB_Device_ProcessControlRequest()
{
    uint8_t* RequestHeader = (uint8_t*)&USB_ControlRequest;

    for (uint8_t i = 0; i < sizeof(USB_Request_Header_t); i++)
        *(RequestHeader++) = Endpoint_Read_8();

    EVENT_USB_Device_ControlRequest();

    if (UEINTX & 1<<RXSTPI) // is setup received?
    {
        uint8_t bmRequestType = USB_ControlRequest.bmRequestType;

        switch (USB_ControlRequest.bRequest)
        {
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
                    {
                       uint8_t index = ((uint8_t)USB_ControlRequest.wIndex & ENDPOINT_EPNUM_MASK);

                        if (index >= ENDPOINT_TOTAL_ENDPOINTS)
                            return;

                        Endpoint_SelectEndpoint(index);
                        CurrentStatus = Endpoint_IsStalled();
                        Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
                    }
                        break;
                    default:
                        return;
                    }

                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_16_LE(CurrentStatus);
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
                    uint8_t devAddr = USB_ControlRequest.wValue & 0x7F;
                    USB_Device_SetDeviceAddress(devAddr);
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_ClearStatusStage();
                    while (!(Endpoint_IsINReady()));
                    UDADDR |= 1<<ADDEN;
                    USB_DeviceState = devAddr ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
                }
                break;
            case REQ_GetDescriptor:
                if ((bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE)) ||
                    (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_INTERFACE)))
                {
                    USB_Device_GetDescriptor();
                }

                break;
            case REQ_GetConfiguration:
                if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_STANDARD | REQREC_DEVICE))
                {
                    UEINTX &= ~(1<<RXSTPI);
                    Endpoint_Write_8(USB_Device_ConfigurationNumber);
                    UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
                    Endpoint_ClearStatusStage();
                }
                break;
            case REQ_SetConfiguration:
                if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_STANDARD | REQREC_DEVICE))
                {
                    if ((uint8_t)USB_ControlRequest.wValue > 1)
                        return;

                    UEINTX &= ~(1<<RXSTPI);
                    USB_Device_ConfigurationNumber = (uint8_t)USB_ControlRequest.wValue;
                    Endpoint_ClearStatusStage();

                    if (USB_Device_ConfigurationNumber)
                    {
                        USB_DeviceState = DEVICE_STATE_Configured;
                    }
                    else
                    {
                        USB_DeviceState = UDADDR & 1<<ADDEN ? DEVICE_STATE_Configured :
                            DEVICE_STATE_Powered;
                    }

                    Endpoint_ConfigureEndpoint(ENDPOINT_DIR_IN | 1, EP_TYPE_INTERRUPT, 8, 1);
                    Endpoint_ConfigureEndpoint(ENDPOINT_DIR_OUT | 2, EP_TYPE_INTERRUPT, 8, 1);
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
        UECONX |= 1<<STALLRQ;
    }
}

void EVENT_USB_Device_ControlRequest(void)
{
}

void EVENT_USB_Device_StartOfFrame(void)
{
    if (IdleMSRemaining)
        IdleMSRemaining--;
}

bool USB_GetHIDReportItemInfo(const uint8_t* ReportData,
                              HID_ReportItem_t* const ReportItem)
{
    if (ReportItem == NULL)
      return false;

    uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
    uint16_t CurrentBit   = ReportItem->BitOffset;
    uint32_t BitMask      = (1 << 0);

    if (ReportItem->ReportID)
    {
        if (ReportItem->ReportID != ReportData[0])
          return false;

        ReportData++;
    }

    ReportItem->PreviousValue = ReportItem->Value;
    ReportItem->Value = 0;

    while (DataBitsRem--)
    {
        if (ReportData[CurrentBit / 8] & (1 << (CurrentBit % 8)))
          ReportItem->Value |= BitMask;

        CurrentBit++;
        BitMask <<= 1;
    }

    return true;
}

ISR(USB_COM_vect, ISR_BLOCK)
{
    USBKB::instance->com();
}

void USBKB::com()
{
    uint8_t PrevSelectedEndpoint = Endpoint_GetCurrentEndpoint();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX &= ~(1<<RXSTPE);
    sei();
    USB_Device_ProcessControlRequest();
    Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);
    UEIENX |= 1<<RXSTPE;
    Endpoint_SelectEndpoint(PrevSelectedEndpoint);
}

ISR(USB_GEN_vect, ISR_BLOCK)
{
    USBKB::instance->gen();
}

void USBKB::gen()
{
    if (USB_INT_HasOccurred(USB_INT_SOFI) && USB_INT_IsEnabled(USB_INT_SOFI))
    {
        UDINT &= ~(1<<SOFI);    // clear INT SOFI
        EVENT_USB_Device_StartOfFrame();
    }

    if (USB_INT_HasOccurred(USB_INT_VBUSTI) && USB_INT_IsEnabled(USB_INT_VBUSTI))
    {
        USBINT &= ~(1<<VBUSTI);

        if (USBSTA & 1<<VBUS)
        {
            PLLCSR = USB_PLL_PSC;
            PLLCSR = (USB_PLL_PSC | (1 << PLLE));
            while (!(USB_PLL_IsReady()));
            USB_DeviceState = DEVICE_STATE_Powered;
        }
        else
        {
            PLLCSR = 0;
            USB_DeviceState = DEVICE_STATE_Unattached;
        }
    }

    if (USB_INT_HasOccurred(USB_INT_SUSPI) && USB_INT_IsEnabled(USB_INT_SUSPI))
    {
        USB_INT_Disable(USB_INT_SUSPI);
        UDIEN |= 1<<WAKEUPE;
        USBCON |= 1<<FRZCLK;
        PLLCSR = 0;
        USB_DeviceState = DEVICE_STATE_Suspended;
    }

    if (UDINT & 1<<WAKEUPI && UDIEN & 1<<WAKEUPE)
    {
        PLLCSR = USB_PLL_PSC;
        PLLCSR = USB_PLL_PSC | 1<<PLLE;
        while (!(USB_PLL_IsReady()));
        USBCON &= ~(1 << FRZCLK);
        UDINT &= ~(1<<WAKEUPI);     // int clear
        UDIEN &= ~(1<<WAKEUPI);     // int disable
        UDIEN |= 1<<SUSPE;

        if (USB_Device_ConfigurationNumber)
            USB_DeviceState = DEVICE_STATE_Configured;
        else
            USB_DeviceState = UDADDR & 1<<ADDEN ? DEVICE_STATE_Addressed : DEVICE_STATE_Powered;

    }

    if (UDINT & 1<<EORSTI && UDIEN & 1<<EORSTE)
    {
        UDINT &= ~(1<<EORSTI);
        USB_DeviceState = DEVICE_STATE_Default;
        USB_Device_ConfigurationNumber = 0;
        UDINT &= ~(1<<SUSPI);
        UDIEN &= ~(1<<SUSPE);
        UDIEN |= 1<<WAKEUPE;
        Endpoint_ConfigureEndpoint(ENDPOINT_CONTROLEP, EP_TYPE_CONTROL, 8, 1);
        UEIENX |= 1<<RXSTPE;
    }
}

void Endpoint_ClearEndpoints(void)
{
    UEINT = 0;

    for (uint8_t EPNum = 0; EPNum < ENDPOINT_TOTAL_ENDPOINTS; EPNum++)
    {
        Endpoint_SelectEndpoint(EPNum);
        UEIENX  = 0;
        UEINTX  = 0;
        UECFG1X = 0;
        UECONX &= ~(1<<EPEN);
    }
}


void CreateKeyboardReport(USB_KeyboardReport_Data_t* const ReportData)
{
	uint8_t UsedKeyCodes      = 0;
	memset(ReportData, 0, sizeof(USB_KeyboardReport_Data_t));

	if ((PINF & 1<<0) == 0)
	    ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_A;
	else if ((PINF & 1<<1) == 0)
	    ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_B;

	if ((PINF & 1<<4) == 0)
	    ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_C;
	else if ((PINF & 1<<5) == 0)
	    ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_D;

	if ((PINF & 1<<6) == 0)
	    ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_E;

    if ((PINF & 1<<7) == 0)
        ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_F;
}



uint8_t USB_ProcessHIDReport(const uint8_t* ReportData,
                             uint16_t ReportSize,
                             HID_ReportInfo_t* const ParserData)
{
	HID_StateTable_t      StateTable[HID_STATETABLE_STACK_DEPTH];
	HID_StateTable_t*     CurrStateTable     = &StateTable[0];
	HID_CollectionPath_t* CurrCollectionPath = NULL;
	HID_ReportSizeInfo_t* CurrReportIDInfo   = &ParserData->ReportIDSizes[0];
	uint16_t              UsageList[HID_USAGE_STACK_DEPTH];
	uint8_t               UsageListSize      = 0;
	HID_MinMax_t          UsageMinMax        = {0, 0};

	memset(ParserData,       0x00, sizeof(HID_ReportInfo_t));
	memset(CurrStateTable,   0x00, sizeof(HID_StateTable_t));
	memset(CurrReportIDInfo, 0x00, sizeof(HID_ReportSizeInfo_t));

	ParserData->TotalDeviceReports = 1;

	while (ReportSize)
	{
		uint8_t  HIDReportItem  = *ReportData;
		uint32_t ReportItemData;

		ReportData++;
		ReportSize--;

		switch (HIDReportItem & HID_RI_DATA_SIZE_MASK)
		{
			case HID_RI_DATA_BITS_32:
				ReportItemData = (((uint32_t)ReportData[3] << 24) | ((uint32_t)ReportData[2] << 16) |
			                       ((uint16_t)ReportData[1] << 8)  | ReportData[0]);
				ReportSize     -= 4;
				ReportData     += 4;
				break;

			case HID_RI_DATA_BITS_16:
				ReportItemData  = (((uint16_t)ReportData[1] << 8) | (ReportData[0]));
				ReportSize     -= 2;
				ReportData     += 2;
				break;

			case HID_RI_DATA_BITS_8:
				ReportItemData  = ReportData[0];
				ReportSize     -= 1;
				ReportData     += 1;
				break;

			default:
				ReportItemData  = 0;
				break;
		}

		switch (HIDReportItem & (HID_RI_TYPE_MASK | HID_RI_TAG_MASK))
		{
			case HID_RI_PUSH(0):
				if (CurrStateTable == &StateTable[HID_STATETABLE_STACK_DEPTH - 1])
				  return HID_PARSE_HIDStackOverflow;

				memcpy((CurrStateTable + 1),
				       CurrStateTable,
				       sizeof(HID_ReportItem_t));

				CurrStateTable++;
				break;

			case HID_RI_POP(0):
				if (CurrStateTable == &StateTable[0])
				  return HID_PARSE_HIDStackUnderflow;

				CurrStateTable--;
				break;

			case HID_RI_USAGE_PAGE(0):
				if ((HIDReportItem & HID_RI_DATA_SIZE_MASK) == HID_RI_DATA_BITS_32)
				  CurrStateTable->Attributes.Usage.Page = (ReportItemData >> 16);

				CurrStateTable->Attributes.Usage.Page       = ReportItemData;
				break;

			case HID_RI_LOGICAL_MINIMUM(0):
				CurrStateTable->Attributes.Logical.Minimum  = ReportItemData;
				break;

			case HID_RI_LOGICAL_MAXIMUM(0):
				CurrStateTable->Attributes.Logical.Maximum  = ReportItemData;
				break;

			case HID_RI_PHYSICAL_MINIMUM(0):
				CurrStateTable->Attributes.Physical.Minimum = ReportItemData;
				break;

			case HID_RI_PHYSICAL_MAXIMUM(0):
				CurrStateTable->Attributes.Physical.Maximum = ReportItemData;
				break;

			case HID_RI_UNIT_EXPONENT(0):
				CurrStateTable->Attributes.Unit.Exponent    = ReportItemData;
				break;

			case HID_RI_UNIT(0):
				CurrStateTable->Attributes.Unit.Type        = ReportItemData;
				break;

			case HID_RI_REPORT_SIZE(0):
				CurrStateTable->Attributes.BitSize          = ReportItemData;
				break;

			case HID_RI_REPORT_COUNT(0):
				CurrStateTable->ReportCount                 = ReportItemData;
				break;

			case HID_RI_REPORT_ID(0):
				CurrStateTable->ReportID                    = ReportItemData;

				if (ParserData->UsingReportIDs)
				{
					CurrReportIDInfo = NULL;

					for (uint8_t i = 0; i < ParserData->TotalDeviceReports; i++)
					{
						if (ParserData->ReportIDSizes[i].ReportID == CurrStateTable->ReportID)
						{
							CurrReportIDInfo = &ParserData->ReportIDSizes[i];
							break;
						}
					}

					if (CurrReportIDInfo == NULL)
					{
						if (ParserData->TotalDeviceReports == HID_MAX_REPORT_IDS)
						  return HID_PARSE_InsufficientReportIDItems;

						CurrReportIDInfo = &ParserData->ReportIDSizes[ParserData->TotalDeviceReports++];
						memset(CurrReportIDInfo, 0x00, sizeof(HID_ReportSizeInfo_t));
					}
				}

				ParserData->UsingReportIDs = true;

				CurrReportIDInfo->ReportID = CurrStateTable->ReportID;
				break;

			case HID_RI_USAGE(0):
				if (UsageListSize == HID_USAGE_STACK_DEPTH)
				  return HID_PARSE_UsageListOverflow;

				UsageList[UsageListSize++] = ReportItemData;
				break;

			case HID_RI_USAGE_MINIMUM(0):
				UsageMinMax.Minimum = ReportItemData;
				break;

			case HID_RI_USAGE_MAXIMUM(0):
				UsageMinMax.Maximum = ReportItemData;
				break;

			case HID_RI_COLLECTION(0):
				if (CurrCollectionPath == NULL)
				{
					CurrCollectionPath = &ParserData->CollectionPaths[0];
				}
				else
				{
					HID_CollectionPath_t* ParentCollectionPath = CurrCollectionPath;

					CurrCollectionPath = &ParserData->CollectionPaths[1];

					while (CurrCollectionPath->Parent != NULL)
					{
						if (CurrCollectionPath == &ParserData->CollectionPaths[HID_MAX_COLLECTIONS - 1])
						  return HID_PARSE_InsufficientCollectionPaths;

						CurrCollectionPath++;
					}

					CurrCollectionPath->Parent = ParentCollectionPath;
				}

				CurrCollectionPath->Type       = ReportItemData;
				CurrCollectionPath->Usage.Page = CurrStateTable->Attributes.Usage.Page;

				if (UsageListSize)
				{
					CurrCollectionPath->Usage.Usage = UsageList[0];

					for (uint8_t i = 1; i < UsageListSize; i++)
					  UsageList[i - 1] = UsageList[i];

					UsageListSize--;
				}
				else if (UsageMinMax.Minimum <= UsageMinMax.Maximum)
				{
					CurrCollectionPath->Usage.Usage = UsageMinMax.Minimum++;
				}

				break;

			case HID_RI_END_COLLECTION(0):
				if (CurrCollectionPath == NULL)
				  return HID_PARSE_UnexpectedEndCollection;

				CurrCollectionPath = CurrCollectionPath->Parent;
				break;

			case HID_RI_INPUT(0):
			case HID_RI_OUTPUT(0):
			case HID_RI_FEATURE(0):
				for (uint8_t ReportItemNum = 0; ReportItemNum < CurrStateTable->ReportCount; ReportItemNum++)
				{
					HID_ReportItem_t NewReportItem;

					memcpy(&NewReportItem.Attributes,
					       &CurrStateTable->Attributes,
					       sizeof(HID_ReportItem_Attributes_t));

					NewReportItem.ItemFlags      = ReportItemData;
					NewReportItem.CollectionPath = CurrCollectionPath;
					NewReportItem.ReportID       = CurrStateTable->ReportID;

					if (UsageListSize)
					{
						NewReportItem.Attributes.Usage.Usage = UsageList[0];

						for (uint8_t i = 1; i < UsageListSize; i++)
						  UsageList[i - 1] = UsageList[i];

						UsageListSize--;
					}
					else if (UsageMinMax.Minimum <= UsageMinMax.Maximum)
					{
						NewReportItem.Attributes.Usage.Usage = UsageMinMax.Minimum++;
					}

					uint8_t ItemTypeTag = (HIDReportItem & (HID_RI_TYPE_MASK | HID_RI_TAG_MASK));

					if (ItemTypeTag == HID_RI_INPUT(0))
					  NewReportItem.ItemType = HID_REPORT_ITEM_In;
					else if (ItemTypeTag == HID_RI_OUTPUT(0))
					  NewReportItem.ItemType = HID_REPORT_ITEM_Out;
					else
					  NewReportItem.ItemType = HID_REPORT_ITEM_Feature;

					NewReportItem.BitOffset = CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType];

					CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType] += CurrStateTable->Attributes.BitSize;

					ParserData->LargestReportSizeBits = MAX(ParserData->LargestReportSizeBits, CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType]);

					if (ParserData->TotalReportItems == HID_MAX_REPORTITEMS)
					  return HID_PARSE_InsufficientReportItems;

					memcpy(&ParserData->ReportItems[ParserData->TotalReportItems],
					       &NewReportItem, sizeof(HID_ReportItem_t));
#if 0
					if (!(ReportItemData & HID_IOF_CONSTANT) && CALLBACK_HIDParser_FilterHIDReportItem(&NewReportItem))
					  ParserData->TotalReportItems++;
#endif
				}

				break;

			default:
				break;
		}

		if ((HIDReportItem & HID_RI_TYPE_MASK) == HID_RI_TYPE_MAIN)
		{
			UsageMinMax.Minimum = 0;
			UsageMinMax.Maximum = 0;
			UsageListSize       = 0;
		}
	}

	if (!(ParserData->TotalReportItems))
	  return HID_PARSE_NoUnfilteredReportItems;

	return HID_PARSE_Successful;
}

uint8_t USBKB::Endpoint_Write_Stream_LE (const void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream = (uint8_t*)Buffer;
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode;

    if ((ErrorCode = Endpoint_WaitUntilReady()))
        return ErrorCode;

    if (BytesProcessed != NULL)
    {
        Length -= *BytesProcessed;
        DataStream += *BytesProcessed;
    }

    while (Length)
    {
        if (!(Endpoint_IsReadWriteAllowed()))
        {
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            USBTask();

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = Endpoint_WaitUntilReady()))
              return ErrorCode;
        }
        else
        {
            Endpoint_Write_8(*DataStream);
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Read_Stream_LE(void * const Buffer, uint16_t Length,
                            uint16_t* const BytesProcessed)
{
    uint8_t* DataStream      = ((uint8_t*)Buffer);
    uint16_t BytesInTransfer = 0;
    uint8_t  ErrorCode;
    if ((ErrorCode = Endpoint_WaitUntilReady()))
      return ErrorCode;

    if (BytesProcessed != NULL)
    {
        Length -= *BytesProcessed;
        DataStream += *BytesProcessed;
    }
    while (Length)
    {
        if (!(Endpoint_IsReadWriteAllowed()))
        {
            Endpoint_ClearOUT();
            USBTask();

            if (BytesProcessed != NULL)
            {
                *BytesProcessed += BytesInTransfer;
                return ENDPOINT_RWSTREAM_IncompleteTransfer;
            }

            if ((ErrorCode = Endpoint_WaitUntilReady()))
              return ErrorCode;
        }
        else
        {
            *DataStream = Endpoint_Read_8();
            DataStream += 1;
            Length--;
            BytesInTransfer++;
        }
    }

    return ENDPOINT_RWSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Read_Control_Stream_LE(void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream = ((uint8_t*)Buffer);

    if (!(Length))
        Endpoint_ClearOUT();

    while (Length)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
          return ENDPOINT_RWCSTREAM_HostAborted;

        if (Endpoint_IsOUTReceived())
        {
            while (Length && Endpoint_BytesInEndpoint())
            {
                *DataStream = Endpoint_Read_8();
                DataStream += 1;
                Length--;
            }

            Endpoint_ClearOUT();
        }
    }

    while (!(Endpoint_IsINReady()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Write_Control_Stream_LE(const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = ((uint8_t*)Buffer);
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
          return ENDPOINT_RWCSTREAM_HostAborted;
        else if (Endpoint_IsOUTReceived())
          break;

        if (Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                Endpoint_Write_8(*DataStream);
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }
            
            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);     // clear IN
        }
    }

    while (!(Endpoint_IsOUTReceived()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
          return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}

uint8_t USBKB::Endpoint_Write_Control_PStream_LE (const void* const Buffer, uint16_t Length)
{
    uint8_t* DataStream     = (uint8_t*)Buffer;
    bool     LastPacketFull = false;

    if (Length > USB_ControlRequest.wLength)
        Length = USB_ControlRequest.wLength;
    else if (!(Length))
        UEINTX &= ~(1<<TXINI | 1<<FIFOCON);

    while (Length || LastPacketFull)
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
          return ENDPOINT_RWCSTREAM_HostAborted;
        else if (Endpoint_IsOUTReceived())
          break;

        if (Endpoint_IsINReady())
        {
            uint16_t BytesInEndpoint = Endpoint_BytesInEndpoint();

            while (Length && (BytesInEndpoint < 8))
            {
                Endpoint_Write_8(pgm_read_byte(DataStream));
                DataStream += 1;
                Length--;
                BytesInEndpoint++;
            }
            
            LastPacketFull = (BytesInEndpoint == 8);
            UEINTX &= ~(1<<TXINI | 1<<FIFOCON);
            //Endpoint_ClearIN();
        }
    }

    while (!(Endpoint_IsOUTReceived()))
    {
        uint8_t USB_DeviceState_LCL = USB_DeviceState;

        if (USB_DeviceState_LCL == DEVICE_STATE_Unattached)
          return ENDPOINT_RWCSTREAM_DeviceDisconnected;
        else if (USB_DeviceState_LCL == DEVICE_STATE_Suspended)
          return ENDPOINT_RWCSTREAM_BusSuspended;
        else if (Endpoint_IsSETUPReceived())
          return ENDPOINT_RWCSTREAM_HostAborted;
    }

    return ENDPOINT_RWCSTREAM_NoError;
}



const USB_Descriptor_HIDReport_Datatype_t PROGMEM KeyboardReport[] =
{
    HID_RI_USAGE_PAGE(8, 0x01), /* Generic Desktop */
    HID_RI_USAGE(8, 0x06), /* Keyboard */
    HID_RI_COLLECTION(8, 0x01), /* Application */
    HID_RI_USAGE_PAGE(8, 0x07), /* Key Codes */
    HID_RI_USAGE_MINIMUM(8, 0xE0), /* Keyboard Left Control */
    HID_RI_USAGE_MAXIMUM(8, 0xE7), /* Keyboard Right GUI */
    HID_RI_LOGICAL_MINIMUM(8, 0x00),
    HID_RI_LOGICAL_MAXIMUM(8, 0x01),
    HID_RI_REPORT_SIZE(8, 0x01),
    HID_RI_REPORT_COUNT(8, 0x08),
    HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_RI_REPORT_COUNT(8, 0x01),
    HID_RI_REPORT_SIZE(8, 0x08),
    HID_RI_INPUT(8, HID_IOF_CONSTANT),
    HID_RI_USAGE_PAGE(8, 0x08), /* LEDs */
    HID_RI_USAGE_MINIMUM(8, 0x01), /* Num Lock */
    HID_RI_USAGE_MAXIMUM(8, 0x05), /* Kana */
    HID_RI_REPORT_COUNT(8, 0x05),
    HID_RI_REPORT_SIZE(8, 0x01),
    HID_RI_OUTPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE | HID_IOF_NON_VOLATILE),
    HID_RI_REPORT_COUNT(8, 0x01),
    HID_RI_REPORT_SIZE(8, 0x03),
    HID_RI_OUTPUT(8, HID_IOF_CONSTANT),
    HID_RI_LOGICAL_MINIMUM(8, 0x00),
    HID_RI_LOGICAL_MAXIMUM(8, 0x65),
    HID_RI_USAGE_PAGE(8, 0x07), /* Keyboard */
    HID_RI_USAGE_MINIMUM(8, 0x00), /* Reserved (no event indicated) */
    HID_RI_USAGE_MAXIMUM(8, 0x65), /* Keyboard Application */
    HID_RI_REPORT_COUNT(8, 0x06),
    HID_RI_REPORT_SIZE(8, 0x08),
    HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),
    HID_RI_END_COLLECTION(0),
};

const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    {
        sizeof(USB_Descriptor_Device_t),
        DTYPE_Device
    },

	0x0110,
	USB_CSCP_NoDeviceClass,
	USB_CSCP_NoDeviceSubclass,
	USB_CSCP_NoDeviceProtocol,
    8,
	0x03EB,
	0x2042,
	0x0001,
	STRING_ID_Manufacturer,
	STRING_ID_Product,
	NO_DESCRIPTOR,
    1
};

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    {
        {
            sizeof(USB_Descriptor_Configuration_Header_t),
            DTYPE_Configuration
        },

        sizeof(USB_Descriptor_Configuration_t),
        1,
        1,
        NO_DESCRIPTOR,
        USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED,
        USB_CONFIG_POWER_MA(100)
    },
    {   // HID_Interface
        {
            sizeof(USB_Descriptor_Interface_t),
            DTYPE_Interface
        },
        INTERFACE_ID_Keyboard,
        0,
        2,
        HID_CSCP_HIDClass,
        HID_CSCP_BootSubclass,
        HID_CSCP_KeyboardBootProtocol,
        NO_DESCRIPTOR
    },
    {   // HID_KeyboardHID
        .Header =
        {
            sizeof(USB_HID_Descriptor_HID_t),
            HID_DTYPE_HID
        },
        0x0111,
        0x00,
        1,
        HID_DTYPE_Report,
        sizeof(KeyboardReport)
    },
    {   // HID_ReportINEndpoint
        {
            sizeof(USB_Descriptor_Endpoint_t),
            DTYPE_Endpoint
        },

        KEYBOARD_IN_EPADDR,
        EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA,
        8,
        5
    },

    .HID_ReportOUTEndpoint =
    {
        .Header =
        {
            .Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint
        },

        .EndpointAddress        = KEYBOARD_OUT_EPADDR,
        .Attributes = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
        .EndpointSize           = 8,
        .PollingIntervalMS      = 5
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
    L"LUFA USB-RS232 Adapter"
};

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
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
		case HID_DTYPE_HID:
			Address = &ConfigurationDescriptor.HID_KeyboardHID;
			Size    = sizeof(USB_HID_Descriptor_HID_t);
			break;
		case HID_DTYPE_Report:
			Address = &KeyboardReport;
			Size    = sizeof(KeyboardReport);
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}

