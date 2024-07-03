// ******************************************************************
//
//    File: twn4.sys.h
//
// Purpose:
//
// Include file for Apps for TWN4. Define system datatypes, macros
// prototypes.
//
// ******************************************************************

#if !defined(__TWN4_SYS_H__) || defined(__TWN4_SYS_C__)
#ifndef __TWN4_SYS_H__
#define __TWN4_SYS_H__
#endif

// Best viewed with tab width set to 4

// ******************************************************************
// ****** Common Definitions ****************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#define true                1
#define false               0

typedef unsigned char byte;
typedef int bool;

#define MAX(a,b)            ((a) > (b) ? (a) : (b))
#define MIN(a,b)            ((a) < (b) ? (a) : (b))
#define ABS(a)              ((a) > 0 ? (a) : (-(a)))

#define ON                  1
#define OFF                 0

#define BYTE0(x)            ((uint8_t) (x)     )
#define BYTE1(x)            ((uint8_t)((x)>> 8))
#define BYTE2(x)            ((uint8_t)((x)>>16))
#define BYTE3(x)            ((uint8_t)((x)>>24))

#define LOBYTE(x)           BYTE0(x)
#define HIBYTE(x)           BYTE1(x)

#define USB_WORD(x)         BYTE0(x),BYTE1(x)
#define USB_DWORD(x)        BYTE0(x),BYTE1(x),BYTE2(x),BYTE3(x)

#define UNUSED_VAR(x)       ((void)sizeof((void)(x),0))

typedef void (*TFunc)(void);

#define CONVERT_INT_TO_BYTES_MSB(Int)          ((byte)(Int >> 24)),((byte)(Int >> 16)),((byte)(Int >> 8)),((byte)Int)
#define CONVERT_INT_TO_BYTES_LSB(Int)          ((byte)Int),((byte)(Int >> 8)),((byte)(Int >> 16)),((byte)(Int >> 24))

#define CONVERT_BYTES_TO_INT_MSB(Bytes)        ((*(Bytes+0) << 24)+(*(Bytes+1) << 16)+(*(Bytes+2) << 8)+(*(Bytes+3)))
#define CONVERT_BYTES_TO_INT_LSB(Bytes)        ((*(Bytes+0))+(*(Bytes+1) << 8)+(*(Bytes+2) << 16)+(*(Bytes+3) << 24))

#define CONVERT_INT16_TO_BYTES_MSB(Int)        ((byte)(Int >> 8)),((byte)Int)
#define CONVERT_INT16_TO_BYTES_LSB(Int)        ((byte)Int),((byte)(Int >> 8))

#define CONVERT_BYTES_TO_INT16_MSB(Bytes)      ((*(Bytes+0) << 8)+(*(Bytes+1)))
#define CONVERT_BYTES_TO_INT16_LSB(Bytes)      ((*(Bytes+0))+(*(Bytes+1) << 8))

#endif

// ******************************************************************
// ****** Error Codes ***********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

// ------ General Errors --------------------------------------------

#define ERR_NONE                         0
#define ERR_OUTOFMEMORY                  1
#define ERR_ISALREADYINIT                2
#define ERR_NOTINIT                      3
#define ERR_ISALREADYOPEN                4
#define ERR_NOTOPEN                      5
#define ERR_RANGE                        6
#define ERR_PARAMETER                    7
#define ERR_GENERAL                      8
#define ERR_NOTSUPPORTED                 9
#define ERR_STATE                        10
#define ERR_COMPATIBILITY                11
#define ERR_DATA                         12

// ------ Storage Errors --------------------------------------------

#define ERR_UNKNOWNSTORAGEID             100
#define ERR_WRONGINDEX                   101
#define ERR_FLASHERASE                   102
#define ERR_FLASHWRITE                   103
#define ERR_SECTORNOTFOUND               104
#define ERR_STORAGEFULL                  105
#define ERR_STORAGEINVALID               106
#define ERR_TRANSACTIONLIMIT             107

// ------ File Errors -----------------------------------------------

#define ERR_UNKNOWNFS                    200
#define ERR_FILENOTFOUND                 201
#define ERR_FILEALREADYEXISTS            202
#define ERR_ENDOFFILE                    203
#define ERR_STORAGENOTFOUND              204
#define ERR_STORAGEALREADYMOUNTED        205
#define ERR_ACCESSDENIED                 206
#define ERR_FILECORRUPT                  207
#define ERR_INVALIDFILEENV               208
#define ERR_INVALIDFILEID                209
#define ERR_RESOURCELIMIT                210

// ------ I2C Errors -----------------------------------------------

#define ERR_TIMEOUT                      300
#define ERR_PECERR                       301
#define ERR_OVR                          302
#define ERR_AF                           303  	// Acknowledge Error
#define ERR_ARLO                         304
#define ERR_BERR                         305    // Bus Error

#endif

// ******************************************************************
// ****** Parameters ************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define TLV_END							0

#define OPEN_PORTS						1
  #define OPEN_PORT_USB_MSK						0x01
  #define OPEN_PORT_COM1_MSK					0x02
  #define OPEN_PORT_COM2_MSK					0x04
  #define OPEN_PORT_COM3_MSK					0x08
  #define OPEN_PORT_SPI_MSK                     0x10
  #define OPEN_PORTS_DEFAULT            		(OPEN_PORT_USB_MSK | OPEN_PORT_COM1_MSK | OPEN_PORT_COM2_MSK)

#define EXECUTE_APP						2
  #define EXECUTE_APP_AUTO						0
  #define EXECUTE_APP_ALWAYS					1
  #define EXECUTE_APP_DEFAULT					EXECUTE_APP_AUTO

#define INDITAG_READMODE				3
  #define INDITAG_READMODE_1					1
  #define INDITAG_READMODE_2					2
  #define INDITAG_READMODE_3					3
  #define INDITAG_READMODE_DEFAULT				INDITAG_READMODE_1

#define COTAG_READMODE					4
  #define COTAG_READMODE_HASH					0
  #define COTAG_READMODE_1						1	// Read 32 bits
  #define COTAG_READMODE_2						2	// Read 48 bits
  #define COTAG_READMODE_3						3	// Read 64 bits
  #define COTAG_READMODE_DEFAULT				COTAG_READMODE_1

#define COTAG_VERIFY					5
  #define COTAG_VERIFY_OFF						OFF
  #define COTAG_VERIFY_ON						ON
  #define COTAG_VERIFY_DEFAULT					COTAG_VERIFY_ON

#define HONEYTAG_READMODE				6
  #define HONEYTAG_READMODE_HASH				0
  #define HONEYTAG_READMODE_1					1
  #define HONEYTAG_READMODE_DEFAULT				HONEYTAG_READMODE_1

#define ICLASS_READMODE					7
  #define ICLASS_READMODE_UID					0
  #define ICLASS_READMODE_PAC					1
  #define ICLASS_READMODE_DEFAULT				ICLASS_READMODE_UID

#define AT55_BITRATE					11
  #define AT55_BITRATE_DEFAULT          		64

#define AT55_OPTIONS					12

  #define AT55_OPT_SEQUENCEMASK					0x03
  #define AT55_OPT_SEQUENCENONE					0x00
  #define AT55_OPT_SEQUENCETERMINATOR			0x01
  #define AT55_OPT_SEQUENCESTARTMARKER			0x02

  #define AT55_OPT_DLPROT_MASK                  0x0C
  #define AT55_OPT_DLPROT_FIXED                 0x00
  #define AT55_OPT_DLPROT_LONGREF               0x04
  #define AT55_OPT_DLPROT_0REF                  0x08
  // #define AT55_OPT_DLPROT_1OF4               0x0C (not supported)

  #define AT55_OPT_TESTMODE_MASK				0x10
  #define AT55_OPT_TESTMODE_OFF					0x00
  #define AT55_OPT_TESTMODE_ON					0x10

  #define AT55_OPTIONS_DEFAULT          		(AT55_OPT_SEQUENCENONE | AT55_OPT_DLPROT_FIXED | AT55_OPT_TESTMODE_OFF)

#define CCID_MAXSLOTINDEX				13
  #define CCID_MAXSLOTINDEX_DEFAULT				0

#define HOST_CHANNEL					14

#define HITAG1S_T0						15
  #define HITAG1S_T0_DEFAULT					21
#define HITAG1S_T1						16
  #define HITAG1S_T1_DEFAULT					29
#define HITAG1S_TGAP					17
  #define HITAG1S_TGAP_DEFAULT					10

#define HITAG2_T0						18
  #define HITAG2_T0_DEFAULT						22
#define HITAG2_T1						19
  #define HITAG2_T1_DEFAULT						28
#define HITAG2_TGAP						20
  #define HITAG2_TGAP_DEFAULT					14

#define ISO14443_BITRATE_TX				21
#define ISO14443_BITRATE_RX				22
  #define ISO14443_BITRATE_106					0
  #define ISO14443_BITRATE_212					1
  #define ISO14443_BITRATE_424					2
  #define ISO14443_BITRATE_848					3
  #define ISO14443_BITRATE_DEFAULT          	ISO14443_BITRATE_106

#define HITAG2_READMODE					23
  #define HITAG2_READMODE_1						1
  #define HITAG2_READMODE_2						2
  #define HITAG2_READMODE_DEFAULT				HITAG2_READMODE_1

#define USB_SUPPORTREMOTEWAKEUP			25
  #define USB_SUPPORTREMOTEWAKEUP_OFF			OFF
  #define USB_SUPPORTREMOTEWAKEUP_ON			ON
  #define USB_SUPPORTREMOTEWAKEUP_DEFAULT		USB_SUPPORTREMOTEWAKEUP_OFF

#define EM4102_OPTIONS					26
  #define EM4102_OPTIONS_F64                	0x01
  #define EM4102_OPTIONS_F32                	0x02
  #define EM4102_OPTIONS_DEFAULT				(EM4102_OPTIONS_F64 | EM4102_OPTIONS_F32)

#define EM4150_OPTIONS					27
  #define EM4150_OPTIONS_F64                	0x01
  #define EM4150_OPTIONS_F40                	0x02
  #define EM4150_OPTIONS_F32                	0x04
  #define EM4150_OPTIONS_DEFAULT				(EM4150_OPTIONS_F64 | EM4150_OPTIONS_F40)

#define USB_SERIALNUMBER				28
  #define USB_SERIALNUMBER_OFF					0
  #define USB_SERIALNUMBER_DEVICEUID			1
  #define USB_SERIALNUMBER_PRODUCTION			2
  #define USB_SERIALNUMBER_ON					1	// mature
  #define USB_SERIALNUMBER_DEFAULT				USB_SERIALNUMBER_OFF

#define USB_KEYBOARDREPEATRATE			29
  #define USB_KEYBOARDREPEATRATE_DEFAULT		10

#define SEOS_TREATMENT					30
  #define SEOS_TREATMENT_DEFAULT				SEOS_TREATMENT_ICLASS
  #define SEOS_TREATMENT_ICLASS					1
  #define SEOS_TREATMENT_ISO14443A          	2

#define SUPPORT_CONFIGCARD				31
  #define SUPPORT_CONFIGCARD_DEFAULT			0
  #define SUPPORT_CONFIGCARD_OFF				0
  #define SUPPORT_CONFIGCARD_ON					1
  #define SUPPORT_UPGRADECARD_ON				2

#define USB_VID							32
  #define USB_VID_ELATEC						0x09D8

#define USB_PID							33
  #define USB_PID_KEYBOARD						0x0410
  #define USB_PID_CDC							0x0420
  #define USB_PID_CCID_HID						0x0425
  #define USB_PID_REPORTS						0x0426
  #define USB_PID_CCID_CDC						0x0427
  #define USB_PID_CCID							0x0428
  #define USB_PID_CCID_KBD                      0x042A

#define ISO14443_3_TDX_CRCCONTROL		34
  #define ISO14443_3_TDX_CRCCTRL_DEFAULT		0x00
  #define ISO14443_3_TDX_CRCCTRL_TX				0x02
  #define ISO14443_3_TDX_CRCCTRL_RX				0x01

#define CUSTOM_OPT_1					35
  #define CUSTOM_OPT_1_OFF						OFF
  #define CUSTOM_OPT_1_ON						ON
  #define CUSTOM_OPT_1_DEFAULT					CUSTOM_OPT_1_OFF

#define ISO7816_CONTROL					36
  #define ISO7816_DEFAULT						(1 << ISO7816_HANDLE_ERROR_SIGNAL_ATR) | (3 << ISO7816_TRANSMIT_ERROR_CNT) | (3 << ISO7816_RECEIVE_ERROR_CNT) | (ISO7816_POWERSELECT_5V << ISO7816_VOLTAGE_SYNC_CARDS)
  #define ISO7816_HANDLE_ERROR_SIGNAL_ATR		0	// 1 Bit
  #define ISO7816_TRANSMIT_ERROR_CNT			1	// 3 Bit
  #define ISO7816_RECEIVE_ERROR_CNT				4	// 3 Bit
  #define ISO7816_VOLTAGE_SYNC_CARDS			7	// 2 Bit
  #define ISO7816_SUPPORT_EMVCO                 9   // 1 Bit

#define PN5180_LPCD_THRESHOLD			37
  #define PN5180_LPCD_THRESHOLD_DEFAULT			5

#define PN5180_LPCD_SENSING_PERIOD		38
  #define PN5180_LPCD_SENSING_PERIOD_DEFAULT    200
  #define PN5180_LPCD_SENSING_PERIOD_MIN		1
  #define PN5180_LPCD_SENSING_PERIOD_MAX		2690

#define PN5180_ASK10_SETTING            39
  #define PN5180_ASK10_SETTING_DEFAULT          PN5180_ASK10_SETTING_FIRMWARE
  #define PN5180_ASK10_SETTING_FIRMWARE         0
  #define PN5180_ASK10_SETTING_EEPROM           1

#define SLEEP_CLOCKS           			40
  #define SLEEP_CLOCKS_DEFAULT					0
  #define SLEEP_CLOCK_SAM1_ON					0x0001
  #define SLEEP_CLOCK_SAM2_ON					0x0002

#define HF_INITIAL_RF_GUARDTIME         41
  #define HF_INITIAL_RF_GUARDTIME_DEFAULT       50

#define APPLEPAY_MID1                   42
#define APPLEPAY_MID2                   43

#define APPLEPAY_VASUPADATA             44
  #define APPLEPAY_VASUPADATA_DEFAULT           { 0x6A, 0x01, 0x00, 0x00, 0x02 }
  #define APPLEPAY_VASUPADATABYTECNT_DEFAULT    5

#define APPLEPAY_PRIVATEKEY             45

#define DESFIRE_OPTIONS                 46
  #define DESFIRE_OPTIONS_DEFAULT               0
  #define DESFIRE_OPTIONS_APIWRAPPED            1

#define USB_STACK						47
  #define USB_STACK_DEFAULT						0

#define USB_KEYBOARDLAYOUT				48
  #define USB_KEYBOARDLAYOUT_ENGLISH			0
  #define USB_KEYBOARDLAYOUT_GERMAN				1
  #define USB_KEYBOARDLAYOUT_FRENCH				2
  #define USB_KEYBOARDLAYOUT_DEFAULT			USB_KEYBOARDLAYOUT_ENGLISH

#define USB_KEYBOARDSENDALTCODES		49
  #define USB_KEYBOARDSENDALTCODES_OFF			0
  #define USB_KEYBOARDSENDALTCODES_ON			1
  #define USB_KEYBOARDSENDALTCODES_DEFAULT		USB_KEYBOARDSENDALTCODES_OFF

#define ENABLE_WATCHDOG					50
  #define WATCHDOG_OFF							0
  #define WATCHDOG_ON							1
  #define WATCHDOG_DEFAULT						WATCHDOG_ON

#define DEISTER_READMODE				51
  #define DEISTER_READMODE_TYPE1_RAW            0x01
  #define DEISTER_READMODE_TYPE2_RAW            0x02
  #define DEISTER_READMODE_TYPE3_RAW            0x04
  #define DEISTER_READMODE_TYPE4_RAW            0x08
  #define DEISTER_READMODE_DEFAULT				0

#define GOOGLESMARTTAP_COLLECTORID      52

#define GOOGLESMARTTAP_PRIVATEKEY       53

#define GOOGLESMARTTAP_LONG_TERM_KEYVER 54


#define AT55_TRESET                     55
  #define AT55_TRESET_DEFAULT                   10000       // microseconds
#define AT55_TPOWERUP                   56
  #define AT55_TPOWERUP_DEFAULT                 5000        // microseconds
#define AT55_TPROG                      57
  #define AT55_TPROG_DEFAULT                    30000       // microseconds
#define AT55_TCUTOFF                    58
  #define AT55_TCUTOFF_DEFAULT                  0           // microseconds, 0 = no cutoff
#define AT55_CSGAP					    59
  #define AT55_CSGAP_DEFAULT				    40          // cycles
#define AT55_CWGAP					    60
  #define AT55_CWGAP_DEFAULT				    28          // cycles
#define AT55_C0						    61
  #define AT55_C0_DEFAULT					    16          // cycles

#define SUPPORT_CONFIG_CARD_HID         62
  #define SUPPORT_CONFIG_CARD_HID_OFF           OFF
  #define SUPPORT_CONFIG_CARD_HID_ON            ON
  #define SUPPORT_CONFIG_CARD_HID_DEFAULT       SUPPORT_CONFIG_CARD_HID_ON

#define PORTS_START_BOOTLOADER        	63
  #define USB_START_BOOTLOADER_MSK				0x01
  #define COM1_START_BOOTLOADER_MSK				0x02
  #define COM2_START_BOOTLOADER_MSK				0x04
  #define PORTS_START_BOOTLOADER_DEFAULT		(USB_START_BOOTLOADER_MSK | COM1_START_BOOTLOADER_MSK | COM2_START_BOOTLOADER_MSK)

#define GIROE_PUBLICKEYHOST             64
#define GIROE_PUBLICKEYHOSTVERSION      65
  #define GIROE_PUBLICKEYHOSTVERSION_DEFAULT    3

#define HF_OUTPUT_POWER                 66
  #define HF_OUTPUT_POWER_MIN                   0
  #define HF_OUTPUT_POWER_MAX                   255
  #define HF_OUTPUT_POWER_DEFAULT               HF_OUTPUT_POWER_MAX

#define LEGICCONNECT_RSSI_MIN           67
  #define LEGICCONNECT_RSSI_MIN_DEFAULT         -128

#define OPTION_BYTES                    68      // Read only

#define EM4305_T1_ON                    69
    #define EM4305_T1_ON_DEFAULT                32
#define EM4305_T0_ON                    70
    #define EM4305_T0_ON_DEFAULT                18
#define EM4305_T0_OFF                   71
    #define EM4305_T0_OFF_DEFAULT               14

#define ICLASS_SUPPORTEDTECH            72
  #define ICLASS_SUPPORTEDTECH_ISO14443A        1
  #define ICLASS_SUPPORTEDTECH_ISO14443B        2
  #define ICLASS_SUPPORTEDTECH_ISO15693         4
  #define ICLASS_SUPPORTEDTECH_DEFAULT			(ICLASS_SUPPORTEDTECH_ISO14443A | ICLASS_SUPPORTEDTECH_ISO14443B | ICLASS_SUPPORTEDTECH_ISO15693)

#define ISO14443_CONTROL                73
  #define ISO14443A_FORCE_L4                    0
  #define ISO14443A_FORCE_L3                    1
  #define ISO14443_CONTROL_DEFAULT              0

#define ONEXCEPTION                     74
  #define ONEXCEPTION_STOP                      0
  #define ONEXCEPTION_RESTART                   1
  #define ONEXCEPTION_DUMP                      2
  #define ONEXCEPTION_DEFAULT                   ONEXCEPTION_STOP

#endif

// ******************************************************************
// ****** Invoke System Function ************************************
// ******************************************************************

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifdef __TWN4_SYS_C__

#define xstr(s) str(s)
#define str(s) #s

#undef SYSFUNC
#define SYSFUNC(ApiNo,FuncNo,Prototype) \
    Prototype                           \
    {                                   \
        __asm("push {r4,r5,lr}");       \
        __asm("mov r4,#" xstr(ApiNo));  \
        __asm("mov r5,#" xstr(FuncNo)); \
        __asm("svc 0x80");              \
        __asm("pop {r4,r5,pc}");        \
    }

#else

typedef struct
{
    unsigned int ApiNo;
    unsigned int FuncNo;
    unsigned int *Param;
    unsigned int *Result;
} TEnvSysCall;

#ifdef MAKEFIRMWARE
#define SYSFUNC(ApiNo,FuncNo,Prototype) Prototype;
#else
#define SYSFUNC(ApiNo,FuncNo,Prototype) Prototype __attribute__( ( naked ) );
#endif

#define API_SYS                 0
#define API_IO                  1
#define API_MEM                 2
#define API_CONV                3
#define API_PERIPH              4
#define API_RF                  5
#define API_TILF                6
#define API_HITAG1S             7
#define API_HITAG2              8
#define API_SM                  9        // former API_SM4X00
#define API_I2C                 10
#define API_MIFARECLASSIC       11
#define API_MIFAREULTRALIGHT    12
#define API_ISO15693            13
#define API_CRYPTO              14
#define API_DESFIRE             15
#define API_ISO7816             16
#define API_ICLASS              17
#define API_ISO14443            18
#define API_LF                  19
#define API_HF                  20
#define API_AT55                21
#define API_EM4150              25
#define API_FILESYS             26
#define API_MIFAREPLUS          27
#define API_ADC                 28
#define API_FELICA              29

#define API_SLE44XX             31
#define API_NTAG                32
#define API_SRX                 33
#define API_SAMAVX              34
#define API_EM4102              35
#define API_SPI                 36
#define API_BLE                 37
#define API_EMVCO               38
#define API_PN5180              39
#define API_I2CCARD             40
#define API_TOPAZ               41
#define API_CTS                 42
#define API_MIRROR              44
#define API_APPLEPAY            45
#define API_TRANSACT            46
#define API_EM4305              47
#define API_KLEVERKEY           48
#define API_GPAY                49
#define API_GIROE               50
#define API_LEAF                51
#define API_ECC                 52

#endif

// ******************************************************************
// ****** System Function ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define USBTYPE_NONE				0
#define USBTYPE_CDC					1
#define USBTYPE_KEYBOARD			4

#define DEVTYPE_LEGICNFC 			10
#define DEVTYPE_MIFARENFC 			11
#define DEVTYPE_LEGIC63				12

#define WAKEUPMASK(WakeupSource)  	(1 << WakeupSource)
#define WAKEUP_SOURCE_NONE      	-1
#define WAKEUP_SOURCE_USB       	0
#define WAKEUP_SOURCE_COM1      	1
#define WAKEUP_SOURCE_COM2      	2
#define WAKEUP_SOURCE_TIMEOUT   	4
#define WAKEUP_SOURCE_LPCD      	5

#define WAKEUP_BY_USB_MSK       	WAKEUPMASK(WAKEUP_SOURCE_USB)
#define WAKEUP_BY_COM1_MSK      	WAKEUPMASK(WAKEUP_SOURCE_COM1)
#define WAKEUP_BY_COM2_MSK      	WAKEUPMASK(WAKEUP_SOURCE_COM2)
#define WAKEUP_BY_TIMEOUT_MSK   	WAKEUPMASK(WAKEUP_SOURCE_TIMEOUT)
#define WAKEUP_BY_LPCD_MSK      	WAKEUPMASK(WAKEUP_SOURCE_LPCD)

#define SLEEPMODE_MSK				0x0F00
#define SLEEPMODE_SLEEP				0x0000
#define SLEEPMODE_STOP				0x0100

typedef struct __attribute__((__packed__))
{
	uint16_t	Compatibility;		// 128 = CORE, ...
	uint16_t	BootBranch;			// Always 1
	uint16_t 	BootVersion;		// 0x0110
	uint16_t	FirmwareKeyType;	// 1 = Standardkey

	byte		BranchNum;			//   1 = Keyboard,   2 = CDC
	char		BranchChar;			// 'K' = Keyboard, 'C' = CDC
	uint16_t	FirmwareVersion;	// 0x0321 = 3.21

	char		AppChars[4];		// "STDC", "STD"
	uint16_t	AppVersion;			// 0x0102 = 1.02
} TVersionInfo;

enum TIntNo
{
	INTNO_SYSTICK,
	INTNO_USB_BYTES_TRANSMITTED,
	INTNO_USB_BYTES_RECEIVED,
	INTNO_COM1_BYTE_TRANSMITTED,
	INTNO_COM1_BYTE_RECEIVED,
	INTNO_COM2_BYTE_TRANSMITTED,
	INTNO_COM2_BYTE_RECEIVED,
	INTNO_GPIO0_TRIGGERED,
	INTNO_GPIO1_TRIGGERED,
	INTNO_GPIO2_TRIGGERED,
	INTNO_GPIO3_TRIGGERED,
	INTNO_GPIO4_TRIGGERED,
	INTNO_GPIO5_TRIGGERED,
	INTNO_GPIO6_TRIGGERED,
	INTNO_GPIO7_TRIGGERED,
	INTNO_TIMER,
  INTNO_I2C_RX_FINISHED,
  INTNO_I2C_TX_FINISHED,
	__INTNO_COUNT_ENUM
};

typedef void (*TInterruptHandler)(void);

#endif

SYSFUNC(API_SYS, 0,bool SysCall(TEnvSysCall *Env))
SYSFUNC(API_SYS, 1,void Reset(void))
SYSFUNC(API_SYS, 2,void StartBootloader(void))
SYSFUNC(API_SYS, 3,unsigned long GetSysTicks(void))
SYSFUNC(API_SYS, 4,int GetVersionString(char *VersionString,int MaxLen))
SYSFUNC(API_SYS, 5,int GetUSBType(void))
SYSFUNC(API_SYS, 6,int GetDeviceType(void))
SYSFUNC(API_SYS, 7,int Sleep(unsigned long Ticks,unsigned long Flags))
SYSFUNC(API_SYS, 8,void GetDeviceUID(byte *UID))
SYSFUNC(API_SYS, 9,bool SetParameters(const byte *TLV,int ByteCount))
SYSFUNC(API_SYS,10,unsigned int GetLastError(void))
SYSFUNC(API_SYS,11,int Diagnostic(int Mode,const void *In,int InLen,void *Out,int *OutLen,int MaxOutLen))

SYSFUNC(API_SYS,13,int GetProdSerNo(byte *SerNo,int MaxLen))
SYSFUNC(API_SYS,14,bool SetInterruptHandler(TInterruptHandler InterruptHandler,int IntNo))
SYSFUNC(API_SYS,15,void GetVersionInfo(TVersionInfo *VersionInfo))
SYSFUNC(API_SYS,16,bool ReadInfoValue(int Index,int FilterType,int *Type,int *Length,byte* Value,int MaxLength))
SYSFUNC(API_SYS,17,bool WriteInfoValue(int Type,int Length,const byte* Value))
SYSFUNC(API_SYS,18,bool GetCustomKeyID(byte *CustomKeyID,int *Length,int MaxLength))
SYSFUNC(API_SYS,19,bool GetParameters(const byte *Types,int TypeCount,byte *TLVBytes,int *TLVByteCount,int TLVMaxByteCount))

// ******************************************************************
// ****** Standard I/O Functions ************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define CHANNEL_NONE        0
#define CHANNEL_USB         1		// CDC or keyboard
#define CHANNEL_COM1        2
#define CHANNEL_COM2        3
#define CHANNEL_I2C         4
#define CHANNEL_COM3        5
#define CHANNEL_SPI         6
#define CHANNEL_COM4        7
#define CHANNEL_COM5        8
#define CHANNEL_CCID_DATA   0x10
#define CHANNEL_CCID_CTRL   0x11
#define CHANNEL_SAM1		0x20
#define CHANNEL_SAM2		0x21
#define CHANNEL_SAM3		0x22
#define CHANNEL_SAM4		0x23
#define CHANNEL_SC1			0x28
#define CHANNEL_RNG			0x30
#define CHANNEL_BLE			0x40
#define CHANNEL_HOST        0x7F

#define DIR_OUT             0
#define DIR_IN              1

#define COM_WORDLENGTH_8        ((byte)8)
#define COM_WORDLENGTH_9        ((byte)9)

#define COM_PARITY_NONE         ((byte)0)
#define COM_PARITY_ODD          ((byte)1)
#define COM_PARITY_EVEN         ((byte)2)

#define COM_STOPBITS_0_5        ((byte)1)
#define COM_STOPBITS_1          ((byte)2)
#define COM_STOPBITS_1_5        ((byte)3)
#define COM_STOPBITS_2          ((byte)4)

#define COM_FLOWCONTROL_NONE    ((byte)0)

#define COM_WORDLENGTH9_ESCAPEVALUE	((byte)179)

typedef struct __attribute__((__packed__))
{
    unsigned long BaudRate;
    byte WordLength;
    byte Parity;
    byte StopBits;
    byte FlowControl;
} TCOMParameters;

#define USB_DEVICE_STATE_NOTINIT        0
#define USB_DEVICE_STATE_DEFAULT        1
#define USB_DEVICE_STATE_ADDRESSED      2
#define USB_DEVICE_STATE_CONFIGURED     3
#define USB_DEVICE_STATE_SUSPENDED      4

#define CDC_CONTROL_LINE_STATE_DTR      0x01
#define CDC_CONTROL_LINE_STATE_RTS      0x02

#define CDC_SERIAL_STATE_DCD            0x01
#define CDC_SERIAL_STATE_DSR            0x02
#define CDC_SERIAL_STATE_RI             0x08
// Note: CTS is not supported by USB CDC device class

typedef struct __attribute__((__packed__))
{
    unsigned long dwDTERate;
    byte bCharFormat;
    byte bParityType;
    byte bDataBits;
} TCDCLineCoding;

#define CDC_STOPBITS_1          0
#define CDC_STOPBITS_1_5        1
#define CDC_STOPBITS_2          2

#define CDC_PARITY_NONE         0
#define CDC_PARITY_ODD          1
#define CDC_PARITY_EVEN         2
#define CDC_PARITY_MARK         3
#define CDC_PARITY_SPACE        4

#define CDC_DATABITS_5          5
#define CDC_DATABITS_6          6
#define CDC_DATABITS_7          7
#define CDC_DATABITS_8          8
#define CDC_DATABITS_16         16

#endif

SYSFUNC(API_IO, 0,void WriteByte(int Channel,byte Byte))
SYSFUNC(API_IO, 1,byte ReadByte(int Channel))
SYSFUNC(API_IO, 2,bool TestEmpty(int Channel,int Dir))
SYSFUNC(API_IO, 3,bool TestFull(int Channel,int Dir))
SYSFUNC(API_IO, 4,int  GetBufferSize(int Channel,int Dir))
SYSFUNC(API_IO, 5,int  GetByteCount(int Channel,int Dir))

SYSFUNC(API_IO, 9,bool SetCOMParameters(int Channel,TCOMParameters *COMParameters))
SYSFUNC(API_IO,10,int  GetUSBDeviceState(void))
SYSFUNC(API_IO,11,int  GetHostChannel(void))
SYSFUNC(API_IO,12,void USBRemoteWakeup(void))
SYSFUNC(API_IO,13,int  WriteBytes(int Channel, const byte* Bytes, int ByteCount))
SYSFUNC(API_IO,14,int  ReadBytes(int Channel, byte* Bytes, int ByteCount))

SYSFUNC(API_IO,15,bool InitChannel(int Channel))
SYSFUNC(API_IO,16,bool DeInitChannel(int Channel))
SYSFUNC(API_IO,17,bool SetHostChannel(int Channel))
SYSFUNC(API_IO,18,bool GetCDCLineCoding(TCDCLineCoding *CDCLineCoding))
SYSFUNC(API_IO,19,byte GetCDCControlLineState(void))
SYSFUNC(API_IO,20,void SetCDCSerialState(byte SerialState))

// ******************************************************************
// ****** Memory Functions ******************************************
// ******************************************************************

SYSFUNC(API_MEM, 0,bool CompBytes(const byte *Data1,const byte *Data2,int ByteCount))
SYSFUNC(API_MEM, 1,void CopyBytes(byte *DestBytes,const byte *SourceBytes,int ByteCount))
SYSFUNC(API_MEM, 2,void FillBytes(byte *Dest,byte Value,int ByteCount))
SYSFUNC(API_MEM, 3,void SwapBytes(byte *Data,int ByteCount))
SYSFUNC(API_MEM, 4,bool ReadBit(const byte *Byte,int BitNr))
SYSFUNC(API_MEM, 5,void WriteBit(byte *Byte,int BitNr,bool Value))
SYSFUNC(API_MEM, 6,void CopyBit(byte *Dest,int DestBitNr,const byte *Source,int SourceBitNr))
SYSFUNC(API_MEM, 7,bool CompBits(const byte *Data1,int Data1StartBit,const byte *Data2,int Data2StartBit,int BitCount))
SYSFUNC(API_MEM, 8,void CopyBits(byte *DestBits,int StartDestBit,const byte *SourceBits,int StartSourceBit,int BitCount))
SYSFUNC(API_MEM, 9,void FillBits(byte *Dest,int StartBit,bool Value,int BitCount))
SYSFUNC(API_MEM,10,void SwapBits(byte *Data,int StartBit,int BitCount))
SYSFUNC(API_MEM,11,int  CountBits(const byte *Data,int StartBit,bool Value,int BitCount))
SYSFUNC(API_MEM,12, uint16_t CalculateCRC16(const byte *Data, int Length, uint16_t InitValue))
SYSFUNC(API_MEM,13, uint32_t CalculateCRC32(const byte *Data, int Length, uint32_t InitValue))
SYSFUNC(API_MEM,14, byte CalculateChecksum(const byte *Data, int Length))

// ******************************************************************
// ****** Conversion Functions **************************************
// ******************************************************************

SYSFUNC(API_CONV, 0,int ScanHexChar(byte Char))
SYSFUNC(API_CONV, 1,int ScanHexString(byte *ASCII,int ByteCount))
SYSFUNC(API_CONV, 2,int ConvertBinaryToString(const byte *SourceBits,int StartBit,int BitCnt,char *String,int Radix,int MinDigits,int MaxDigits))
SYSFUNC(API_CONV, 3,bool ConvertStringToBinary(const char *String,int StringLength,byte *DestBytes,int DestByteCount,int Radix))
SYSFUNC(API_CONV, 4,bool ConvertDigitsToBinary(const byte *SourceBytes,int SourceByteCnt,int SourceBitsPerDigit,int SourceDigitCnt,byte *DestBytes,int DestByteCnt,int Radix))

// ******************************************************************
// ****** Peripheral Functions **************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

// ****** GPIOs *****************************************************

// Bitmasks of GPIOs
#define GPIO0                   0x0001
#define GPIO1                   0x0002
#define GPIO2                   0x0004
#define GPIO3                   0x0008
#define GPIO4                   0x0010
#define GPIO5                   0x0020
#define GPIO6                   0x0040
#define GPIO7                   0x0080
#define GPIO_COM4_RESET         0x0100
#define GPIO_COM5_RESET         0x0200

// GPIO Pullup/Pulldown
#define GPIO_PUPD_NOPULL        0
#define GPIO_PUPD_PULLUP        1
#define GPIO_PUPD_PULLDOWN      2

// GPIO Output Type
#define GPIO_OTYPE_PUSHPULL     0
#define GPIO_OTYPE_OPENDRAIN    1

// GPIO Trigger Interrupt
#define TRIGGER_RISING				0
#define TRIGGER_FALLING				1
#define TRIGGER_RISING_FALLING		2

// ****** Access to LEDs ********************************************

#define REDLED                  GPIO0
#define GREENLED                GPIO1
#define YELLOWLED               GPIO2
#define BLUELED                 GPIO2

// ****** Beeper ****************************************************

#define BEEP_FREQUENCY_HIGH     2400
#define BEEP_FREQUENCY_LOW      2057

#define BEEP_NEVER_STOP         -1

// ****** Sequencer *************************************************

#define SEQ_FUNC(x)			( x         << 24)
#define SEQ_CTRL(x)			((x | 0x80) << 24)

#define SEQ_SETBITS			SEQ_FUNC(2)
#define SEQ_CLEARBITS		SEQ_FUNC(3)
#define SEQ_TOGGLEBITS		SEQ_FUNC(4)

#define SEQ_STOP			SEQ_CTRL(0)
#define SEQ_WAIT			SEQ_CTRL(1)
#define SEQ_SETCOUNTER		SEQ_CTRL(2)
#define SEQ_JUMP			SEQ_CTRL(3)
#define SEQ_DECJMPNZ		SEQ_CTRL(4)

// ****** Play Melody ***********************************************

// Notes

#define PM_C3               1047
#define PM_CIS3             1109
#define PM_DES3             1109
#define PM_D3               1175
#define PM_DIS3             1245
#define PM_ES3              1245
#define PM_E3               1319
#define PM_F3               1397
#define PM_FIS3             1480
#define PM_GES3             1480
#define PM_G3               1568
#define PM_GIS3             1661
#define PM_AES3             1661
#define PM_A3               1760
#define PM_B3               1865
#define PM_H3               1976
#define PM_C4               2093
#define PM_CIS4             2217
#define PM_DES4             2217
#define PM_D4               2349
#define PM_DIS4             2489
#define PM_ES4              2489
#define PM_E4               2637
#define PM_F4               2794
#define PM_FIS4             2960
#define PM_GES4             2960
#define PM_G4               3136
#define PM_GIS4             3322
#define PM_AES4             3322
#define PM_A4               3520
#define PM_AIS4             3729
#define PM_B4               3729
#define PM_H4               3951
#define PM_C5               4186
#define PM_CIS5             4435
#define PM_DES5             4435
#define PM_D5               4699
#define PM_DIS5             4978
#define PM_ES5              4978
#define PM_E5               5274
#define PM_F5               5588

// Command codes

#define PM_CMD_END			0
#define PM_CMD_KEYDOWN      1
#define PM_CMD_KEYUP        2
#define PM_CMD_DELAY        3

// Commands

#define PM_END                          PM_CMD_END
#define PM_KEYDOWN(Voice,Frequency)     PM_CMD_KEYDOWN,Voice,CONVERT_INT16_TO_BYTES_LSB(Frequency)
#define PM_KEYUP(Voice)                 PM_CMD_KEYUP,Voice
#define PM_DELAY(Time)                  PM_CMD_DELAY,CONVERT_INT16_TO_BYTES_LSB(Time)

#endif

SYSFUNC(API_PERIPH, 0,void GPIOConfigureOutputs(int Bits,int PullUpDown,int OutputType))
SYSFUNC(API_PERIPH, 1,void GPIOConfigureInputs(int Bits,int PullUpDown))
SYSFUNC(API_PERIPH, 2,void GPIOSetBits(int Bits))
SYSFUNC(API_PERIPH, 3,void GPIOClearBits(int Bits))
SYSFUNC(API_PERIPH, 4,void GPIOToggleBits(int Bits))
SYSFUNC(API_PERIPH, 5,void GPIOBlinkBits(int Bits,int TimeHi,int TimeLo))
SYSFUNC(API_PERIPH, 6,int  GPIOTestBit(int Bit))
SYSFUNC(API_PERIPH, 7,void Beep(int Volume,int Frequency,int OnTime,int OffTime))
SYSFUNC(API_PERIPH, 8,void DiagLEDOn(void))
SYSFUNC(API_PERIPH, 9,void DiagLEDOff(void))
SYSFUNC(API_PERIPH,10,void DiagLEDToggle(void))
SYSFUNC(API_PERIPH,11,bool DiagLEDIsOn(void))
SYSFUNC(API_PERIPH,12,void SendWiegand(int GPIOData0,int GPIOData1,int PulseTime,int IntervalTime,const byte *Bits,int BitCount))
SYSFUNC(API_PERIPH,13,void SendOmron(int GPIOClock,int GPIOData,int T1,int T2,int T3,const byte *Bits,int BitCount))
SYSFUNC(API_PERIPH,14,bool GPIOPlaySequence(const int *NewSequence,int ByteCount))
SYSFUNC(API_PERIPH,15,void GPIOStopSequence(void))
SYSFUNC(API_PERIPH,16,void LEDInit(int LEDs))
SYSFUNC(API_PERIPH,17,void LEDOn(int LEDs))
SYSFUNC(API_PERIPH,18,void LEDOff(int LEDs))
SYSFUNC(API_PERIPH,19,void LEDToggle(int LEDs))
SYSFUNC(API_PERIPH,20,void LEDBlink(int LEDs,int TimeOn,int TimeOff))
SYSFUNC(API_PERIPH,21,bool GPIOConfigureInterrupt(int GPIOBits,bool Enable,int Edge))
SYSFUNC(API_PERIPH,22,void BeepOn(int Volume,int Frequency))
SYSFUNC(API_PERIPH,23,void BeepOff(void))
SYSFUNC(API_PERIPH,24,void PlayMelody(const byte *Melody,int MelodyLength))

// ******************************************************************
// ****** RF Tag Functions ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define NOTAG               0
// LF Tags
#define LFTAG_EM4102        0x40    // "EM4x02/CASI-RUSCO" (aka IDRO_A)
#define LFTAG_HITAG1S       0x41    // "HITAG 1/HITAG S"   (aka IDRW_B)
#define LFTAG_HITAG2        0x42    // "HITAG 2"           (aka IDRW_C)
#define LFTAG_EM4150        0x43    // "EM4x50"            (aka IDRW_D)
#define LFTAG_AT5555        0x44    // "T55x7"             (aka IDRW_E)
#define LFTAG_ISOFDX        0x45    // "ISO FDX-B"         (aka IDRO_G)
#define LFTAG_EM4026        0x46    // N/A                 (aka IDRO_H)
#define LFTAG_HITAGU        0x47    // N/A                 (aka IDRW_I)
#define LFTAG_EM4305        0x48    // "EM4305"            (aka IDRW_K)
#define LFTAG_HIDPROX       0x49	// "HID Prox"
#define LFTAG_TIRIS         0x4A	// "ISO HDX/TIRIS"
#define LFTAG_COTAG         0x4B	// "Cotag"
#define LFTAG_IOPROX        0x4C	// "ioProx"
#define LFTAG_INDITAG       0x4D	// "Indala"
#define LFTAG_HONEYTAG      0x4E	// "NexWatch"
#define LFTAG_AWID      	0x4F	// "AWID"
#define LFTAG_GPROX      	0x50	// "G-Prox"
#define LFTAG_PYRAMID      	0x51	// "Pyramid"
#define LFTAG_KERI      	0x52	// "Keri"
#define LFTAG_DEISTER		0x53	// "Deister"
#define LFTAG_CARDAX		0x54	// "Cardax"
#define LFTAG_NEDAP			0x55	// "Nedap"
#define LFTAG_PAC			0x56	// "PAC"
#define LFTAG_IDTECK		0x57	// "IDTECK"
#define LFTAG_ULTRAPROX		0x58	// "UltraProx"
#define LFTAG_ICT			0x59	// "ICT"
#define LFTAG_ISONAS		0x5A	// "Isonas"
// HF Tags
#define HFTAG_MIFARE        0x80	// "ISO14443A/MIFARE"
#define HFTAG_ISO14443B     0x81	// "ISO14443B"
#define HFTAG_ISO15693      0x82	// "ISO15693"
#define HFTAG_LEGIC         0x83	// "LEGIC"
#define HFTAG_HIDICLASS     0x84	// "HID iCLASS"
#define HFTAG_FELICA        0x85	// "FeliCa"
#define HFTAG_SRX           0x86	// "SRX"
#define HFTAG_NFCP2P        0x87	// "NFC Peer-to-Peer"
#define HFTAG_BLE        	0x88	// "Bluetooth Low Energy"
#define HFTAG_TOPAZ         0x89    // "Topaz"
#define HFTAG_CTS           0x8A    // "CTS256 / CTS512"
#define HFTAG_BLELC        	0x8B	// "Bluetooth Low Energy LEGIC Connect"

#define TAGMASK(Tagtype)    (Tagtype == NOTAG ? 0 : 1 << (Tagtype & 0x1F))

#define ALL_LFTAGS          0xFFFFFFFF
#define ALL_HFTAGS          0xFFFFFFFF

#endif

SYSFUNC(API_RF, 0,bool SearchTag(int *TagType,int *IDBitCount,byte *ID,int MaxIDBytes))
SYSFUNC(API_RF, 1,void SetRFOff(void))
SYSFUNC(API_RF, 2,void SetTagTypes(unsigned int LFTagTypes,unsigned int HFTagTypes))
SYSFUNC(API_RF, 3,void GetTagTypes(unsigned int *LFTagTypes,unsigned int *HFTagTypes))
SYSFUNC(API_RF, 4,void GetSupportedTagTypes(unsigned int *LFTagTypes,unsigned int *HFTagTypes))

// ******************************************************************
// ****** TILF Functions ********************************************
// ******************************************************************

SYSFUNC(API_TILF, 0,bool TILF_SearchTag(int *IDBitCount,byte *ID,int MaxIDBytes))
SYSFUNC(API_TILF, 1,bool TILF_ChargeOnlyRead(byte *ReadData))
SYSFUNC(API_TILF, 2,bool TILF_ChargeOnlyReadLo(byte *ReadData))
SYSFUNC(API_TILF, 3,bool TILF_SPProgramPage(const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF, 4,bool TILF_SPProgramPageLo(const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF, 5,bool TILF_MPGeneralReadPage(int Address,byte *ReadData))
SYSFUNC(API_TILF, 6,bool TILF_MPSelectiveReadPage(int Address,const byte *SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF, 7,bool TILF_MPProgramPage(int Address,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF, 8,bool TILF_MPSelectiveProgramPage(int Address,const byte *SelectiveAddress,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF, 9,bool TILF_MPLockPage(int Address,byte *ReadData))
SYSFUNC(API_TILF,10,bool TILF_MPSelectiveLockPage(int Address,const byte *SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF,11,bool TILF_MPGeneralReadPageLo(int Address,byte *ReadData))
SYSFUNC(API_TILF,12,bool TILF_MPSelectiveReadPageLo(int Address,const byte *SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF,13,bool TILF_MPProgramPageLo(int Address,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF,14,bool TILF_MPSelectiveProgramPageLo(int Address,const byte *SelectiveAddress,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF,15,bool TILF_MPLockPageLo(int Address,byte *ReadData))
SYSFUNC(API_TILF,16,bool TILF_MPSelectiveLockPageLo(int Address,const byte *SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF,17,bool TILF_MUGeneralReadPage(int Address,byte *ReadData))
SYSFUNC(API_TILF,18,bool TILF_MUSelectiveReadPage(int Address,int SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF,19,bool TILF_MUSpecialReadPage(int Address,const byte *SpecialAddress1,const byte *SpecialAddress2,byte *ReadData))
SYSFUNC(API_TILF,20,bool TILF_MUProgramPage(int Address,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF,21,bool TILF_MUSelectiveProgramPage(int Address,int SelectiveAddress,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF,22,bool TILF_MUSpecialProgramPage(int Address,const byte *SpecialAddress1,const byte *SpecialAddress2,const byte *WriteData,byte *ReadData))
SYSFUNC(API_TILF,23,bool TILF_MULockPage(int Address,byte *ReadData))
SYSFUNC(API_TILF,24,bool TILF_MUSelectiveLockPage(int Address,int SelectiveAddress,byte *ReadData))
SYSFUNC(API_TILF,25,bool TILF_MUSpecialLockPage(int Address,const byte *SpecialAddress1,const byte *SpecialAddress2,byte *ReadData))

// ******************************************************************
// ****** Hitag 1/S Functions ***************************************
// ******************************************************************

SYSFUNC(API_HITAG1S, 0,bool Hitag1S_SearchTag(int *IDBitCount,byte *ID,int MaxIDBytes))
SYSFUNC(API_HITAG1S, 1,bool Hitag1S_ReadPage(int PageAddress,byte *Page))
SYSFUNC(API_HITAG1S, 2,bool Hitag1S_ReadBlock(int BlockAddress,byte *Block,byte *BytesRead))
SYSFUNC(API_HITAG1S, 3,bool Hitag1S_WritePage(int PageAddress,const byte *Page))
SYSFUNC(API_HITAG1S, 4,bool Hitag1S_WriteBlock(int BlockAddress,const byte *Block,byte *BytesWritten))
SYSFUNC(API_HITAG1S, 5,bool Hitag1S_Halt(void))

// ******************************************************************
// ****** Hitag 2 Functions *****************************************
// ******************************************************************

SYSFUNC(API_HITAG2, 0,bool Hitag2_SearchTag(int *IDBitCount,byte *ID,int MaxIDBytes))
SYSFUNC(API_HITAG2, 1,bool Hitag2_ReadPage(int PageAddress,byte *Page))
SYSFUNC(API_HITAG2, 2,bool Hitag2_WritePage(int PageAddress,const byte *Page))
SYSFUNC(API_HITAG2, 3,bool Hitag2_Halt(void))
SYSFUNC(API_HITAG2, 4,void Hitag2_SetPassword(const byte *Password))

// ******************************************************************
// ****** SM4X00/SM6X00 Functions ***********************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

// Provide macros for obsolete names of functions
#define SM4200_GenericRaw(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)  SM4X00_GenericRaw(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)
#define SM4200_Generic(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)     SM4X00_Generic(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)
#define SM4200_StartBootloader(TLV,TLVLength,MaxTLVLength)                                  SM4X00_StartBootloader(TLV,TLVLength,MaxTLVLength)
#define SM4200_EraseFlash()                                                                 SM4X00_EraseFlash()
#define SM4200_ProgramBlock(Data,Done)                                                      SM4X00_ProgramBlock(Data,Done)

// Provide more macros for more obsolete names of function
#define SM4X00_GenericRaw(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)  SM_GenericRaw(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)
#define SM4X00_Generic(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)     SM_Generic(TXData,TXDataLength,RXData,RXDataLength,MaxRXDataLength,Timeout)
#define SM4X00_StartBootloader(TLV,TLVLength,MaxTLVLength)                                  SM_4x00StartBootloader(TLV,TLVLength,MaxTLVLength)
#define SM4X00_EraseFlash()                                                                 SM_4x00EraseFlash()
#define SM4X00_ProgramBlock(Data,Done)                                                      SM_4x00ProgramBlock(Data,Done)

#endif

SYSFUNC(API_SM, 0,bool SM_GenericRaw(const byte *TXData,int TXDataLength,byte *RXData,int *RXDataLength,int MaxRXDataLength,int Timeout))
SYSFUNC(API_SM, 1,bool SM_Generic(const byte *TXData,int TXDataLength,byte *RXData,int *RXDataLength,int MaxRXDataLength,int Timeout))
SYSFUNC(API_SM, 2,bool SM_4x00StartBootloader(byte *TLV,int *TLVLength,int MaxTLVLength))
SYSFUNC(API_SM, 3,bool SM_4x00EraseFlash(void))
SYSFUNC(API_SM, 4,bool SM_4x00ProgramBlock(byte *Data,bool *Done))

// ******************************************************************
// ****** I2C Functions *********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define I2CMODE_MASTER              0x0000    // 0000000000000000
#define I2CMODE_SLAVE               0x1000    // 0001000000000000
#define I2CMODE_CHANNEL             0x2000    // 0010000000000000
#define I2CMODE_MASTER_400k         0x4000    // 0100000000000000
#define I2CMODE_ADDRESS_MASK        0x007F    // 0000000001111111

#define I2C_ERROR_BUSERROR          0x000100  // Bus Error
#define I2C_ERROR_ARLO              0x000200  // Arbitation Lost
#define I2C_ERROR_AF                0x000400  // ACK failure
#define I2C_ERROR_OVR               0x000800  // Overrun/Unerrun
#define I2C_ERROR_PECERR            0x001000  // PEC Error in reception
#define I2C_PACKAGE_MAX_SIZE        100
#define I2C_PACKAGE_MAX_PACKAGENUM  100

typedef struct __attribute__((__packed__))
{
  byte SlaveAddress;
  byte PackageSize;
  byte Data[I2C_PACKAGE_MAX_SIZE];
} TI2CPackage;

#endif

SYSFUNC(API_I2C, 0,bool I2CInit(int Mode))
SYSFUNC(API_I2C, 1,void I2CDeInit(void))
SYSFUNC(API_I2C, 2,bool I2CMasterStart(void))
SYSFUNC(API_I2C, 3,void I2CMasterStop(void))
SYSFUNC(API_I2C, 4,bool I2CMasterTransmitByte(byte Byte))
SYSFUNC(API_I2C, 5,byte I2CMasterReceiveByte(void))
SYSFUNC(API_I2C, 6,void I2CMasterBeginWrite(int Address))
SYSFUNC(API_I2C, 7,void I2CMasterBeginRead(int Address))
SYSFUNC(API_I2C, 8,void I2CMasterSetAck(bool SetOn))
SYSFUNC(API_I2C, 9,bool I2CReceiveAsync(byte Address, byte RecSize, TInterruptHandler RCallback))
SYSFUNC(API_I2C, 10,bool I2CSendAsync(byte Address, const byte* Data, byte DataSize, bool GenerateSTOP, TInterruptHandler SCallback))
SYSFUNC(API_I2C, 11,int I2CAsyncGetNumMessages(void))
SYSFUNC(API_I2C, 12,int I2CAsyncGetMessage(byte *SlaveAddress, byte *Message, int MaxSize))
SYSFUNC(API_I2C, 13,void I2CForceStop(void))
SYSFUNC(API_I2C, 14,int I2CGetLastError(void))

// ******************************************************************
// ****** Mifare Classic Functions **********************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define KEYA    0
#define KEYB    1

#endif

SYSFUNC(API_MIFARECLASSIC, 0, bool MifareClassic_Login(const byte* Key, byte KeyType, int Sector))
SYSFUNC(API_MIFARECLASSIC, 1, bool MifareClassic_ReadBlock(int Block, byte* Data))
SYSFUNC(API_MIFARECLASSIC, 2, bool MifareClassic_WriteBlock(int Block, const byte* Data))
SYSFUNC(API_MIFARECLASSIC, 3, bool MifareClassic_ReadValueBlock(int Block, int* Value))
SYSFUNC(API_MIFARECLASSIC, 4, bool MifareClassic_WriteValueBlock(int Block, int Value))
SYSFUNC(API_MIFARECLASSIC, 5, bool MifareClassic_IncrementValueBlock(int Block, int Value))
SYSFUNC(API_MIFARECLASSIC, 6, bool MifareClassic_DecrementValueBlock(int Block, int Value))
SYSFUNC(API_MIFARECLASSIC, 7, bool MifareClassic_CopyValueBlock(int SourceBlock, int DestBlock))

// ******************************************************************
// ****** Mifare Ultralight Functions *******************************
// ******************************************************************

SYSFUNC(API_MIFAREULTRALIGHT, 0, bool MifareUltralight_ReadPage(int Page, byte* Data))
SYSFUNC(API_MIFAREULTRALIGHT, 1, bool MifareUltralight_WritePage(int Page, const byte* Data))
SYSFUNC(API_MIFAREULTRALIGHT, 2, bool MifareUltralightC_Authenticate(const byte* Key))
SYSFUNC(API_MIFAREULTRALIGHT, 3, bool MifareUltralightC_SAMAuthenticate(int KeyNo, int KeyVersion, const byte* DIVInput, int DIVByteCnt))
SYSFUNC(API_MIFAREULTRALIGHT, 4, bool MifareUltralightC_WriteKeyFromSAM(int KeyNo, int KeyVersion, const byte* DIVInput, int DIVByteCnt))
SYSFUNC(API_MIFAREULTRALIGHT, 5, bool MifareUltralightEV1_FastRead(int StartPage, int NumberOfPages, byte* Data))
SYSFUNC(API_MIFAREULTRALIGHT, 6, bool MifareUltralightEV1_IncCounter(int CounterAddr, int IncrValue))
SYSFUNC(API_MIFAREULTRALIGHT, 7, bool MifareUltralightEV1_ReadCounter(int CounterAddr, int* CounterValue))
SYSFUNC(API_MIFAREULTRALIGHT, 8, bool MifareUltralightEV1_ReadSig(byte* ECCSig))
SYSFUNC(API_MIFAREULTRALIGHT, 9, bool MifareUltralightEV1_GetVersion(byte* Version))
SYSFUNC(API_MIFAREULTRALIGHT, 10, bool MifareUltralightEV1_PwdAuth(const byte* Password, const byte* PwdAck))
SYSFUNC(API_MIFAREULTRALIGHT, 11, bool MifareUltralightEV1_CheckTearingEvent(int CounterAddr, byte* ValidFlag))

// ******************************************************************
// ****** ISO15693 Functions ****************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

// ISO15693 Tag Types
#define ISO15693_TAGTYPE_ICODESL2			0x00
#define ISO15693_TAGTYPE_ICODESL2S			0x01
#define ISO15693_TAGTYPE_UNKNOWNNXP			0x0F
#define ISO15693_TAGTYPE_TAGITHFIPLUSINLAY	0x10
#define ISO15693_TAGTYPE_TAGITHFIPLUSCHIP	0x11
#define ISO15693_TAGTYPE_TAGITHFISTD		0x12
#define ISO15693_TAGTYPE_TAGITHFIPRO		0x13
#define ISO15693_TAGTYPE_UNKNOWNTI			0x1F
#define ISO15693_TAGTYPE_MB89R118			0x20
#define ISO15693_TAGTYPE_MB89R119			0x21
#define ISO15693_TAGTYPE_MB89R112			0x22
#define ISO15693_TAGTYPE_UNKNOWNFUJI		0x2F
#define ISO15693_TAGTYPE_24LR16				0x30
#define ISO15693_TAGTYPE_24LR64				0x31
#define ISO15693_TAGTYPE_LRI1K				0x40
#define ISO15693_TAGTYPE_LRI2K				0x41
#define ISO15693_TAGTYPE_LRIS2K				0x42
#define ISO15693_TAGTYPE_LRIS64K			0x43
#define ISO15693_TAGTYPE_UNKNOWNST			0x4F
#define ISO15693_TAGTYPE_SRF55V02P			0x50
#define ISO15693_TAGTYPE_SRF55V10P			0x51
#define ISO15693_TAGTYPE_SRF55V02S			0x52
#define ISO15693_TAGTYPE_SRF55V10S			0x53
#define ISO15693_TAGTYPE_UNKNOWNINFINEON	0x5F
#define ISO15693_TAGTYPE_UNKNOWN			0xFF

typedef struct __attribute__((__packed__))
{
	byte DSFID_Present:1;
	byte AFI_Present:1;
	byte VICC_Memory_Size_Present:1;
	byte IC_Reference_Present:1;
	byte Res1:4;
	byte UID[8];
	byte DSFID;
	byte AFI;
	byte BlockSize;
	uint16_t Number_of_Blocks;
	byte IC_Reference;
} TISO15693_SystemInfo;

#endif

SYSFUNC(API_ISO15693, 0, bool ISO15693_GenericCommand(byte Flags, byte Command, byte* Data, int* Length, int BufferSize))
SYSFUNC(API_ISO15693, 1, bool ISO15693_GetSystemInformation(TISO15693_SystemInfo* SystemInfo))
SYSFUNC(API_ISO15693, 2, bool ISO15693_GetSystemInformationExt(TISO15693_SystemInfo* SystemInfo))
SYSFUNC(API_ISO15693, 3, int  ISO15693_GetTagTypeFromUID(byte* UID))
SYSFUNC(API_ISO15693, 4, int  ISO15693_GetTagTypeFromSystemInfo(TISO15693_SystemInfo* SystemInfo))
SYSFUNC(API_ISO15693, 5, bool ISO15693_ReadSingleBlock(int BlockNumber, byte* BlockData, int* Length, int BufferSize))
SYSFUNC(API_ISO15693, 6, bool ISO15693_ReadSingleBlockExt(int BlockNumber, byte* BlockData, int* Length, int BufferSize))
SYSFUNC(API_ISO15693, 7, bool ISO15693_WriteSingleBlock(int BlockNumber, const byte* BlockData, int Length))
SYSFUNC(API_ISO15693, 8, bool ISO15693_WriteSingleBlockExt(int BlockNumber, const byte* BlockData, int Length))

// ******************************************************************
// ****** Crypto Functions ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define CRYPTO_ENV0         		0
#define CRYPTO_ENV1         		1
#define CRYPTO_ENV2         		2
#define CRYPTO_ENV3         		3
#define CRYPTO_ENV_CNT      		4

#define CRYPTOMODE_AES128			0
#define CRYPTOMODE_AES192			1
#define CRYPTOMODE_AES256			2
#define CRYPTOMODE_3DES				3
#define CRYPTOMODE_3K3DES			4
#define CRYPTOMODE_CBC_DES			5
#define CRYPTOMODE_CBC_DFN_DES		6
#define CRYPTOMODE_CBC_3DES			7
#define CRYPTOMODE_CBC_DFN_3DES		8
#define CRYPTOMODE_CBC_3K3DES		9
#define CRYPTOMODE_CBC_AES128		10
#define CRYPTOMODE_CBC_AES192       11
#define CRYPTOMODE_CBC_AES256       12
#define CRYPTOMODE_CTR_AES128       13

#endif

#define SHA1                        0
#define SHA256                      1

SYSFUNC(API_CRYPTO, 0, void Crypto_Init(int CryptoEnv, int CryptoMode, const byte* Key, int KeyByteCnt))
SYSFUNC(API_CRYPTO, 1, void Encrypt(int CryptoEnv, const byte* PlainBlock, byte* CipheredBlock, int BlockByteCnt))
SYSFUNC(API_CRYPTO, 2, void Decrypt(int CryptoEnv, const byte* CipheredBlock, byte* PlainBlock, int BlockByteCnt))
SYSFUNC(API_CRYPTO, 3, void CBC_ResetInitVector(int CryptoEnv))
SYSFUNC(API_CRYPTO, 4, bool DecryptSecrets(const byte *CipheredData,byte *PlainData,int ByteCnt))
SYSFUNC(API_CRYPTO, 5, bool Crypto_InitByKeyIndex(int CryptoEnv, int CryptoMode, int KeyIndex))
SYSFUNC(API_CRYPTO, 6, void SHAx_Init(int WhichSHA))
SYSFUNC(API_CRYPTO, 7, void SHAx_Update(const byte* Message, int MessageByteCnt))
SYSFUNC(API_CRYPTO, 8, void SHAx_Final(byte* Digest, int* DigestByteCnt, int MaxDigestByteCnt))


// ******************************************************************
// ****** DESFire Functions *****************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define DESF_FILETYPE_STDDATAFILE           0
#define DESF_FILETYPE_BACKUPDATAFILE        1
#define DESF_FILETYPE_VALUEFILE             2
#define DESF_FILETYPE_LINEARRECORDFILE      3
#define DESF_FILETYPE_CYCLICRECORDFILE      4

#define DESF_COMMSET_PLAIN                  0
#define DESF_COMMSET_PLAIN_MACED            1
#define DESF_COMMSET_FULLY_ENC              3

#define DESF_AUTHMODE_COMPATIBLE            0
#define DESF_AUTHMODE_EV1                   1

#define DESF_KEYTYPE_3DES                   0
#define DESF_KEYTYPE_3K3DES                 1
#define DESF_KEYTYPE_AES                    2

#define DESF_KEYLEN_3DES                   	16
#define DESF_KEYLEN_3K3DES                 	24
#define DESF_KEYLEN_AES                    	16

typedef struct __attribute__((__packed__))
{
	struct TDESFireKeySettings
	{
		byte AllowChangeMasterKey:1;
		byte FreeDirectoryList:1;
		byte FreeCreateDelete:1;
		byte ConfigurationChangeable:1;
		byte ChangeKeyAccessRights:4;
	} KeySettings;
	int NumberOfKeys;
	int KeyType;
} TDESFireMasterKeySettings;

typedef struct __attribute__((__packed__))
{
	byte FileType;
	byte CommSet;
	uint16_t AccessRights;
	union TDESFireSpecificFileInfo
	{
		struct TDESFireDataFileSettings
		{
			uint32_t FileSize;
		} DataFileSettings;
		struct TDESFireValueFileSettings
		{
			uint32_t LowerLimit;
			uint32_t UpperLimit;
			uint32_t LimitedCreditValue;
			struct TValueFileOptions
			{
			    byte LimitedCreditEnabled:1;
			    byte FreeGetValue:1;
			    byte RFU:6;
			} ValueFileOptions;
		} ValueFileSettings;
		struct TDESFireRecordFileSettings
		{
			uint32_t RecordSize;
			uint32_t MaxNumberOfRecords;
			uint32_t CurrentNumberOfRecords;
		} RecordFileSettings;
	}	SpecificFileInfo;
} TDESFireFileSettings;

typedef struct __attribute__((__packed__))
{
	byte VendorID;
	byte Type;
	byte SubType;
	byte VersionMajor;
	byte VersionMinor;
	uint32_t StorageSize;
	byte CommunicationProtocol;
} TDESFireTagInfo;

typedef struct __attribute__((__packed__))
{
	byte UID[7];
	byte ProdBatchNumber[5];
	byte CalendarWeekOfProduction;
	byte YearOfProduction;
} TDESFireProdInfo;

typedef struct __attribute__((__packed__))
{
	TDESFireTagInfo HWInfo;
	TDESFireTagInfo SWInfo;
	TDESFireProdInfo ProdInfo;
} TDESFireVersion;

#endif

SYSFUNC(API_DESFIRE, 0, bool DESFire_GetApplicationIDs(int CryptoEnv, int* AIDs, int* NumberOfAIDs, int MaxAIDCnt))
SYSFUNC(API_DESFIRE, 1, bool DESFire_CreateApplication(int CryptoEnv, int AID, const TDESFireMasterKeySettings* MasterKeySettings))
SYSFUNC(API_DESFIRE, 2, bool DESFire_DeleteApplication(int CryptoEnv, int AID))
SYSFUNC(API_DESFIRE, 3, bool DESFire_SelectApplication(int CryptoEnv, int AID))
SYSFUNC(API_DESFIRE, 4, bool DESFire_Authenticate(int CryptoEnv, int KeyNoTag, const byte* Key, int KeyByteCount, int KeyType, int Mode))
SYSFUNC(API_DESFIRE, 5, bool DESFire_GetKeySettings(int CryptoEnv, TDESFireMasterKeySettings* MasterKeySettings))
SYSFUNC(API_DESFIRE, 6, bool DESFire_GetFileIDs(int CryptoEnv, byte* FileIDList, int* FileIDCount, int MaxFileIDCount))
SYSFUNC(API_DESFIRE, 7, bool DESFire_GetFileSettings(int CryptoEnv, int FileNo, TDESFireFileSettings* FileSettings))
SYSFUNC(API_DESFIRE, 8, bool DESFire_ReadData (int CryptoEnv, int FileNo, byte* Data, int Offset, int Length, int CommSet))
SYSFUNC(API_DESFIRE, 9, bool DESFire_WriteData(int CryptoEnv, int FileNo, const byte* Data, int Offset, int Length, int CommSet))
SYSFUNC(API_DESFIRE,10, bool DESFire_GetValue(int CryptoEnv, int FileNo, int* Value, int CommSet))
SYSFUNC(API_DESFIRE,11, bool DESFire_Credit(int CryptoEnv, int FileNo, const int Value, int CommSet))
SYSFUNC(API_DESFIRE,12, bool DESFire_Debit(int CryptoEnv, int FileNo, const int Value, int CommSet))
SYSFUNC(API_DESFIRE,13, bool DESFire_LimitedCredit(int CryptoEnv, int FileNo, const int Value, int CommSet))
SYSFUNC(API_DESFIRE,14, bool DESFire_FreeMem(int CryptoEnv, int* FreeMemory))
SYSFUNC(API_DESFIRE,15, bool DESFire_FormatTag(int CryptoEnv))
SYSFUNC(API_DESFIRE,16, bool DESFire_CreateDataFile(int CryptoEnv, int FileNo, const TDESFireFileSettings* FileSettings))
SYSFUNC(API_DESFIRE,17, bool DESFire_CreateValueFile(int CryptoEnv, int FileNo, const TDESFireFileSettings* FileSettings))
SYSFUNC(API_DESFIRE,18, bool DESFire_GetVersion(int CryptoEnv, TDESFireVersion* Version))
SYSFUNC(API_DESFIRE,19, bool DESFire_DeleteFile(int CryptoEnv, int FileNo))
SYSFUNC(API_DESFIRE,20, bool DESFire_CommitTransaction(int CryptoEnv))
SYSFUNC(API_DESFIRE,21, bool DESFire_AbortTransaction(int CryptoEnv))
SYSFUNC(API_DESFIRE,22, bool DESFire_GetUID(int CryptoEnv, byte* UID, int* Length, int BufferSize))
SYSFUNC(API_DESFIRE,23, bool DESFire_GetKeyVersion(int CryptoEnv, int KeyNo, byte* KeyVer))
SYSFUNC(API_DESFIRE,24, bool DESFire_ChangeKeySettings(int CryptoEnv, const TDESFireMasterKeySettings* MasterKeySettings))
SYSFUNC(API_DESFIRE,25, bool DESFire_ChangeKey(int CryptoEnv, int KeyNo, const byte* OldKey, int OldKeyByteCount, const byte* NewKey, int NewKeyByteCount, byte KeyVersion, const TDESFireMasterKeySettings* MasterKeySettings))
SYSFUNC(API_DESFIRE,26, bool DESFire_ChangeFileSettings(int CryptoEnv, int FileNo, int NewCommSet, int OldAccessRights, int NewAccessRights))
SYSFUNC(API_DESFIRE,27, bool DESFire_DisableFormatTag(int CryptoEnv))
SYSFUNC(API_DESFIRE,28, bool DESFire_EnableRandomID(int CryptoEnv))
SYSFUNC(API_DESFIRE,29, bool DESFire_SetDefaultKey(int CryptoEnv, const byte* Key, int KeyByteCount, byte KeyVersion))
SYSFUNC(API_DESFIRE,30, bool DESFire_SetATS(int CryptoEnv, const byte* ATS, int Length))
SYSFUNC(API_DESFIRE,31, bool DESFire_CreateRecordFile(int CryptoEnv, int FileNo, const TDESFireFileSettings* FileSettings))
SYSFUNC(API_DESFIRE,32, bool DESFire_ReadRecords(int CryptoEnv, int FileNo, byte* RecordData, int* RecDataByteCnt, int Offset, int NumberOfRecords, int RecordSize, int CommSet))
SYSFUNC(API_DESFIRE,33, bool DESFire_WriteRecord(int CryptoEnv, int FileNo, const byte* Data, int Offset, int Length, int CommSet))
SYSFUNC(API_DESFIRE,34, bool DESFire_ClearRecordFile(int CryptoEnv, int FileNo))

// ******************************************************************
// ****** SAM Functions *********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define ISO7816_PROTOCOL_T0				        0
#define ISO7816_PROTOCOL_T1				        1

#define ISO7816_APDU_MAXDATASIZE		        256

#define ISO7816_ERR_ICC_MUTE			        0xFE
#define ISO7816_ERR_XFR_PARITY			        0xFD
#define ISO7816_ERR_HW_ERROR			        0xFB
#define ISO7816_ERR_BADATRTS			        0xF8
#define ISO7816_ERR_BADATRTCK			        0xF7
#define ISO7816_ERR_ICCPROTNOTSUPPORTED	        0xF6
#define ISO7816_ERR_CMD_SLOT_BUSY		        0xE0

#define ISO7816_CLOCKSTATUS_RUNNING		        0
#define ISO7816_CLOCKSTATUS_CLKSTPL		        1
#define ISO7816_CLOCKSTATUS_CLKSTPH		        2
#define ISO7816_CLOCKSTATUS_CLKSTPU		        3

#define ISO7816_POWERSELECT_AUTO		        0
#define ISO7816_POWERSELECT_5V			        1
#define ISO7816_POWERSELECT_3V			        2
#define ISO7816_POWERSELECT_1V8			        3

#define ISO7816_ICCPRESENTANDACTIVE		        0
#define ISO7816_ICCPRESENTANDINACTIVE	        1
#define ISO7816_NOICCPRESENT			        2

#define ISO7816_FREQUENCY_15000000		        7
#define ISO7816_FREQUENCY_7500000		        6
#define ISO7816_FREQUENCY_5000000		        5
#define ISO7816_FREQUENCY_3750000		        4
#define ISO7816_FREQUENCY_2500000		        3
#define ISO7816_FREQUENCY_1875000		        2
#define ISO7816_FREQUENCY_1250000		        1
#define ISO7816_FREQUENCY_1000000		        0
#define ISO7816_FREQUENCY_DEFAULT		        ISO7816_FREQUENCY_5000000

#define ISO7816_CARDTYPE_HIDSIO                 0x00000010
#define ISO7816_CARDTYPE_SAMAV2_DESFIRE8_COLD   0x00000020
#define ISO7816_CARDTYPE_SAMAV2_DESFIRE8_WARM   0x00000021
#define ISO7816_CARDTYPE_SAMAV2_MIFAREPLUS_COLD 0x00000022
#define ISO7816_CARDTYPE_SAMAV3                 0x00000023

typedef	struct __attribute__((__packed__))
{
	byte bmICCStatus:2;
	byte bmRFU:4;
	byte bmCommandStatus:2;
} TISO7816StatusReg; // Size = 1

typedef struct __attribute__((__packed__))
{
	TISO7816StatusReg bStatus;
	byte bError;
	byte bClockStatus;
} TISO7816SlotStatus; // Size = 3

typedef struct __attribute__((__packed__))
{
	byte bmFindexDindex;
	byte bmTCCKS;
	byte bGuardTime;
	byte bWaitingInteger;
	byte bClockStop;
} TProtocolDataT0; // Size = 5

typedef struct __attribute__((__packed__))
{
	byte bmFindexDindex;
	byte bmTCCKS;
	byte bGuardTime;
	byte bmWaitingIntegers;
	byte bClockStop;
	byte bIFSC;
	byte bNadValue;
	byte bWTX;
} TProtocolDataT1; // Size = 8

typedef struct __attribute__((__packed__))
{
	byte Protocol;
	byte Freq;
	uint16_t F;
	uint16_t D;
	union TProtocolData
	{
		TProtocolDataT0 T0;
		TProtocolDataT1 T1;
	} ProtocolData;
} TISO7816CommSettings; // Size = 14

typedef struct __attribute__((__packed__))
{
	byte CLA;
	byte INS;
	byte P1;
	byte P2;
	uint16_t Lc;
	uint16_t Le;
	struct TISO7816_ProtocolHeaderFlags
	{
		byte LePresent:1;
		byte ExtendedAPDU:1;
		byte RFU:6;
	} Flags;
} TISO7816_ProtocolHeader; // Size = 9

typedef struct __attribute__((__packed__))
{
	byte CLA;
	byte INS;
	byte P1;
	byte P2;
	byte P3;
} TISO7816_T0_Header; // Size = 5

typedef struct __attribute__((__packed__))
{
    uint16_t StatusWord;
    uint16_t RemainingTXBytes;
} TISO7816_T0_Status; // Size = 4

#endif

SYSFUNC(API_ISO7816, 0, bool ISO7816_GetSlotStatus(int Channel, TISO7816SlotStatus* SlotStatus))
SYSFUNC(API_ISO7816, 1, bool ISO7816_IccPowerOn(int Channel, byte* ATR, int* ATRByteCnt, int MaxATRByteCnt, byte bPowerSelect, TISO7816StatusReg* bStatus, byte* bError))
SYSFUNC(API_ISO7816, 2, bool ISO7816_IccPowerOff(int Channel, TISO7816SlotStatus* SlotStatus))
SYSFUNC(API_ISO7816, 3, bool ISO7816_SetCommSettings(int Channel, const TISO7816CommSettings* CommSettings))
SYSFUNC(API_ISO7816, 4, bool ISO7816_Transceive(int Channel,const byte* TX, int LenTX, byte* RX, int* LenRX, int MaxRXByteCnt, TISO7816StatusReg* bStatus, byte* bError))
SYSFUNC(API_ISO7816, 5, bool ISO7816_ExchangeAPDU(int Channel, const TISO7816_ProtocolHeader* Header, const byte* TXData, int TXByteCnt, byte* RXData, int* RXByteCnt, int MaxRXByteCnt, uint16_t* StatusWord))
SYSFUNC(API_ISO7816, 6, bool ISO7816_T0_TPDU(int Channel, const TISO7816_T0_Header* Header, const byte* TXData, int TXByteCnt, byte* RXData, int* RXByteCnt, int MaxRXByteCnt, TISO7816_T0_Status* Status))
SYSFUNC(API_ISO7816, 7, bool ISO7816_CheckWellKnownCards(int Channel, int* CardType))

// ******************************************************************
// ****** ICLASS SIO Functions **************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

typedef struct __attribute__((__packed__))
{
    byte ApplicationLimit;
    byte OTP[2];
    byte BlockWriteLock;
    byte ChipConfig;
    byte MemoryConfig;
    byte EAS;
    byte Fuse;
} TICLASS_ConfigBlock; // Size = 8

#define ICLASS_KEYTYPE_CREDIT   0
#define ICLASS_KEYTYPE_DEBIT    1

#endif

SYSFUNC(API_ICLASS, 0, bool ICLASS_GetPACBits(byte* PACBits, int* PACBitCnt, int MaxPACBytes))
SYSFUNC(API_ICLASS, 1, bool ICLASS_SelectPage(int Book, int Page, TICLASS_ConfigBlock* ConfigBlock))
SYSFUNC(API_ICLASS, 2, bool ICLASS_Authenticate(const byte* KeyReferenceOID, int KeyType))
SYSFUNC(API_ICLASS, 3, bool ICLASS_ReadBlock(int Block, byte* BlockData))
SYSFUNC(API_ICLASS, 4, bool ICLASS_WriteBlock(int Block, const byte* BlockData))
SYSFUNC(API_ICLASS, 5, bool ICLASS_SEosSelect(byte* FCI, int* FCIByteCnt, int MaxFCIByteCnt))
SYSFUNC(API_ICLASS, 6, bool ICLASS_SEosAuthenticate(const byte* AdfOID, int AdfOIDByteCnt, const byte* PrivEncKeyRefOID, const byte* PrivMACKeyRefOID, const byte* AuthKeyRefOID))
SYSFUNC(API_ICLASS, 7, bool ICLASS_SEosReadData(byte* Data, int* ByteCnt, int MaxDataByteCnt, const byte* ObjectTag, int ObjectTagByteCnt))
SYSFUNC(API_ICLASS, 8, bool ICLASS_SEosWriteData(const byte* Data, int ByteCnt, const byte* ObjectTag, int ObjectTagByteCnt))

// ******************************************************************
// ****** ISO14443 Transparent Transponder Access Functions *********
// ******************************************************************

SYSFUNC(API_ISO14443,  0, bool ISO14443A_GetATS(byte* ATS, int* ATSByteCnt, int MaxATSByteCnt))
SYSFUNC(API_ISO14443,  1, bool ISO14443B_GetATQB(byte* ATQB, int* ATQBByteCnt, int MaxATQBByteCnt))
SYSFUNC(API_ISO14443,  2, bool ISO14443_4_CheckPresence(void))
SYSFUNC(API_ISO14443,  3, bool ISO14443_4_TDX(byte* TXRX, int TXByteCnt, int* RXByteCnt, int MaxRXByteCnt))
SYSFUNC(API_ISO14443,  4, bool ISO14443A_GetATQA(byte* ATQA))
SYSFUNC(API_ISO14443,  5, bool ISO14443A_GetSAK(byte* SAK))
SYSFUNC(API_ISO14443,  6, bool ISO14443B_GetAnswerToATTRIB(byte* AnswerToATTRIB, int* AnswerToATTRIBByteCnt, int MaxAnswerToATTRIBByteCnt))
SYSFUNC(API_ISO14443,  7, bool ISO14443_3_TDX(byte* TXRX, int TXByteCnt, int* RXByteCnt, int MaxRXByteCnt, int Timeout))
SYSFUNC(API_ISO14443,  8, bool ISO14443A_SearchMultiTag(int* UIDCnt, int* UIDListByteCnt, byte *UIDList, int MaxUIDListByteCnt))
SYSFUNC(API_ISO14443,  9, bool ISO14443A_SelectTag(const byte* UID, int UIDByteCnt))
SYSFUNC(API_ISO14443, 10, bool preISO14443B_GetATR(byte* ATR, int* ATRByteCnt, int MaxATRByteCnt))
SYSFUNC(API_ISO14443, 11, bool ISO14443A_Reselect(void))

// ******************************************************************
// ****** LF Functions **********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define LFSEQCMD_END			0
#define LFSEQCMD_TUNE			1

#define LFSEQCMD_TXBEGIN		2
#define LFSEQCMD_TXOFF			3
#define LFSEQCMD_TXON			4
#define LFSEQCMD_TXKEEPOFF		5
#define LFSEQCMD_TXKEEPON		6
#define LFSEQCMD_TXSAMPLES		7

#define LFSEQCMD_RXBEGIN		8
#define LFSEQCMD_RXSKIP			9
#define LFSEQCMD_RXSAMPLES		10

#endif

SYSFUNC(API_LF,0,bool LFSequencer(const byte *Sequence,int SequenceByteCnt,byte *ReadBytes,int *ReadByteCnt,int MaxReadByteCnt))

// ******************************************************************
// ****** HF Functions **********************************************
// ******************************************************************

// Coming later...

// ******************************************************************
// ****** AT55 Functions ********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__


#endif

SYSFUNC(API_AT55,0,void AT55_Begin(void))
SYSFUNC(API_AT55,1,bool AT55_ReadBlock(int Address,byte *Data))
SYSFUNC(API_AT55,2,bool AT55_ReadBlockProtected(int Address,byte *Data,const byte *Password))
SYSFUNC(API_AT55,3,bool AT55_WriteBlock(int Address,const byte *Data))
SYSFUNC(API_AT55,4,bool AT55_WriteBlockProtected(int Address,const byte *Data,const byte *Password))
SYSFUNC(API_AT55,5,bool AT55_WriteBlockAndLock(int Address,const byte *Data))
SYSFUNC(API_AT55,6,bool AT55_WriteBlockProtectedAndLock(int Address,const byte *Data,const byte *Password))

// ******************************************************************
// ****** EM4150 Functions ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define EM4150_BITRATE_UNKNOWN             	0
#define EM4150_BITRATE_F64                 	1
#define EM4150_BITRATE_F40                	2
#define EM4150_BITRATE_F32                	3

#endif

SYSFUNC(API_EM4150,0,bool EM4150_Login(const byte *Password))
SYSFUNC(API_EM4150,1,bool EM4150_ReadWord(int Address,byte *Word))
SYSFUNC(API_EM4150,2,bool EM4150_WriteWord(int Address,const byte *Word))
SYSFUNC(API_EM4150,3,bool EM4150_WritePassword(const byte *ActualPassword,const byte *NewPassword))
SYSFUNC(API_EM4150,4,int  EM4150_GetTagInfo(void))

// ******************************************************************
// ****** File System ***********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define SID_INTERNALFLASH		1

#define FS_MOUNT_NONE			0
#define FS_MOUNT_READONLY		1
#define FS_MOUNT_READWRITE		2

#define FS_FORMATMAGICVALUE		0x74496F44

#define FS_READ                 0
#define FS_WRITE                1
#define FS_APPEND               2

#define FILE_ENV0         		0
#define FILE_ENV1         		1
#define FILE_ENV2         		2
#define FILE_ENV3         		3
#define FILE_ENV_CNT      		4

#define	FS_POSABS				0
#define	FS_POSREL				1
#define	FS_POSEND				2

typedef struct __attribute__((__packed__))
{
	uint32_t ID;
	uint32_t Length;
} TFileInfo; // Size = 8

typedef struct __attribute__((__packed__))
{
	byte ID;
	uint32_t Size;
	uint32_t Free;
} TStorageInfo; // Size = 13

#endif

SYSFUNC(API_FILESYS, 0,bool FSMount(int StorageID,int Mode))
SYSFUNC(API_FILESYS, 1,bool FSFormat(int StorageID,int MagicValue))
SYSFUNC(API_FILESYS, 2,bool FSOpen(int FileEnv,int StorageID,uint32_t FileID,int Mode))
SYSFUNC(API_FILESYS, 3,bool FSClose(int FileEnv))
SYSFUNC(API_FILESYS, 4,void FSCloseAll(void))
SYSFUNC(API_FILESYS, 5,bool FSSeek(int FileEnv,int Origin,int Pos))
SYSFUNC(API_FILESYS, 6,bool FSTell(int FileEnv,int Origin,int *Pos))
SYSFUNC(API_FILESYS, 7,bool FSReadBytes(int FileEnv,void *Data,int ByteCount,int *BytesRead))
SYSFUNC(API_FILESYS, 8,bool FSWriteBytes(int FileEnv,const void *Data,int ByteCount,int *BytesWritten))
SYSFUNC(API_FILESYS, 9,bool FSFindFirst(int StorageID,TFileInfo *pFileInfo))
SYSFUNC(API_FILESYS,10,bool FSFindNext(TFileInfo *pFileInfo))
SYSFUNC(API_FILESYS,11,bool FSDelete(int StorageID,uint32_t FileID))
SYSFUNC(API_FILESYS,12,bool FSRename(int StorageID,uint32_t OldFileID,uint32_t NewFileID))
SYSFUNC(API_FILESYS,13,bool FSGetStorageInfo(int StorageID,TStorageInfo *pStorageInfo))

// ******************************************************************
// ****** Mifare Plus ***********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_MIFAREPLUS, 0, bool MFP_WritePerso(int BlockNr, const byte* Data))
SYSFUNC(API_MIFAREPLUS, 1, bool MFP_CommitPerso(void))
SYSFUNC(API_MIFAREPLUS, 2, bool MFP_Authenticate(int CryptoEnv, int KeyBNr, const byte* Key))
SYSFUNC(API_MIFAREPLUS, 3, bool MFP_ReadBlock(int CryptoEnv, int Block, byte* Data))
SYSFUNC(API_MIFAREPLUS, 4, bool MFP_WriteBlock(int CryptoEnv, int Block, const byte* Data))
SYSFUNC(API_MIFAREPLUS, 5, bool MFP_ReadValueBlock(int CryptoEnv, int Block, int* Value))
SYSFUNC(API_MIFAREPLUS, 6, bool MFP_WriteValueBlock(int CryptoEnv, int Block, int Value))
SYSFUNC(API_MIFAREPLUS, 7, bool MFP_IncrementValueBlock(int CryptoEnv, int Block, int Value))
SYSFUNC(API_MIFAREPLUS, 8, bool MFP_DecrementValueBlock(int CryptoEnv, int Block, int Value))
SYSFUNC(API_MIFAREPLUS, 9, bool MFP_CopyValueBlock(int CryptoEnv, int SourceBlock, int DestBlock))

// ******************************************************************
// ****** ADC *******************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define ADC_CHANNEL_0			0
#define ADC_CHANNEL_1			1

#endif

SYSFUNC(API_ADC, 0,bool ADCInitChannel(int ADCChannel))
SYSFUNC(API_ADC, 1,int ADCGetConversionValue(int ADCChannel))

// ******************************************************************
// ****** FeliCa ****************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_FELICA, 0, bool FeliCa_TDX(byte* TXRX, int TXByteCnt, int* RXByteCnt, int MaxRXByteCnt, byte MaximumResponseTime, byte NumberOfBlocks))
SYSFUNC(API_FELICA, 1, bool FeliCa_ReadWithoutEncryption(int NumberOfServices, const uint16_t* ServiceCodeList, int NumberOfBlocks, const uint16_t* BlockList, byte* Data))
SYSFUNC(API_FELICA, 2, bool FeliCa_WriteWithoutEncryption(int NumberOfServices, const uint16_t* ServiceCodeList, int NumberOfBlocks, const uint16_t* BlockList, const byte* Data))
SYSFUNC(API_FELICA, 3, bool FeliCa_RequestSystemCode(int* NumberOfSystemCodes, uint16_t* SystemCodeList, int MaxNumberOfSystemCodes))
SYSFUNC(API_FELICA, 4, bool FeliCa_Poll(uint16_t SystemCode, byte* IDm, byte* PMm))
SYSFUNC(API_FELICA, 5, bool FeliCa_RequestService(int NumberOfServices, const uint16_t* ServiceCodeList, uint16_t* KeyVersionList))

// ******************************************************************
// ****** SLE44XX Cards *********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_SLE44XX, 0, bool SLE44XX_GetATR(int Channel, byte* ATR))
SYSFUNC(API_SLE44XX, 1, bool SLE444X_ReadMainMemory(int Channel, int Address, byte* Data, int ByteCnt))
SYSFUNC(API_SLE44XX, 2, bool SLE444X_UpdateMainMemory(int Channel, int Address, byte Value))
SYSFUNC(API_SLE44XX, 3, bool SLE444X_ReadSecurityMemory(int Channel, byte* SecMemData))
SYSFUNC(API_SLE44XX, 4, bool SLE444X_UpdateSecurityMemory(int Channel, int Address, byte SecMemData))
SYSFUNC(API_SLE44XX, 5, bool SLE444X_ReadProtectionMemory(int Channel, byte* ProtMemData))
SYSFUNC(API_SLE44XX, 6, bool SLE444X_WriteProtectionMemory(int Channel, int Address, byte ProtMemData))
SYSFUNC(API_SLE44XX, 7, bool SLE444X_CompareVerificationData(int Channel, int Address, byte VerificationData))
SYSFUNC(API_SLE44XX, 8, bool SLE44X8_ReadMainMemory(int Channel, int Address, byte* Data, int ByteCnt))
SYSFUNC(API_SLE44XX, 9, bool SLE44X8_WriteErrorCounter(int Channel, int Address, byte Byte))
SYSFUNC(API_SLE44XX,10, bool SLE44X8_VerifyPSCByte(int Channel, int Address, byte Byte))
SYSFUNC(API_SLE44XX,11, bool SLE44X8_UpdateMainMemory(int Channel, int Address, byte Value))


// ******************************************************************
// ****** NTAG2XX ***************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_NTAG, 0, bool NTAG_Read(int Page, byte* Data))
SYSFUNC(API_NTAG, 1, bool NTAG_Write(int Page, const byte* Data))
SYSFUNC(API_NTAG, 2, bool NTAG_FastRead(int StartPage, int NumberOfPages, byte* Data))
SYSFUNC(API_NTAG, 3, bool NTAG_ReadCounter(int* CounterValue))
SYSFUNC(API_NTAG, 4, bool NTAG_ReadSig(byte* ECCSig))
SYSFUNC(API_NTAG, 5, bool NTAG_GetVersion(byte* Version))
SYSFUNC(API_NTAG, 6, bool NTAG_PwdAuth(const byte* Password, const byte* PwdAck))
SYSFUNC(API_NTAG, 7, bool NTAG_SectorSelect(int Sector))

// ******************************************************************
// ****** SRX *******************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_SRX, 0, bool SRX_ReadBlock(int Block, byte* Data))
SYSFUNC(API_SRX, 1, bool SRX_WriteBlock(int Block, const byte* Data))

// ******************************************************************
// ****** SAMAVX ****************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

typedef struct __attribute__((__packed__))
{
	byte VersionKeyA;
	byte VersionKeyB;
	byte VersionKeyC;
	uint32_t DF_AID;
	byte DF_KeyNo;
	byte KeyNoCEK;
	byte KeyVCEK;
	byte RefNoKUC;
	uint16_t SET;
} TSAMAVxKeyEntryData; // Size = 13

#endif

SYSFUNC(API_SAMAVX, 0, bool SAMAVx_AuthenticateHost(int CryptoEnv, int KeyNo, const byte* Key, int KeyByteCount, int KeyType))
SYSFUNC(API_SAMAVX, 1, bool SAMAVx_GetKeyEntry(int KeyNo, TSAMAVxKeyEntryData* KeyEntryData))

// ******************************************************************
// ****** EM4102 Functions ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define EM4102_BITRATE_UNKNOWN            	0
#define EM4102_BITRATE_F64                 	1
#define EM4102_BITRATE_F32                	2

#endif

SYSFUNC(API_EM4102, 0,int EM4102_GetTagInfo(void))

// ******************************************************************
// ****** SPI Functions *********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define SPI_MODE_MASTER           	1		// Only master is supported today
#define SPI_MODE_SLAVE            	0

#define SPI_CPOL_LOW              	0
#define SPI_CPOL_HIGH             	1

#define SPI_CPHA_EDGE1            	0
#define SPI_CPHA_EDGE2            	1

#define SPI_CLOCKRATE_15_MHZ   		0
#define SPI_CLOCKRATE_7_5_MHZ      	1
#define SPI_CLOCKRATE_3_75_MHZ     	2
#define SPI_CLOCKRATE_1_88_MHZ     	3
#define SPI_CLOCKRATE_938_KHZ      	4
#define SPI_CLOCKRATE_469_KHZ      	5
#define SPI_CLOCKRATE_234_KHZ      	6
#define SPI_CLOCKRATE_117_KHZ      	7

#define SPI_FIRSTBIT_MSB            0
#define SPI_FIRSTBIT_LSB            1

typedef struct __attribute__((__packed__))
{
    byte Mode;
    byte CPOL;
    byte CPHA;
    byte ClockRate;
    byte BitOrder;
} TSPIParameters;

#endif

SYSFUNC(API_SPI, 0, bool SPIInit(const TSPIParameters *SPIParameters))
SYSFUNC(API_SPI, 1, void SPIDeInit(void))
SYSFUNC(API_SPI, 2, void SPIMasterBeginTransfer(void))
SYSFUNC(API_SPI, 3, void SPIMasterEndTransfer(void))
SYSFUNC(API_SPI, 4, bool SPITransmit(const byte *TXData,int ByteCount))
SYSFUNC(API_SPI, 5, bool SPIReceive(byte *RXData,int ByteCount))
SYSFUNC(API_SPI, 6, bool SPITransceive(const byte *TXData,byte *RXData,int ByteCount))

// ******************************************************************
// ****** BLE Functions *********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

// ------------------------------------------------------------------

#define BLE_MODE_NOTINITIALIZED						-1
#define BLE_MODE_CUSTOM								0
#define BLE_MODE_ADVERTISEMENT						1
#define BLE_MODE_BEACON								2
#define BLE_MODE_ADVERTISEMENT_SM					3
#define BLE_MODE_PACK_ID							4
#define BLE_MODE_DISCOVER							5
#define BLE_MODE_MOBILE_BADGE_2_0					10
#define BLE_MODE_MOBILE_BADGE_2_0_AT2				11
#define BLE_MODE_MOBILE_BADGE_2_0_APPROVAL          245
#define BLE_MODE_MOBILE_BADGE_2_0_APPROVAL_REMOTE   246
#define BLE_MODE_OFF								255

// ------------------------------------------------------------------

#define BLE_CTECHANNEL_MASK                         0x000000FF

#define BLE_CTEMODE_MASK                            0x00000F00
#define BLE_CTEMODE_OFF                             0x00000000
#define BLE_CTEMODE_TX                              0x00000100
#define BLE_CTEMODE_RX                              0x00000200
#define BLE_CTEMODE_CW                              0x00000300

#define BLE_CTEBITRATE_MASK                         0x0000F000
#define BLE_CTEBITRATE_1MBIT                        0x00001000
#define BLE_CTEBITRATE_2MBIT                        0x00002000
#define BLE_CTEBITRATE_125KBIT                      0x00003000
#define BLE_CTEBITRATE_500KBIT                      0x00004000
// ------------------------------------------------------------------

#define BLE_DISC_START_PHY_1M						0
#define BLE_DISC_STOP_PHY_1M						1
#define BLE_DISC_SERVICE							10
#define BLE_DISC_SERVICE_WITH_UUID					11
#define BLE_DISC_CHARAC								20
#define BLE_DISC_CHARAC_WITH_UUID					21
#define BLE_SET_CHARAC_DISABLE_NOTIF_INDIC 			30
#define BLE_SET_CHARAC_NOTIFICATION 				31
#define BLE_SET_CHARAC_INDICATION 					32

#define BLE_CHECK_NAME 								0
#define BLE_CHECK_UUID								3
#define BLE_CHECK_TX_POWER_LEVEL					4
#define BLE_CHECK_SIMPLE_PAIRING					5
#define BLE_CHECK_SERVICE_DATA						6
#define BLE_CHECK_MANUFACTURER_DATA					7

#define BLE_READ_CHARAC								0
#define BLE_READ_SERVICE_WITH_UUID 					1
#define BLE_READ_CHARAC_WITH_OFFSET					2
#define BLE_READ_CHARAC_WITH_VALUE_EVENT 			3

#define BLE_WRITE_CHARAC 							0
#define BLE_WRITE_CHARAC_WITHOUT_RESP 				1

#define BLE_CMD_SET_GATT_MTU						0
#define BLE_CMD_REQUEST_RSSI						1
#define BLE_CONN_STREAM_AVAILABLE					2
#define BLE_CMD_REQUEST_DTM_TEST_END				3
#define BLE_CMD_REQUEST_DISABLE_CONNECTION			4
#define BLE_CMD_ENABLE_MOBILE_BADGE_2_0				5
#define BLE_CMD_REQUEST_RESULTCODE					6
#define BLE_CMD_SYSTEM_SET_TX_POWER					7

#define BLE_SM_BONDING_CONFIRM						0
#define BLE_SM_CONFIGURE							1
#define BLE_SM_DELETE_BONDINGS						2
#define BLE_SM_STORE_BONDING_CONFIGURATION			3
#define BLE_SM_SET_BONDABLE_MODE					4
#define BLE_SM_LIST_ALL_BONDINGS					5
#define BLE_SM_ENTER_PASSKEY						10
#define BLE_SM_SET_PASSKEY							11
#define BLE_SM_PASSKEY_CONFIRM						12
#define BLE_SM_INCREASE_SECURITY					20

#define BLE_SM_SET_OOB_DATA							0
#define BLE_SM_SET_SC_REMOTE_OOB_DATA				1
#define BLE_SET_MOBILE_BADGE_MASTERKEY				10

#define BLE_STREAM_CONN_NONE						0
#define BLE_STREAM_CONN_ADVERTISE					1
#define BLE_STREAM_CONN_DISCOVER					2

#define BLE_STREAM_GATT_SERVER						0
#define BLE_STREAM_GATT_CLIENT						1

#define BLE_STREAM_TRANSFER_BLOCKWISE				0
#define BLE_STREAM_TRANSFER_BYTEWISE				1

// ****** BLE Message Types *****************************************

#define BLE_NO_MESSAGE								         0x000000

#define BLE_MESSAGE_TYPE_MASK                                0xFF0000
#define BLE_RESPONSE_TYPE                                    0x200000
#define BLE_EVENT_TYPE                                       0xA00000

// ------ Message Class 00 ------------------------------------------

#define BLE_EVENT_DFU_BOOT                                   0xA00000
#define BLE_EVENT_DFU_BOOT_FAILURE                           0xA00001

// ------ Message Class 01 ------------------------------------------

#define CMD_SYSTEM_HELLO                                     0x200100
#define CMD_SYSTEM_RESET                                     0x200101
#define CMD_SYSTEM_GET_BT_ADDRESS                            0x200103
#define CMD_SYSTEM_GET_IDENTITY_ADDRESS                      0x200115   // BGM220
#define CMD_SYSTEM_SET_TX_POWER                              0x20010A
#define CMD_SYSTEM_GET_VERSION                               0x20011B   // BGM220

#define BLE_EVENT_SYSTEM_BOOT                                0xA00100

// ------ Message Class 03 ------------------------------------------

#define CMD_LE_GAP_SET_MODE                                  0x200301
#define CMD_LE_GAP_END_PROCEDURE                             0x200303
#define CMD_LE_GAP_SET_ADV_PARAMETERS                        0x200304
#define CMD_LE_GAP_SET_CONN_PARAMETERS                       0x200305
#define CMD_LE_GAP_SET_ADV_DATA                              0x200307
#define CMD_LE_GAP_BT5_SET_ADV_DATA                          0x20030C   // BGM220
#define CMD_LE_GAP_START_ADVERTISING                         0x200314   // BGM220
#define CMD_LE_GAP_SET_DISCOVERY_TIMING                      0x200316
#define CMD_LE_GAP_START_DISCOVERY                           0x200318
#define CMD_LE_GAP_CONNECT                                   0x20031A
#define CMD_LE_GAP_SET_DISCOVERY_EXTENDED_SCAN_RESPONSE      0x20031C

#define BLE_EVENT_LE_GAP_SCAN_RESPONSE                       0xA00300
#define BLE_EVENT_LE_GAP_EXTENDED_SCAN_RESPONSE              0xA00304

// ------ Message Class 04 (BGM220) ---------------------------------

#define CMD_ADVERTISER_CREATE_SET	                         0x200401
#define CMD_ADVERTISER_DELETE_SET	                         0x200402
#define CMD_ADVERTISER_SET_TIMING	                         0x200403
#define CMD_ADVERTISER_SET_CHANNEL_MAP	                     0x200404
#define CMD_ADVERTISER_SET_REPORT_SCAN_REQUEST	             0x200405
#define CMD_ADVERTISER_SET_PHY	                             0x200406
#define CMD_ADVERTISER_SET_CONFIGURATION	                 0x200407
#define CMD_ADVERTISER_CLEAR_CONFIGURATION	                 0x200408
#define CMD_ADVERTISER_START	                             0x200409
#define CMD_ADVERTISER_STOP	                                 0x20040A
#define CMD_ADVERTISER_SET_TX_POWER	                         0x20040B
#define CMD_ADVERTISER_START_PERIODIC_ADVERTISING	         0x20040C
#define CMD_ADVERTISER_STOP_PERIODIC_ADVERTISING	         0x20040D
#define CMD_ADVERTISER_SET_LONG_DATA	                     0x20040E
#define CMD_ADVERTISER_SET_DATA	                             0x20040F
#define CMD_ADVERTISER_SET_RANDOM_ADDRESS	                 0x200410
#define CMD_ADVERTISER_CLEAR_RANDOM_ADDRESS	                 0x200411

// ------ Message Class 05 ------------------------------------------

#define CMD_SCANNER_START                                    0x200503   // BGM220
#define CMD_SCANNER_STOP                                     0x200505   // BGM220

#define BLE_EVENT_SCANNER_SCAN_REPORT                        0xA00501   // BGM220

// ------ Message Class 06 ------------------------------------------

#define CMD_CONNECTION_OPEN                                  0x200604   // BGM220
#define CMD_CONNECTION_CLOSE                                 0x200605   // BGM220

#define BLE_EVENT_CONNECTION_OPENED                          0xA00600
#define BLE_EVENT_CONNECTION_CLOSED                          0xA00601
#define BLE_EVENT_CONNECTION_PARAMETERS                      0xA00602
#define BLE_EVENT_CONNECTION_RSSI                            0xA00603
#define BLE_EVENT_CONNECTION_PHY_STATUS                      0xA00604
#define BLE_EVENT_CONNECTION_GET_REMOTE_TX_POWER_COMPLETED   0xA00605
#define BLE_EVENT_CONNECTION_TX_POWER                        0xA00606
#define BLE_EVENT_CONNECTION_REMOTE_USED_FEATURES            0xA00608

// ------ Message Class 08 ------------------------------------------

#define CMD_LE_CONNECTION_SET_PARAMETERS                     0x200800
#define CMD_LE_CONNECTION_GET_RSSI                           0x200801
#define CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY              0x200802
#define CMD_LE_CONNECTION_CLOSE                              0x200804

#define BLE_EVENT_LE_CONNECTION_OPENED                       0xA00800
#define BLE_EVENT_LE_CONNECTION_CLOSED                       0xA00801
#define BLE_EVENT_LE_CONNECTION_PARAMETERS                   0xA00802
#define BLE_EVENT_LE_CONNECTION_RSSI                         0xA00803
#define BLE_EVENT_LE_CONNECTION_PHY_STATUS                   0xA00804

// ------ Message Class 09 ------------------------------------------

#define CMD_GATT_SET_MAX_MTU                                 0x200900
#define CMD_GATT_DISCOVER_PRIMARY_SERVICES                   0x200901
#define CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID           0x200902
#define CMD_GATT_DISCOVER_CHARACTERISTICS                    0x200903
#define CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID            0x200904
#define CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION             0x200905
#define CMD_GATT_READ_CHARACTERISTIC_VALUE                   0x200907
#define CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID           0x200908
#define CMD_GATT_WRITE_CHARACTERISTIC_VALUE                  0x200909
#define CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE 0x20090A
#define CMD_GATT_SEND_CHARACTERISTIC_CONFIRMATION            0x20090D
#define CMD_GATT_READ_CHARACTERISTIC_VALUE_FROM_OFFSET       0x200912

#define BLE_EVENT_GATT_MTU_EXCHANGED                         0xA00900
#define BLE_EVENT_GATT_SERVICE                               0xA00901
#define BLE_EVENT_GATT_CHARACTERISTIC                        0xA00902
#define BLE_EVENT_GATT_CHARACTERISTIC_VALUE                  0xA00904
#define BLE_EVENT_GATT_PROCEDURE_COMPLETED                   0xA00906

// ------ Message Class 0A ------------------------------------------

#define CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE                 0x200A00
#define CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE                  0x200A01
#define CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE                0x200A02
#define CMD_GATT_SREVER_SEND_CHARACTERISTIC_NOTIFICATION     0x200A05
#define CMD_GATT_SERVER_FIND_ATTRIBUTE                       0x200A06

#define BLE_EVENT_GATT_SERVER_ATTRIBUTE_VALUE                0xA00A00
#define BLE_EVENT_GATT_SERVER_USER_READ_REQUEST              0xA00A01
#define BLE_EVENT_GATT_SERVER_USER_WRITE_REQUEST             0xA00A02
#define BLE_EVENT_GATT_SERVER_CHARACTERISTIC_STATUS          0xA00A03
#define BLE_EVENT_GATT_SERVER_EXECUTE_WRITE_COMPLETED        0xA00A04

// ------ Message Class 0B ------------------------------------------

#define CMD_ENDPOINT_SEND                                    0x200B00
#define CMD_ENDPOINT_SET_STREAMING_DESTINATION               0x200B01
#define CMD_ENDPOINT_CLOSE                                   0x200B02
#define CMD_ENDPOINT_READ_COUNTERS                           0x200B05

#define BLE_EVENT_ENDPOINT_CLOSING                           0xA00B03
#define BLE_EVENT_ENDPOINT_DATA                              0xA00B01
#define BLE_EVENT_ENDPOINT_STATUS                            0xA00B02
#define BLE_EVENT_ENDPOINT_SYNTAX_ERROR                      0xA00B00

// ------ Message Class 0D ------------------------------------------

#define CMD_FLASH_PS_SAVE                                    0x200D02
#define CMD_FLASH_PS_LOAD                                    0x200D03

// ------ Message Class 0E ------------------------------------------

#define CMD_TEST_DTM_TX                                      0x200E00
#define CMD_TEST_DTM_RX                                      0x200E01
#define CMD_TEST_DTM_END                                     0x200E02
#define CMD_TEST_DTM_TX_V4                                   0x200E03   // BGM220
#define CMD_TEST_DTM_TX_CW                                   0x200E04   // BGM220

#define BLE_EVENT_TEST_DTM_COMPLETED                         0xA00E00

// ------ Message Class 0F ------------------------------------------

#define CMD_SM_SET_BONDABLE_MODE                             0x200F00
#define CMD_SM_CONFIGURE                                     0x200F01
#define CMD_SM_STORE_BONDING_CONFIGURATION                   0x200F02
#define CMD_SM_INCREASE_SECURITY                             0x200F04
#define CMD_SM_DELETE_BONDING                                0x200F06
#define CMD_SM_DELETE_BONDINGS                               0x200F07
#define CMD_SM_ENTER_PASSKEY                                 0x200F08
#define CMD_SM_PASSKEY_CONFIRM                               0x200F09
#define CMD_SM_SET_OOB_DATA                                  0x200F0A
#define CMD_SM_LIST_ALL_BONDINGS                             0x200F0B
#define CMD_SM_BONDING_CONFIRM                               0x200F0E
#define CMD_SM_SET_PASSKEY                                   0x200F10
#define CMD_SM_USE_SC_OOB                                    0x200F11
#define CMD_SM_SET_SC_REMOTE_OOB_DATA                        0x200F12

#define BLE_EVENT_SM_PASSKEY_DISPLAY                         0xA00F00
#define BLE_EVENT_SM_PASSKEY_REQUEST                         0xA00F01
#define BLE_EVENT_SM_CONFIRM_PASSKEY                         0xA00F02
#define BLE_EVENT_SM_BONDED                                  0xA00F03
#define BLE_EVENT_SM_BONDING_FAILED                          0xA00F04
#define BLE_EVENT_SM_LIST_BONDING_ENTRY                      0xA00F05
#define BLE_EVENT_SM_LIST_ALL_BONDINGS_COMPLETE              0xA00F06
#define BLE_EVENT_SM_CONFIRM_BONDING                         0xA00F09

// ------ Message Class FF ------------------------------------------

#define CMD_USER_MESSAGE_TO_TARGET                           0x20FF00

// ------------------------------------------------------------------

// BLE configuration structure
typedef struct __attribute__((__packed__))
{
	uint32_t ConnectTimeout;
	byte Power;
	byte BondableMode;
	uint16_t AdvInterval;       // multiples of 0.625ms
	byte ChannelMap;
	byte DiscoverMode;
	byte ConnectMode;
	byte SecurityFlags;
	byte IOCapabilities;
	uint32_t Passkey;
} TBLEConfig;

// BLE UUID structure
typedef struct __attribute__((__packed__))
{
	int UUIDLength;
	byte UUID[16];
} TBLEUUID;

#endif

SYSFUNC(API_BLE, 0, bool BLEPresetConfig(const TBLEConfig *BLEConfig))
SYSFUNC(API_BLE, 1, bool BLEPresetUserData(byte ScanResp, const byte *UserData, int UserDataLength))
SYSFUNC(API_BLE, 2, bool BLEInit(int NewMode))
SYSFUNC(API_BLE, 3, int  BLECheckEvent(void))
SYSFUNC(API_BLE, 4, bool BLEGetAddress(byte *DeviceAddress, byte *RemoteAddress, byte *Type))
SYSFUNC(API_BLE, 5, bool BLEGetVersion(byte *HWVersion, byte *BootString))
SYSFUNC(API_BLE, 6, bool BLEGetEnvironment(byte *DeviceRole, byte *SecurityMode, byte *Rssi))
SYSFUNC(API_BLE, 7, bool BLEGetGattServerAttributeValue(int AttrHandle, byte *Data, int *Len, int MaxLen))
SYSFUNC(API_BLE, 8, bool BLESetGattServerAttributeValue(int AttrHandle, int Offset, const byte *Data, int Len))
SYSFUNC(API_BLE, 9, bool BLERequestRssi(void))
SYSFUNC(API_BLE, 10,bool BLERequestEndpointClose(void))
SYSFUNC(API_BLE, 11,bool BLEGetGattServerCharacteristicStatus(int *AttrHandle, int *AttrStatusFlag, int *AttrConfigFlag))
SYSFUNC(API_BLE, 12,bool BLEFindGattServerAttribute(const byte *UUID, int UUIDLength, int *AttrHandle))
SYSFUNC(API_BLE, 13,bool BLEDiscover(int DiscoverMode, unsigned long GattHandle, const TBLEUUID *BLEUUID))
SYSFUNC(API_BLE, 14,bool BLECheckDiscoveredString(int CheckMode, const byte *CompareString, int Length))
SYSFUNC(API_BLE, 15,bool BLEConnectToDevice(const byte *Address, byte AddressType))
SYSFUNC(API_BLE, 16,bool BLEDisconnectFromDevice(void))
SYSFUNC(API_BLE, 17,bool BLEGattGetAttribute(TBLEUUID *BLEUUID, unsigned long *GattHandle))
SYSFUNC(API_BLE, 18,bool BLEGattGetValue(int ReadMode, unsigned long GattHandle, const TBLEUUID *BLEUUID, byte *AttrOpcode, byte *Data, int *Len, int MaxLen))
SYSFUNC(API_BLE, 19,bool BLEGattSetValue(int WriteMode, unsigned long GattHandle, int Offset, const byte *Data, int DataLength))
SYSFUNC(API_BLE, 20,int  BLECommand(int CommandCode, int Parameter))
SYSFUNC(API_BLE, 21,bool BLESecurity(int SMMode, unsigned long Flag1, unsigned long Flag2))
SYSFUNC(API_BLE, 22,bool BLESecuritySetOob(int SMOOBMode, const byte *OobData, int DataLength))
SYSFUNC(API_BLE, 23,bool BLESecurityUseScOob(int Enable, byte *OobData, int *Length, int MaxLength))
SYSFUNC(API_BLE, 24,bool BLESetStreamingUUID(const byte *ServiceUUID, int ServiceUUIDLength, const byte *CharacUUID, int CharacUUIDLength))
SYSFUNC(API_BLE, 25,bool BLESetStreamingMode(int ConnMode, int GattMode, int TransferMode))
SYSFUNC(API_BLE, 26,bool BLEGetDiscoveredData(byte *Data, int *Len, int MaxLen))
SYSFUNC(API_BLE, 27,bool BLEInitCTE(int ModeBits))

// ******************************************************************
// ****** EMVCo Functions *******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define ERRORDOMAIN_PN5180						0x10000000

#define ERR_PN5180_TIMER1TIMEOUT				(ERRORDOMAIN_PN5180 | 0x00000001)
#define ERR_PN5180_LOCKUPPROTECT				(ERRORDOMAIN_PN5180 | 0x00000002)
#define ERR_PN5180_DATAINTEGRITY				(ERRORDOMAIN_PN5180 | 0x00000003)
#define ERR_PN5180_RXDATABUFFTOOSMALL			(ERRORDOMAIN_PN5180 | 0x00000004)
#define ERR_PN5180_PROTOCOLERROR				(ERRORDOMAIN_PN5180 | 0x00000005)
#define ERR_PN5180_COLLISION					(ERRORDOMAIN_PN5180 | 0x00000006)
#define ERR_PN5180_UNEXPECTEDDATA				(ERRORDOMAIN_PN5180 | 0x00000007)

#endif

SYSFUNC(API_EMVCO, 0, void EMVCo_SetRFOn(void))
SYSFUNC(API_EMVCO, 1, bool EMVCo_WUPA(byte* ATQA))
SYSFUNC(API_EMVCO, 2, bool EMVCo_WUPB(byte* ATQB, int* ATQBByteCnt, int MaxATQBByteCnt))
SYSFUNC(API_EMVCO, 3, bool EMVCo_ISO14443A_DoCascadeLevel(byte CascadeLevel, byte* UIDCLn))
SYSFUNC(API_EMVCO, 4, bool EMVCo_ISO14443A_Select(byte CascadeLevel, const byte* UIDCLn, byte* SAK))
SYSFUNC(API_EMVCO, 5, bool EMVCo_ISO14443_3_TDX(byte* TXRX, int TXByteCnt, int* RXByteCnt, int MaxRXByteCnt, int Timeout, bool EMDHandling))


// ******************************************************************
// ****** PN5180 Functions ******************************************
// ******************************************************************

SYSFUNC(API_PN5180, 0, bool PN5180_BootStart(void))
SYSFUNC(API_PN5180, 1, bool PN5180_BootStop(void))
SYSFUNC(API_PN5180, 2, bool PN5180_BootGetVersion(byte *Version))
SYSFUNC(API_PN5180, 3, bool PN5180_BootSecureWrite(const byte *ChunkData,int ChunkLength))

// ******************************************************************
// ****** I2C Memory Cards ******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_I2CCARD, 0, bool I2CCard_Read(int Channel, int Addr, byte* Data, int ByteCnt))
SYSFUNC(API_I2CCARD, 1, bool I2CCard_Write(int Channel, int Addr, const byte* Data, int ByteCnt))

// ******************************************************************
// ****** Transponder Type Topaz ************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_TOPAZ, 0, bool Topaz_RID(byte* HR0, byte* HR1, byte* UID))
SYSFUNC(API_TOPAZ, 1, bool Topaz_ReadByte(const byte* UID, byte ADD, byte* Byte))
SYSFUNC(API_TOPAZ, 2, bool Topaz_ReadAllBlocks(const byte* UID, byte* HR0, byte* HR1, byte* Data))
SYSFUNC(API_TOPAZ, 3, bool Topaz_WriteByteWithErase(const byte* UID, byte ADD, byte Byte))
SYSFUNC(API_TOPAZ, 4, bool Topaz_WriteByteNoErase(const byte* UID, byte ADD, byte Byte))

// ******************************************************************
// ****** CTS256 / CTS512 *******************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#endif

SYSFUNC(API_CTS, 0, bool CTS_ReadBlock(int Block, byte* Data))
SYSFUNC(API_CTS, 1, bool CTS_WriteBlock(int Block, const byte* Data))
SYSFUNC(API_CTS, 2, bool CTS_UpdateBlock(int Block, const byte* Data))

// ******************************************************************
// ****** Intelligent Periphery Extender (IPE) **********************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define IPE_VERSION_MAJOR       1
#define IPE_VERSION_MINOR       0
#define IPE_VERSION_BUILD       0

#define IPE_TXD                 GPIO0
#define IPE_RXD                 GPIO1
#define IPE_RESET               GPIO2
#define IPE_BOOT0               GPIO3

#define IPE_HEADER_START        0xAA
#define IPE_HEADER_SIZE         3

#define IPE_COMMAND_VERSION         0x02
#define IPE_COMMAND_LED             0x03
#define IPE_INPUT_CTRL              0x04
#define IPE_COMMAND_ONEWIRE         0x05
#define IPE_COMMAND_I2CTRANSPARENT  0x06
#define IPE_COMMAND_DIGITALINPUTS   0x07
#define IPE_COMMAND_DIGITALOUTPUTS  0x08
#define IPE_COMMAND_HARDWAREDETECT  0x09
#define IPE_COMMAND_HARDWARESET     0x0A
#define IPE_COMMAND_GETHARDWARETYPE 0x0B
#define IPE_INVALID                 0x55

#define IPE_INDEX_HEADER        0
#define IPE_INDEX_LEN           1
#define IPE_INDEX_COMMAND       2

// IPE LED definitions - all except WHITE match OSDP standard
#define IPE_LEDCOLOR_BLACK      0x00
#define IPE_LEDCOLOR_RED        0x01
#define IPE_LEDCOLOR_GREEN      0x02
#define IPE_LEDCOLOR_AMBER      0x03
#define IPE_LEDCOLOR_BLUE       0x04
#define IPE_LEDCOLOR_WHITE      0x07

// LED temporary and permanent controls - note that these match OSDP standard
#define IPE_LED_TEMP_NOP        0x00    // No change from currently active
#define IPE_LED_TEMP_CANCEL     0x01    // Cancel immeditely and move to permanent settings
#define IPE_LED_TEMP_SET        0x02    // Take temporary settings provided in this frame

#define IPE_LED_PERM_NOP        0x00    // No change to previously saved (ignore ones in this frame)
#define IPE_LED_PERM_SET        0x01    // Update permanent settings to ones in this frame.

// IPE LED Page
typedef struct __attribute__((__packed__))
{
    uint8_t     Reader;
    uint8_t     ID;
    uint8_t     TempControl;
    uint8_t     TempOnTime;
    uint8_t     TempOffTime;
    uint8_t     TempOnColor;
    uint8_t     TempOffColor;
    uint8_t     TempTimerLSB;
    uint8_t     TempTimerMSB;
    uint8_t     PermControl;
    uint8_t     PermOnTime;
    uint8_t     PermOffTime;
    uint8_t     PermOnColor;
    uint8_t     PermOffColor;
    uint8_t     TempDelay;
    uint8_t     PermDelay;
} TIPELEDCtrl;

#endif

// ******************************************************************
// ****** OSDP (leveraging TCSP) ************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define OSDP_READER_NUMBER  0x00    // Theoretically, there could be multiple readers on one device (with same address). But in our case, there is only 1.


// OSDP Command Codes
#define OSDP_POLL       	0x60
#define OSDP_ID         	0x61
#define OSDP_CAP        	0x62
#define OSDP_DIAG       	0x63
#define OSDP_LSTAT      	0x64
#define OSDP_ISTAT      	0x65
#define OSDP_OSTAT      	0x66
#define OSDP_RSTAT      	0x67
#define OSDP_OUT        	0x68
#define OSDP_LED        	0x69
#define OSDP_BUZ        	0x6A
#define OSDP_TEXT       	0x6B
#define OSDP_TDSET      	0x6D
#define OSDP_COMSET     	0x6E
#define OSDP_DATA       	0x6F
#define OSDP_PROMPT     	0x71
#define OSDP_BIOREAD    	0x73
#define OSDP_KEYSET     	0x75
#define OSDP_CHLNG      	0x76
#define OSDP_SCRYPT     	0x77
#define OSDP_MAXREPLY   	0x7B
#define OSDP_ACURXSIZE   	0x7B	// same numbering is on purpose!
#define OSDP_MFG        	0x80
#define OSDP_FILETRANSFER 	0x7C
#define OSDP_XWR			0xA1
#define OSDP_ABORT			0xA2
#define OSDP_PIVDATA		0xA3
#define OSDP_GENAUTH		0xA4
#define OSDP_CRAUTH			0xA5
#define OSDP_KEEPACTIVE		0xA5

// OSDP Response Codes
#define OSDP_NONE       0x00
#define OSDP_ACK        0x40
#define OSDP_NAK        0x41
#define OSDP_PDID       0x45
#define OSDP_PDCAP      0x46
#define OSDP_LSTATR     0x48
#define OSDP_ISTATR     0x49
#define OSDP_OSTATR     0x4A
#define OSDP_RSTATR     0x4B
#define OSDP_RAW        0x50
#define OSDP_FMT        0x51
#define OSDP_KEYPAD     0x53
#define OSDP_COM        0x54
#define OSDP_CCRYPT     0x76
#define OSDP_RMAC_I     0x78
#define OSDP_BUSY       0x79
#define OSDP_MFGSTATR   0x83
#define OSDP_MFGREP     0x90
#define OSDP_FTSTAT		0x7A

// OSDP NAK error codes
#define OSDP_NAK_CHECK_CHAR_ERROR               0x01
#define OSDP_NAK_COMMAND_LENGTH_ERR             0x02
#define OSDP_NAK_UNKNOWN_COMMAND_CODE           0x03
#define OSDP_NAK_UNEXPECTED_SEQ_NUM             0x04
#define OSDP_NAK_SECURE_BLOCK_NOT_SUPPORTED     0x05
#define OSDP_NAK_SECURE_CONDITIONS_NOT_MET      0x06
#define OSDP_NAK_CANT_PROCESS_COMMAND_RECORD    0x09

// OSDP NAK codes related to individual records
#define OSDP_NAK_RECORD_OK                      0x00
#define OSDP_NAK_RECORD_BAD                     0x01
#define OSDP_NAK_GENERIC_ERR                    0xFF

// OSDP Capabilites Report Variables (listed in Appendix B of standard v2.1.7)
enum OSDPCap_ContactStatus      { InOnOff=1,        InOnOffCfg=2,       InOnOffCfgMonitor=3,            InOnOffCfgMonitorEOL=4      };
enum OSDPCap_OutputControl      { OutOnOff=1,       OutOnOffCfg=2,      OutTimed=3,                     OutOnOffCfgTimed=4          };
enum OSDPCap_CardDataFormat     { BitsUpTo1024=1,   CharsUpTo256=2,     BitsUpTo1024orCharsUpTo256=3                                };
enum OSDPCap_LEDControl         { LEDOnOffOnly=1,   LEDOnOffTimed=2,    LEDOnOffTimedBiColor=3,         LEDOnOffTimedTriColor=4     };
enum OSDPCap_AudibleOutput      { BuzzOnOffOnly=1,  BuzzOnOffTimed=2                                                                };
enum OSDPCap_TextOutput         { TextNoSupport=0,  Text1Row16Char=1,   Text2Row16Char=2,               Text4Row16Char=3            };
enum OSDPCap_CheckCharSupport   { CheckSumOnly=0,   CheckSumAndCRC=1                                                                };
enum OSDPCap_ComSecurity        { SecurityNone=0,   SecurityAES128=1                                                                };
enum OSDPCap_SecurityAlgorithms { NoAlogrithms=0x00,   AlgorithmAES128=0x01                                                         };
enum OSDPCap_SmartCardSupport   { TransparentReaderNotSupported=0,      TransparentReaderSupported=1                                };

/// @brief Siemens SiPass Command Msg IDs
/// @note  See: "SiPass Support of OSDP Readers_1.0.1"
typedef enum _SiPass_MsgIds{
  SiPass_MsgId_InitFWDownload,  // Inits firmware download
  SiPass_MsgId_FWDataPacket,    // firmware data packet
  SiPass_MsgId_FWEndReboot,     // firmware download finished - reboot reader
  SiPass_MsgId_FWEndNoReboot,   // firmware download finished - don't reboot reader; wait for reboot command
  SiPass_MsgId_Reboot,          // reboot reader
  SiPass_MsgId_CFGInit = 0x10,         // Inits configuration file download
  SiPass_MsgId_CFGDataPacket = 0x11,   // Configuration data packet
  SiPass_MsgId_CFGEnd = 0x12,           // Configuration download finished
  SiPass_MsgId_Discover = 0x20,         // if the serial number is the same or higher, the reader will answer with ID report
  SiPass_MsgId_SetAddress = 0x21        // if the serial number is the same or higher, the reader will accept new address
} SiPass_MsgIds;

/// @brief Siemens SiPass Replies
/// @note  See: "SiPass Support of OSDP Readers_1.0.1"
typedef enum _SiPass_Replies
{
  SiPass_Reply_FWInitSuccess = 0x80,      // Initialisation for firmware download - Completed Ok
  SiPass_Reply_FWInitFailed = 0x81,       // Initialisation for firmware download - Failed
  SiPass_Reply_FWDataPacketOK = 0x82,     // Firmware data packet - Stored Ok
  SiPass_Reply_FWDataPacketError = 0x83,  // Firmware data packet - Storage Error
  SiPass_Reply_FWFinishOK = 0x84,         // End of firmware download - Ok
  SiPass_Reply_FWFinishError = 0x85,      // End of firmware download - Failed (Storage, Processing or Chksum error)
  SiPass_Reply_CFGInitSuccess = 0x90,     // Initialisation for config file download - Completed Ok
  SiPass_Reply_CFGInitFailed = 0x91,      // Initialisation for config file download - Failed
  SiPass_Reply_CFGDataPacketOK = 0x92,    // Config file data packet - Stored Ok
  SiPass_Reply_CFGDataPacketError = 0x93, // Config file data packet - Storage Error
  SiPass_Reply_CFGFinishedOK = 0x94,      // End of config file download - Ok
  SiPass_Reply_CFGFinishedError = 0x95,   // End of config file download - Failed (Storage, Processing or Chksum error)
} SiPass_Replies;

// Parsing Buzzer Command
typedef struct __attribute__((__packed__))
{
    uint8_t     Reader;
    uint8_t     ToneCode;
    uint8_t     OnTime;
    uint8_t     OffTime;
    uint8_t     Count;
} TOSDPRdrBuzzCtrl;

#define OSDP_BUZZ_TONE_NONE         0x00
#define OSDP_BUZZ_TONE_OFF          0x01
#define OSDP_BUZZ_TONE_DEFAULT      0x02

#define OSDP_BUZZ_COUNT_NON_STOP    0x00

/**
 * @brief Temporary Control Code Values
 *
 */
typedef enum _TOSDPTempControlCode
{
    OSDP_LED_Temp_Control_Code_NOP,           /* Do not alter temporary settings */
    OSDP_LED_Temp_Control_Code_CancelCurrent, /* Cancel current temp action and jump directly to this commands permanent state */
    OSDP_LED_Temp_Control_Code_Immediat       /* Set the temporary state as given */
} TOSDPTempControlCode;

/**
 * @brief Permanent Control Code Values
 *
 */
typedef enum
{
    OSDP_LED_Perm_Control_Code_NOP, /* do not alter this LEDs permanent settings */
    OSDP_LED_Perm_Control_Code_Set  /* Set the permanent state as given */
} TOSDPPermControlCode;

/**
 * @brief This represents a complete singe OSDP LED Command
 *
 * @return typedef struct
 */
typedef struct __attribute__((__packed__)) _TOSDPRdrLEDCtrl
{
    uint8_t Reader;                   /* Reader ID -- Typical 0 */
    uint8_t ID;                       /* (Mapped) LED ID -- not the physical LED channel */
    TOSDPTempControlCode TempControl; /* Control Code for temporary phase of type @TOSDPTempControlCode */
    uint8_t TempOnTime;               /* Time where the led displays OnColor while in temporary phase */
    uint8_t TempOffTime;              /* Time where the led displays OffColor while in temporary phase */
    uint8_t TempOnColor;              /* Color which is displayed while in OnTime */
    uint8_t TempOffColor;             /* Color which is displayed while in OffTime */
    uint8_t TempTimerLSB;             /* Low Byte of time the led will stay in temporary state */
    uint8_t TempTimerMSB;             /* High Byte of time the led will stay in temporary state */
    TOSDPPermControlCode PermControl; /* Control Code for permanent state of type @TOSDPPermControlCode */
    uint8_t PermOnTime;               /* Time where the led displays OnColor while in permanent phase */
    uint8_t PermOffTime;              /* Time where the led displays OffColor while in permanent phase */
    uint8_t PermOnColor;              /* Color which is displayed while in OnTime */
    uint8_t PermOffColor;             /* Color which is displayed while in OffTime */
} TOSDPRdrLEDCtrl;

#define OSDP_LED_TEMP_NOP       0x00
#define OSDP_LED_TEMP_CANCEL    0x01
#define OSDP_LED_TEMP_SET       0x02

#define OSDP_LED_PERM_NOP       0x00
#define OSDP_LED_PERM_SET       0x01



#endif

// ******************************************************************
// ****** Mirror Image **********************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

#define MS_IDLE                    0
#define MS_INIT                    1
#define MS_WRITEFIRMWARE           2
#define MS_WRITEAPP                3
#define MS_DONE                    10
#define MS_ERROR                   11

#define MIRROR_TYPE_FIRMWARE       2
#define MIRROR_TYPE_APP            3

#endif

SYSFUNC(API_MIRROR, 0,bool Mirror_GetState(void))
SYSFUNC(API_MIRROR, 1,bool Mirror_Start(void))
SYSFUNC(API_MIRROR, 2,bool Mirror_Init(int ObjectType,int ObjectSize,const byte *pHeader,int HeaderSize))
SYSFUNC(API_MIRROR, 3,bool Mirror_Write(const byte *pObjectChunk,int ObjectChunkSize))
SYSFUNC(API_MIRROR, 4,bool Mirror_Finish(void))

// ******************************************************************
// ****** Apple Pay *************************************************
// ******************************************************************

SYSFUNC(API_APPLEPAY, 0, bool ApplePay_SelectOSE(void))
SYSFUNC(API_APPLEPAY, 1, bool ApplePay_GetVASDataRaw(byte* VASData, int* VASDataByteCnt, int MaxVASDataByteCnt))
SYSFUNC(API_APPLEPAY, 2, bool ApplePay_GetVASDataDecrypted(byte* VASData, int* VASDataByteCnt, int MaxVASDataByteCnt))

// ******************************************************************
// ****** Transact **************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

typedef struct __attribute__((__packed__))
{
    byte TCI[3];
	byte GPayMobileCredentialMasterKey[16];
	byte AppleMobileCredentialMasterKey[16];
	byte AppleMobileCredentialASCIIIDMasterKey[16];
	byte PhysicalDESFireKey[16];
	byte MifareClassicKey[6];
} TTransactKeys;

#endif

SYSFUNC(API_TRANSACT, 0, bool Transact_GenerateKeys(const byte* KeyPackage, int KeyPackageLength, TTransactKeys* Keys))

// ******************************************************************
// ****** EM4305 ****************************************************
// ******************************************************************

SYSFUNC(API_EM4305, 0, void EM4305_Begin(void))
SYSFUNC(API_EM4305, 1, bool EM4305_Read(int Address,byte *Data))
SYSFUNC(API_EM4305, 2, bool EM4305_Write(int Address,const byte *Data))
SYSFUNC(API_EM4305, 3, bool EM4305_Login(const byte *Password))
SYSFUNC(API_EM4305, 4, bool EM4305_Protect(const byte *Data))
SYSFUNC(API_EM4305, 5, bool EM4305_Disable(void))

// ******************************************************************
// ****** KleverKey *************************************************
// ******************************************************************

SYSFUNC(API_KLEVERKEY, 0, bool KleverKey_GetAppKey(byte *AppKey))

// ******************************************************************
// ****** GPay ******************************************************
// ******************************************************************

#ifndef __TWN4_SYS_C__

typedef struct __attribute__((__packed__))
{
    byte MobileDeviceNonce[32];
    byte TerminalNonce[32];
    byte MobileDeviceEphPubKey[33];
    byte TerminalEphPubKey[33];
    byte TerminalEphPrivKey[32];
    byte SignedData[80];
    int SignedDataByteCnt;
} TGPayEncryption;

#endif

SYSFUNC(API_GPAY, 0, bool GPay_SelectOSE(void))
SYSFUNC(API_GPAY, 1, bool GPay_SelectSmartTap2(byte* MobileDeviceNonce))
SYSFUNC(API_GPAY, 2, bool GPay_NegotiateSecureSessions(TGPayEncryption* GPayEnv))
SYSFUNC(API_GPAY, 3, bool GPay_GetData(const TGPayEncryption* GPayEnv, byte* Data, int* DataByteCnt, int MaxDataByteCnt))

// ******************************************************************
// ****** Giro-e ****************************************************
// ******************************************************************

SYSFUNC(API_GIROE, 0, bool GiroE_CheckGiroCard(void))
SYSFUNC(API_GIROE, 1, bool GiroE_ReadCardDataGiroCard(byte* Out, int* OutByteCnt, int MaxOutByteCnt))

// ******************************************************************
// ****** Leaf ******************************************************
// ******************************************************************

#define LEAF_STATUS_OK              0
#define LEAF_STATUS_KEYUPDATED      1
#define LEAF_STATUS_GENERALERROR    -1

SYSFUNC(API_LEAF, 0, int  Leaf_Init(int CryptoEnv))
SYSFUNC(API_LEAF, 1, bool Leaf_SelectApplication(int CryptoEnv, int AID))
SYSFUNC(API_LEAF, 2, bool Leaf_AuthenticateTag(int CryptoEnv, int KeyNoSAM, int KeyVerSAM, int KeyNoTag, const byte* DivInp, int DivInpByteCnt))
SYSFUNC(API_LEAF, 3, bool Leaf_ReadCredentialData(int CryptoEnv, int FileNo, byte* Data, int Offset, int Length))
SYSFUNC(API_LEAF, 4, bool Leaf_WriteCredentialData(int CryptoEnv, int FileNo, byte* Data, int Offset, int Length))

// ******************************************************************
// ****** ECC *******************************************************
// ******************************************************************

#define ECC_SECP160R1   0
#define ECC_SECP192R1   1
#define ECC_SECP224R1   2
#define ECC_SECP256R1   3
#define ECC_SECP256K1   4

SYSFUNC(API_ECC, 0, int  ECC_GetCurvePrivateKeySize(int Curve))
SYSFUNC(API_ECC, 1, int  ECC_GetCurvePublicKeySize(int Curve))
SYSFUNC(API_ECC, 2, bool ECC_IsValidPublicKey(const byte* PublicKey, int PublicKeyByteCnt, int Curve))
SYSFUNC(API_ECC, 3, bool ECC_ComputePublicKey(const byte* PrivateKey, int PrivateKeyByteCnt, byte* PublicKey, int* PublicKeyByteCnt, int MaxPublicKeyByteCnt, int Curve))
SYSFUNC(API_ECC, 4, bool ECC_Decompress(const byte* Compressed, int CompressedByteCnt, byte* PublicKey, int* PublicKeyByteCnt, int MaxPublicKeyByteCnt, int Curve))
SYSFUNC(API_ECC, 5, bool ECC_Compress(const byte* PublicKey, int PublicKeyByteCnt, byte* Compressed, int* CompressedByteCnt, int MaxCompressedByteCnt, int Curve))
SYSFUNC(API_ECC, 6, bool ECC_SharedSecret(const byte* PublicKey, int PublicKeyByteCnt, const byte* PrivateKey, int PrivateKeyByteCnt, byte* Secret, int* SecretByteCnt, int MaxSecretByteCnt, int Curve))
SYSFUNC(API_ECC, 7, bool ECC_Sign(const byte* PrivateKey, int PrivateKeyByteCnt, const byte* MessageHash, int MessageHashByteCnt, byte* Signature, int* SignatureByteCnt, int MaxSignatureByteCnt, int Curve))
SYSFUNC(API_ECC, 8, bool ECC_Verify(const byte* PublicKey, int PublicKeyByteCnt, const byte* MessageHash, int MessageHashByteCnt, const byte* Signature, int SignatureByteCnt, int Curve))

// ******************************************************************
// ******* End of System Functions **********************************
// ******************************************************************

#pragma GCC diagnostic pop

#endif
