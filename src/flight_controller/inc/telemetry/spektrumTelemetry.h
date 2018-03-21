#pragma once

#include "spektrumTelemetrySensors.h"

#pragma pack(1)

#define SPEKTRUM_SRXL_ID  0xA5
#define SRXL_TELEM_ID  0x80
#define SRXL_TELEM_LENGTH 21
    
#define SRXL_BIND_ID  0x41
#define SRXL_BIND_LENGTH 19
#define SRXL_BIND_ENTER 0xEB
#define SRXL_BIND_REQUEST_STATUS 0xB5
#define SRXL_BIND_BOUND_DATA_REPORT 0xDB
#define SRXL_BIND_SET_BIND 0x5B

#define SRXL_POLY 0x1021        //CRC polynomial

#define MAX_SENSORS 8

typedef enum
{
	TELEM_START      = 0,
	TELEM_FLIGHTLOG  = TELEM_START,
	TELEM_INTERNAL   = 1,
	TELEM_XBUS       = 2,
	NUM_TELEM_STATES = 3
} TELEMETRY_STATE;


#define INTERNAL_ID   0x7E
typedef struct
{
	uint8_t	  id;					// Source device = 0x7E
	uint8_t	  rfu;
	uint16_t	rpm;				// Current rpm, in 10's
	uint16_t	packVoltage;
	uint16_t	temperature;		// Temperature in F
	uint8_t	  spare[8];
} TLM_INTERNAL;

#define FLIGHTLOG_ID  0x7F
typedef struct
{
	uint8_t	  id;					// Source device = 0x7F
	uint8_t	  rfu;
	uint16_t	A;					// FFFF is not reporting
	uint16_t	B;
	uint16_t	L;
	uint16_t	R;
	uint16_t	F;
	uint16_t	H;
	uint16_t  rxVoltage;			// Volts, .1V
} TLM_FLIGHTLOG;

typedef struct
{
	uint8_t   data[16];
} TLM_XBUS;
#define TLM_XBUS_LEN			(sizeof(TLM_XBUS))

typedef struct
{
	uint8_t   data[16];
} TLM_SERIAL;
#define TLM_SERIAL_LEN			(sizeof(TLM_SERIAL))

typedef union
{
	TLM_FLIGHTLOG   flightLog;
	TLM_INTERNAL    internalSensors;
	TLM_XBUS        xBUS;
	TLM_SERIAL      serial;
	uint8_t			raw[16];
} TELEMETRY_STR;

typedef union
{
	struct PACKET
	{
		uint8_t SRXL_ID;
		uint8_t identifier;
		uint8_t length;
		TELEMETRY_STR data;
		uint16_t crc;
	}packet;
	uint8_t raw[21];

} STR_SRXL_TELEM;
#define SRXLTELEM_PACKET_LEN  (sizeof(STR_SRXL_TELEM))

typedef union
{
	struct
	{
		uint8_t   info;
		uint8_t   holds;
		uint16_t  frames;
		uint16_t  fades[4];
		uint8_t   rfu[4];
	};
	uint8_t asBytes[16];
} STR_FLIGHT_LOG;
STR_FLIGHT_LOG flightLog;
#define FLIGHT_LOG_LEN  (sizeof(STR_FLIGHT_LOG))

typedef union
{
	struct
	{
		uint8_t		request;
		uint64_t	guid;
		uint8_t		type;
		uint32_t	rfID;
	};
	uint8_t	raw[14];
	
} STR_BIND;
#define SRXLBIND_PAYLOAD_LEN  (sizeof(STR_BIND))

typedef struct
{
	uint8_t		srxlID;		//0xA5 SPM SRXL
	uint8_t		subID;		//0x41 for BIND packet type
	uint8_t		length;		//19 for BIND packet type
	STR_BIND	data;		//Bind packet payload data
	uint16_t	crc;
} STR_SRXL_BIND;
#define SRXLBIND_PACKET_LEN  (sizeof(STR_SRXL_BIND))

typedef enum
{
	REMOTE_A = 0,
	REMOTE_B = 1,
	REMOTE_L = 2,
	REMOTE_R = 3,
} RF_REMOTES;

struct
{
	uint8_t sensorPosition;
	uint8_t textLine;
	uint8_t sensorCount;
	uint8_t sensorAddress[MAX_SENSORS];
} xbus;


//VTX enums and struct
typedef enum
{
	SPEK_VTX_BAND_FATSHARK = 0,
	SPEK_VTX_BAND_RACEBAND = 1,
	SPEK_VTX_BAND_E        = 2,
	SPEK_VTX_BAND_B        = 3,
	SPEK_VTX_BAND_A        = 4,
} VTX_BAND;

typedef enum
{
	SPEK_VTX_POWER_25MW  = 0,
	SPEK_VTX_POWER_250MW = 1,
	SPEK_VTX_POWER_500MW = 2,
} VTX_POWER;

typedef enum
{
	SPEK_VTX_US = 0,
	SPEK_VTX_EU = 1,
} VTX_REGION;

typedef enum
{
	
	SPEK_VTX_ACTIVE = 0,		//turn on power
	SPEK_VTX_PIT    = 1,		//low power mode while in pit
} VTX_PIT;

typedef struct
{
	uint8_t vtxChannel;
	VTX_BAND vtxBand;
	VTX_POWER vtxPower;
	VTX_REGION vtxRegion;
	VTX_PIT vtxPit;
} SPM_VTX_DATA;



//TEXT GEN interface table
typedef struct {
	const char *name;
	const uint32_t type;
	const char *group;
	void *ptr;
	float Min;
	float Max;
	float Default;
	const char *strDefault;
} interface_table;

enum
{
    IDLE=0,
    CHANGING_SETTING=1,
    SAVING=2,
	STARTUP=3,

};

typedef struct pidSpektrumTelem_t
{
    uint8_t status ;
    uint32_t waitTime;
    uint32_t currentTime;
    int32_t row;
    int32_t column;
    int32_t columnAxis;
    int32_t vStickStatus;
    int32_t hStickStatus;
} pidSpektrumTelem_t;

extern pidSpektrumTelem_t pidSpektrumTelem;

extern uint32_t VtxSpektrumBandAndChannelToVtxBandChannel(VTX_BAND vtxBand, uint8_t channel);
extern void     InitSpektrumTelemetry(void);
extern void     sendSpektrumSRXL(uint32_t baseAddress, uint8_t packetSize);
extern void     sendSpektrumTelem(void);
extern void     sendSpektrumBind(void);
extern void     textMenuUpdate(void);
extern uint16_t srxlCrc16(uint16_t crc, uint8_t data, uint16_t poly);
