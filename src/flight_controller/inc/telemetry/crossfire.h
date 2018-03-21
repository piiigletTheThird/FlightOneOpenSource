#pragma once

#define CRSF_SYNC_BYTE 0xC8

enum
{
	CRSF_TYPE_GPS       = 0x02,
	CRSF_TYPE_BATT      = 0x08,
	CRSF_TYPE_HB        = 0x0B,
	CRSF_TYPE_LINK_STAT = 0x14,
	CRSF_TYPE_RC        = 0x16,
	CRSF_TYPE_ATTITUDE  = 0x1E,
	CRSF_TYPE_FLIGHT_MD = 0x21,
};

enum
{
    CRSF_GPS_PAYLOAD_SIZE       = 15,
    CRSF_BATT_PAYLOAD_SIZE      = 8,
    CRSF_LINK_STAT_PAYLOAD_SIZE = 10,
    CRSF_RC_PAYLOAD_SIZE        = 22, //11 bits per channel * 16 channels packed into 22 bytes total
    CRSF_ATTITUDE_PAYLOAD_SIZE  = 6,
};

enum
{
    CRSF_ADD_BROADCAST         = 0x00,
    CRSF_ADD_TBS_CORE_PNP_PRO  = 0x08,
    CRSF_ADD_RESERVED1         = 0x8A,
    CRSF_ADD_CURRENT_SENSOR    = 0xC0,
    CRSF_ADD_TBS_BLACKBOX      = 0xC4,
    CRSF_ADD_COLIBRI_RACE_FC   = 0xC8,
    CRSF_ADD_RESERVED2         = 0xCA,
    CRSF_ADD_RACE_TAG          = 0xCC,
    CRSF_ADD_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADD_CRSF_RECEIVER     = 0xEC,
    CRSF_ADD_CRSF_TRANSMITTER  = 0xEE
};

extern uint8_t CrsfCrc8(uint8_t * ptr, uint8_t len);
extern void    SendCrsfTelem(void);
extern int     InitCrsfTelemetry(int usartNumber);