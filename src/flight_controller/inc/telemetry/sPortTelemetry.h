#pragma once


#define REQUEST_ID     0x01
#define REQUEST_PIDS   0x02
#define REQUEST_RATES  0x03
#define REQUEST_VRX    0x04
#define REQUEST_MORE   0x05

#define ID_CMD_PRINT   0x12
#define ID_CMD_ERASE   0x13

#define ID_ROLL_KP     0x01
#define ID_ROLL_KI     0x02
#define ID_ROLL_KD     0x03
#define ID_PITCH_KP    0x04
#define ID_PITCH_KI    0x05
#define ID_PITCH_KD    0x06
#define ID_YAW_KP      0x07
#define ID_YAW_KI      0x08
#define ID_YAW_KD      0x09
#define ID_ROLL_RATE   0x0A
#define ID_ROLL_EXPO   0x0B
#define ID_ROLL_ACROP  0x0C
#define ID_PITCH_RATE  0x0D
#define ID_PITCH_EXPO  0x0E
#define ID_PITCH_ACROP 0x0F
#define ID_YAW_RATE    0x10
#define ID_YAW_EXPO    0x11
#define ID_YAW_ACROP   0x12

#define ID_DEVICE      0x13
#define ID_BAND        0x14
#define ID_CHANNEL     0x15
#define ID_POWER       0x16
#define ID_PIT         0x17


#define RSSI_ID                 0xf101
#define ADC1_ID                 0xf102
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_LAST_ID           0x030f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f


#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28

// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42


#define SPORT_FRAME_HEADER		0x10
#define BYTESTUFF				0x7d

//TODO: Hack to make this lua crap work
extern volatile uint32_t luaPacketPendingTime;
extern volatile int32_t  luaOutPacketOne;
extern uint32_t          transmitDataBufferIdx;
extern uint32_t          transmitDataBufferSent;

extern volatile uint8_t telemtryRxBuffer[];
extern void SendSmartPortLua(void);
extern void SendSmartPort(void);
extern void CheckIfSportReadyToSend(void);
extern void InitSport(uint32_t usartNumber);
extern void InitSoftSport(void);
extern void InitAllSport(void);
extern void DeInitSoftSport(void);
