#pragma once

#define MAX_OSD_CHARMAP_SIZE 16384


#define OSD_SCREEN_BUFFER_SIZE 480    //max screen size in characters
#define OSD_EMPTY_CHAR ' '            //make sure these two don't match
#define OSD_EMPTY_CHAR_SHADOW_START 0 //make sure these two don't match

#define SCREEN_SIZE_PAL  480
#define SCREEN_SIZE_NTSC 390
#define VIDEO_LINES_PAL  16
#define VIDEO_LINES_NTSC 13


//MAX Register defines
#define MAX7456ADD_VM0           0x00
#define MAX7456ADD_VM1           0x01
#define MAX7456ADD_HOS           0x02
#define MAX7456ADD_VOS           0x03
#define MAX7456ADD_DMM           0x04
#define MAX7456ADD_DMAH          0x05
#define MAX7456ADD_DMAL          0x06
#define MAX7456ADD_DMDI          0x07
#define MAX7456ADD_CMM           0x08
#define MAX7456ADD_CMAH          0x09
#define MAX7456ADD_CMAL          0x0a
#define MAX7456ADD_CMDI          0x0b
#define MAX7456ADD_OSDM          0x0c
#define MAX7456ADD_RB0           0x10
#define MAX7456ADD_RB1           0x11
#define MAX7456ADD_RB2           0x12
#define MAX7456ADD_RB3           0x13
#define MAX7456ADD_RB4           0x14
#define MAX7456ADD_RB5           0x15
#define MAX7456ADD_RB6           0x16
#define MAX7456ADD_RB7           0x17
#define MAX7456ADD_RB8           0x18
#define MAX7456ADD_RB9           0x19
#define MAX7456ADD_RB10          0x1a
#define MAX7456ADD_RB11          0x1b
#define MAX7456ADD_RB12          0x1c
#define MAX7456ADD_RB13          0x1d
#define MAX7456ADD_RB14          0x1e
#define MAX7456ADD_RB15          0x1f
#define MAX7456ADD_OSDBL         0x6c
#define MAX7456ADD_STAT          0xA0

//MAX Command defines

// video mode register 0 bits
#define VIDEO_BUFFER_DISABLE     0x01
#define MAX7456_RESET            0x02
#define VERTICAL_SYNC_NEXT_VSYNC 0x04
#define OSD_ENABLE               0x08
#define SYNC_MODE_AUTO           0x00
#define SYNC_MODE_INTERNAL       0x30
#define SYNC_MODE_EXTERNAL       0x20


#define CLEAR_display 0x04
#define CLEAR_display_vert 0x06
#define END_string 0xff
#define WRITE_NVRAM 0xa0
// with NTSC
#define ENABLE_display 0x08
#define ENABLE_display_vert 0x0c
#define MAX7456_reset 0x02
#define DISABLE_display 0x00
#define STATUS_REG_NVR_BUSY 0x20


#define NVRAM_SIZE 54

//Signal defines
#define SIGNAL_NTSC              0x01
#define SIGNAL_PAL               0x02

#ifndef WHITEBRIGHTNESS
  #define WHITEBRIGHTNESS 0x01
#endif
#ifndef BLACKBRIGHTNESS
  #define BLACKBRIGHTNESS 0x00
#endif

#define BWBRIGHTNESS ((BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS)

typedef enum
{
	OSD_COMMAND_STATUS_UNKNOWN         = -1,
	OSD_COMMAND_STATUS_IDLE            =  0,
	OSD_COMMAND_STATUS_SPI_SENDING_DMA =  1,
	OSD_COMMAND_STATUS_SPI_SENDING_IRQ =  2,
	OSD_COMMAND_STATUS_SPI_SENDING_BLK =  3,
	OSD_COMMAND_STATUS_SPI_SENDING_DMA_VIDEO_TYPE =  4,
} max_osd_command_status_t;

typedef enum
{
	OSD_STATUS_UNKNOWN          = -1,
	OSD_STATUS_DISABLED         =  0,
	OSD_STATUS_ENABLED          =  1,
	OSD_STATUS_SPI_RESETTING    =  2,
	OSD_STATUS_LOADING_CHAR_MAP =  3,
} max_osd_status_t;

typedef enum
{
	OSD_STATUS_UPDATE_IDLE      =  0,
	OSD_STATUS_REQUEST_UPDATE   =  1,
	OSD_STATUS_UPDATE_APROVED   =  2,
	OSD_STATUS_UPDATING         =  3,
} max_osd_update_status_t;


typedef enum
{
	OSD_TYPE_UNKNOWN = -1,
	OSD_TYPE_SPI     =  0,
	OSD_TYPE_SERIAL  =  1,
} max_osd_type_t;

typedef enum
{
	VIDEO_MODE_UNKNOWN = -1,
	VIDEO_MODE_NTSC    = 0x00,
	VIDEO_MODE_PAL     = 0x40,
} video_mode_t;

typedef enum
{
	OSD_REQUEST_SCREEN_OFF = 0,
	OSD_REQUEST_SCREEN_1   = 1,
	OSD_REQUEST_SCREEN_2   = 2,
} osd_request_screen_t;

typedef struct
{
	max_osd_status_t         status;
	max_osd_update_status_t  osdUpdateStatus;
	max_osd_command_status_t commandStatus;
	uint32_t                 lastComandMillis;
	osd_request_screen_t     requestScreen;
	max_osd_type_t           type;
	video_mode_t             videoMode;
	uint8_t                  screenBuffer[OSD_SCREEN_BUFFER_SIZE];
	uint8_t                  shadowBuffer[OSD_SCREEN_BUFFER_SIZE];
	int                      osdRows;
	int                      osdColumnSize;
	int                      osdScreenSize;
} max_osd_record;


extern volatile max_osd_record maxOsdRecord;
extern const unsigned char charmap[];

extern int  InitMaxOsd(void);
extern void MaxOsdDmaTxCallback(uint32_t callbackNumber);
extern int  HandleMaxOsd(void);
extern void MaxOsdSetVidoOnOff(int on);
extern int  ForceUpdateCharMap(void);
extern int  ForceUpdateOsd(void);