#pragma once

enum 
{
	PERSISTANCE_FOUND_NONE = 0,
	PERSISTANCE_FOUND_ONE  = 1,
	PERSISTANCE_FOUND_TWO  = 2,
};

enum {
    PAGE_PROGRAM_IN_PROGRESS   = (1 << 0),
    MASS_ERASE_IN_PROGRESS     = (1 << 1),
    SECTOR_ERASE_IN_PROGRESS   = (1 << 2),
    CHIP_BUSY                  = (1 << 3),
    READ_STATUS_IN_PROGRESS    = (1 << 4),
	READ_ID_IN_PROGRESS        = (1 << 5),
	DMA_READ_ID_IN_PROGRESS    = (1 << 6),
	READ_ANDOR_WRITE_COMPLETE  = (1 << 7),
	DMA_DATA_READ_IN_PROGRESS  = (1 << 8),
	DMA_DATA_WRITE_IN_PROGRESS = (1 << 9),
	DATA_WRITE_IN_PROGRESS     = (1 << 10),
	IRQ_DATA_WRITE_IN_PROGRESS = (1 << 11),
	BLK_DATA_WRITE_IN_PROGRESS = (1 << 12),
};

//data in read buffer goes from 5 to 260. the first
//data in write buffer goes from 4 to 259. the first
#define FLASH_CHIP_BUFFER_WRITE_DATA_SIZE  256
#define FLASH_CHIP_BUFFER_WRITE_DATA_START 4
#define FLASH_CHIP_BUFFER_WRITE_DATA_END   259
#define FLASH_CHIP_BUFFER_READ_DATA_SIZE   256
#define FLASH_CHIP_BUFFER_READ_DATA_START  4
#define FLASH_CHIP_BUFFER_READ_DATA_END    259
#define FLASH_CHIP_BUFFER_SIZE             260
#define BUFFER_STATUS_FILLING_A 1
#define BUFFER_STATUS_FILLING_B 2

#define FLASH_DISABLED 0
#define STAT_FLASH_ENABLED 1
#define FLASH_FULL 2

extern volatile uint32_t flashWriteAddress;
extern uint8_t *flashTxBuffer;
extern uint8_t *flashRxBuffer;
extern volatile uint32_t flashWriteInProgress;

typedef struct {
	uint8_t rxBuffer[FLASH_CHIP_BUFFER_SIZE];
	uint8_t txBuffer[FLASH_CHIP_BUFFER_SIZE];
	volatile uint32_t txBufferPtr;
	volatile uint32_t rxBufferPtr;
} buffer_record;

//8bit storage
#define PERSISTANCE_VERSION 005
//#define PERSISTANCE_VERSION 002

//8bit storage
typedef struct
{
	uint8_t           version;
	uint16_t          generation;
	uint8_t           itteration;
	volatile float    motorTrim[8];
	volatile int8_t   yawKiTrim8[20];
	volatile int8_t   rollKiTrim8[20];
	volatile int8_t   pitchKiTrim8[20];
	volatile int8_t   geeForce[20];
	volatile uint32_t rememberence[20];
	uint8_t crc;
} __attribute__ ((__packed__)) persistance_data_record;

/*
typedef struct
{
	uint8_t version;
	uint16_t generation;
	uint8_t itteration;
	volatile float motorTrim[8];
	volatile float yawKiTrim[11];
	volatile float rollKiTrim[11];
	volatile float pitchKiTrim[11];
	volatile int8_t 
	uint8_t crc;
} persistance_data_record;
*/

typedef struct
{
	uint8_t enabled;
	uint8_t version;
	uint32_t start1;
	uint32_t start2;
	uint32_t end;
	uint32_t currentWriteAddress;
	uint8_t generation;
	uint8_t itteration;
	persistance_data_record data;
} persistance_record;

typedef struct {
	volatile uint32_t enabled;
	volatile uint32_t chipId;
	volatile uint32_t flashSectors;
	volatile uint32_t pagesPerSector;
	volatile uint32_t sectorSize;
	volatile uint32_t totalSize;
	volatile uint32_t pageSize;
	volatile uint32_t status;
	volatile uint32_t bufferStatus;
	uint8_t commandRxBuffer[4]; //used for replies of commands
	uint8_t commandTxBuffer[4]; //used for sending chip commands. Needs to be separate of data buffer since both can be in use at once
	buffer_record buffer[2];
	uint8_t bufferNum;
	volatile uint32_t currentWriteAddress;
} flash_info_record;

extern persistance_record persistance;
extern flash_info_record flashInfo;

extern int  CheckIfFlashBusy(void);
extern void FlashDeinit(void);
extern void FlashDmaRxCallback(uint32_t callbackNumber);
extern int  InitFlashChip(void);
extern void DataFlashBeginProgram(uint32_t address);
extern int  FlashChipWriteData(uint8_t *data, uint8_t length);
extern int  FlashChipReadData(uint32_t address, uint8_t *buffer, int length);
extern void DataFlashProgramPage(uint32_t address, uint8_t *data, uint16_t length);
extern void WriteEnableDataFlash(void);
extern int  MassEraseDataFlashByPage(int blocking, int all);
extern int  MassEraseDataFlash(int blocking);
extern void M25p16DmaWritePage(uint32_t address, uint8_t *txBuffer, uint8_t *rxBuffer);
extern int  M25p16ReadPage(uint32_t address, uint8_t *txBuffer, uint8_t *rxBuffer);
extern int  WriteEnableDataFlashDma(void);
extern void M25p16BlockingWritePage(uint32_t address, uint8_t *txBuffer);
extern void M25p16BlockingWritePagePartial(uint32_t address, uint8_t *txBuffer, uint32_t size);
extern void M25p16IrqWritePage(uint32_t address, uint8_t *txBuffer);
extern int  SavePersistance(void);
extern int  ResetPersistance(int preserveGeneration);
extern int  CheckIfFlashSpiBusy(void);