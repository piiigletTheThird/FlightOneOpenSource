#include "includes.h"


uint32_t rfblVersion;
uint32_t cfg1Version;
uint32_t rebootAddress;
uint32_t bootCycles;
uint32_t bootDirection;

int HandleRfbl(void)
{
    ReadRfblBkRegs();
#ifdef DEBUG
    HandleRfblDisasterPrevention();
#endif
	return(0);
}

void HandleRfblDisasterPrevention(void)
{
	//flight code is working and has been running at least 5 seconds. Safe to reset RFBL/Recovery disaster counter.
	bootDirection = BOOT_TO_APP_COMMAND;
	bootCycles    = (uint32_t)0x00000000;
	rebootAddress = ADDRESS_FLASH_START;
	WriteRfblBkRegs();
}

int HandleFcStartupReg(void)
{
    if (RtcReadBackupRegister(FC_STATUS_REG) == FC_STATUS_INFLIGHT) { //FC crashed while inflight.
    	//crashed FC startup
    } else if (RtcReadBackupRegister(FC_STATUS_REG) == BOOT_TO_SPEKTRUM9) {
    	//SpektrumBind (mainConfig.rcControlsConfig.bind);
    	RtcWriteBackupRegister(FC_STATUS_REG,FC_STATUS_STARTUP);
    } else if (RtcReadBackupRegister(FC_STATUS_REG) == BOOT_TO_SPEKTRUM5) {
    	//SpektrumBind (mainConfig.rcControlsConfig.bind);
    	RtcWriteBackupRegister(FC_STATUS_REG,FC_STATUS_STARTUP);
    } else if (RtcReadBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG) == BOOT_TO_SPEKTRUM9) {
		//SpektrumBind (mainConfig.rcControlsConfig.bind);
		RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_APP_COMMAND);
	} else if (RtcReadBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG) == BOOT_TO_SPEKTRUM5) {
		//SpektrumBind (mainConfig.rcControlsConfig.bind);
		RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_APP_COMMAND);
    } else {
    	RtcWriteBackupRegister(FC_STATUS_REG,FC_STATUS_STARTUP);
	}
	return(0);
}

void WriteRfblBkRegs(void)
{
	RtcWriteBackupRegister(RFBL_BKR_RFBL_VERSION_REG,   rfblVersion);
	RtcWriteBackupRegister(RFBL_BKR_CFG1_VERSION_REG,   cfg1Version);
	RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG, bootDirection);
	RtcWriteBackupRegister(RFBL_BKR_BOOT_CYCLES_REG,    bootCycles);
	RtcWriteBackupRegister(RFBL_BKR_BOOT_ADDRESSS_REG,  rebootAddress); //todo: make configurable
}

void ReadRfblBkRegs(void)
{
    rfblVersion   = RtcReadBackupRegister(RFBL_BKR_RFBL_VERSION_REG);
    cfg1Version   = RtcReadBackupRegister(RFBL_BKR_CFG1_VERSION_REG);
    bootDirection = RtcReadBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG);
    bootCycles    = RtcReadBackupRegister(RFBL_BKR_BOOT_CYCLES_REG);
    rebootAddress = RtcReadBackupRegister(RFBL_BKR_BOOT_ADDRESSS_REG);
}

void upgradeRfbl(void)
{
	/*
	uint16_t rfblSize          = 0x0000;
	uint32_t rfblAddress       = 0x00000000;
	uint32_t wordOffset2       = 0x00000000;
	uint32_t addressFlashStart = ADDRESS_FLASH_START;
	uint32_t addressFlashEnd   = ADDRESS_FLASH_END;
	uint32_t addressRfblStart  = ADDRESS_RFBL_START;
	uint32_t firmwareFinderData[3];
	bool continueOn = false;
	FLASH_Status status = 0;
	uint32_t data32;
	int8_t attemptsRemaining = 5;
	//FADE 43F4  A62F E81A
	//F443 DEFA  1AE8 2FA6
	for (uint32_t wordOffset = addressFlashStart; wordOffset < addressFlashEnd; wordOffset += 4) { //scan up to 1mb of flash starting at after rfbl //todo:set per mcu
		//flash goes like this. RFBL -> FW -> ESCs -> fade43f4 a62fe81a -> RFBL SIZE in 16 bit hex variable -> fade43f4 a62fe81a -> RFBL
		memcpy( &firmwareFinderData, (char *) wordOffset, sizeof(firmwareFinderData) );
		if ( (firmwareFinderData[0] == RFBLMR1) && (firmwareFinderData[1] == RFBLMR2) ) { //RFBLM1 and 2 are different enfian of RFBLMR1 and 2
			rfblSize = (uint16_t)( ( (firmwareFinderData[2] & 0xFF) << 8) | ((firmwareFinderData[2] >> 8) & 0xFF)) ; //endian
			rfblAddress = wordOffset+20;
			memcpy( &firmwareFinderData, (char *) wordOffset+12, sizeof(firmwareFinderData) );
			if ( (firmwareFinderData[0] == RFBLMR1) && (firmwareFinderData[1] == RFBLMR2) ) {
				continueOn = true;
				break;
			}
		}
	}
	if (!continueOn) {
		return;
	}
	SKIP_GYRO=true;
	eraseRfbl(rfblSize);
	FLASH_Unlock();
	while (attemptsRemaining--) {
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
		for (uint32_t wordOffset = rfblAddress; wordOffset < (rfblAddress + rfblSize); wordOffset += 4) {
			memcpy( &data32, (char *) wordOffset, sizeof(data32) ); //copy from new RFBL one word at a time
			status = FLASH_ProgramWord( addressRfblStart + wordOffset2, data32);
			wordOffset2 += 4;
			if (status != FLASH_COMPLETE) {
				break;
			}
		}
		if (status == FLASH_COMPLETE) {
			break;
		}
	}
	FLASH_Lock();
	SKIP_GYRO=false;
	rfblVersion = RFBL_VERSION;
	cfg1Version = CFG1_VERSION;
	*/
}

void eraseRfbl(uint32_t firmwareSize)
{
	(void)(firmwareSize);
	/*
	FLASH_Status status       = 0;
    int8_t attemptsRemaining  = 5;
    uint32_t wordOffset       = 0;
    uint32_t addressRfblStart = ADDRESS_RFBL_START;
	SKIP_GYRO=true;
	FLASH_Unlock();
	while (attemptsRemaining--) {
		FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR );
		for (wordOffset = 0; wordOffset < firmwareSize; wordOffset += 4) {
			if (wordOffset % GetPageSize(addressRfblStart + wordOffset) == 0) {
				status = FLASH_EraseSector( GetSector(addressRfblStart + wordOffset), VoltageRange_3 );
                if (status != FLASH_COMPLETE) {
                    break;
                }
			}
		}
        if (status == FLASH_COMPLETE) {
            break;
        }
	}
	FLASH_Lock();
	SKIP_GYRO=false;
	*/
}
