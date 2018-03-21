#include "includes.h"

uint32_t doingAutoA = 0;
esc_one_wire_status escOneWireStatus[16];
esc_hex_location escHexByPosition[51];
uint32_t oneWireActive = 0;
uint32_t oneWireHasRun = 0;

// EEPROM layout - BLHeli rev 20, bumped in 14.0
const BLHeli_EEprom_t BLHeli20_EEprom = {
    .BL_GOV_P_GAIN = 0,
    .BL_GOV_I_GAIN = 1,
    .BL_GOV_MODE = 2,
    .BL_MOT_GAIN = 4,
    .BL_STARTUP_PWR = 6,
    .BL_PWM_FREQ = 7,
    .BL_DIRECTION = 8,
    .BL_INPUT_POL = 9,
    .BL_INIT_L = 10,
    .BL_INIT_H = 11,
    .BL_ENABLE_TX = 12,
    .BL_COMM_TIMING = 18,
    .BL_PPM_MIN_THROTLE = 22,
    .BL_PPM_MAX_THROTLE = 23,
    .BL_BEEP_STRENGTH = 24,
    .BL_BEACON_STRENGTH = 25,
    .BL_BEACON_DELAY = 26,
    .BL_DEMAG_COMP = 28,
    .BL_BEC_VOLTAGE_HIGH = 29,
    .BL_PPM_CENTER = 30,
    .BL_TEMP_PROTECTION = 32,
    .BL_ENABLE_POWER_PROT = 33,
    .BL_ENABLE_PWM_INPUT = 34,
    .BL_PWM_DITHER = 35,
    .BL_BRAKE_ON_STOP = BLHELI_END_DATA,
    .BL_LED_CONTROL = BLHELI_END_DATA,
};
// EEPROM layout - BLHeli rev 21, bumped in 14.5
const BLHeli_EEprom_t BLHeli21_EEprom = {
    .BL_GOV_P_GAIN = 0,
    .BL_GOV_I_GAIN = 1,
    .BL_GOV_MODE = 2,
    .BL_MOT_GAIN = 4,
    .BL_STARTUP_PWR = 6,
    .BL_PWM_FREQ = 7,
    .BL_DIRECTION = 8,
    .BL_INPUT_POL = 9,
    .BL_INIT_L = 10,
    .BL_INIT_H = 11,
    .BL_ENABLE_TX = 12,
    .BL_COMM_TIMING = 18,
    .BL_PPM_MIN_THROTLE = 22,
    .BL_PPM_MAX_THROTLE = 23,
    .BL_BEEP_STRENGTH = 24,
    .BL_BEACON_STRENGTH = 25,
    .BL_BEACON_DELAY = 26,
    .BL_DEMAG_COMP = 28,
    .BL_BEC_VOLTAGE_HIGH = 29,
    .BL_PPM_CENTER = 30,
    .BL_TEMP_PROTECTION = 32,
    .BL_ENABLE_POWER_PROT = 33,
    .BL_ENABLE_PWM_INPUT = 34,
    .BL_PWM_DITHER = 35,
    .BL_BRAKE_ON_STOP = 36,
    .BL_LED_CONTROL = BLHELI_END_DATA,
};
// EEPROM layout - BLHeli_S rev 32
const BLHeli_EEprom_t BLHeliS32_EEprom = {
    .BL_GOV_P_GAIN = BLHELI_END_DATA,
    .BL_GOV_I_GAIN = BLHELI_END_DATA,
    .BL_GOV_MODE = BLHELI_END_DATA,
    .BL_MOT_GAIN = BLHELI_END_DATA,
    .BL_STARTUP_PWR = 6,
    .BL_PWM_FREQ = BLHELI_END_DATA,
    .BL_DIRECTION = 8,
    .BL_INPUT_POL = BLHELI_END_DATA,
    .BL_INIT_L = 10,
    .BL_INIT_H = 11,
    .BL_ENABLE_TX = 12,
    .BL_COMM_TIMING = 18,
    .BL_PPM_MIN_THROTLE = 22,
    .BL_PPM_MAX_THROTLE = 23,
    .BL_BEEP_STRENGTH = 24,
    .BL_BEACON_STRENGTH = 25,
    .BL_BEACON_DELAY = 26,
    .BL_DEMAG_COMP = 28,
    .BL_BEC_VOLTAGE_HIGH = BLHELI_END_DATA,
    .BL_PPM_CENTER = 30,
    .BL_TEMP_PROTECTION = 32,
    .BL_ENABLE_POWER_PROT = 33,
    .BL_ENABLE_PWM_INPUT = 34,
    .BL_PWM_DITHER = BLHELI_END_DATA,
    .BL_BRAKE_ON_STOP = 36,
    .BL_LED_CONTROL = 37,
};

const oneWireParameter_t* oneWireParameters[] = {
    &motorBeaconDelayParameter,
    &motorBeaconStrengthParameter,
    &motorBeepStrengthParameter,
    &motorBrakeOnStopParameter,
    &motorDemagParameter,
    &motorDirectionParameter,
    &motorEnableTxParameter,
    &motorFrequencyParameter,
    &motorMaxThrottleParameter,
    &motorMinThrottleParameter,
    &motorMidThrottleParameter,
    &motorStartupPowerParameter,
    &motorTimingParameter,
    &motorTempProtectionParameter,
    NULL
};


uint8_t  oneWireOutBuffer[ESC_BUF_SIZE];
uint8_t  oneWireInBuffer[ESC_BUF_SIZE];
volatile uint32_t oneWireInBufferIdx;
volatile uint32_t oneWireOngoing;

static const uint8_t  bootInit[] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
static const uint16_t signaturesAtmel[]  = {0x9307, 0x930A, 0x930F, 0x940B, 0};
static const uint16_t signaturesSilabs[] = {0xF310, 0xF330, 0xF410, 0xF390, 0xF850, 0xE8B1, 0xE8B2, 0};


static uint32_t ProcessEEprom(motor_type actuator, uint8_t eepromBuffer[], uint32_t hasDataSize);
static uint32_t SignatureMatch(uint16_t signature, const uint16_t *list, uint32_t listSize);
static void     AppendBlHeliCrc(uint8_t outBuffer[], uint32_t len);
//static void     SendHello(void);
static uint16_t Crc16Byte(uint16_t from, uint8_t byte);
//static uint32_t PollReadReadyBLHeli(void);
//static uint32_t ReadFlashAtmelBLHeli(ioMem_t *ioMem);
//static uint32_t ReadFlashSiLabsBLHeli(motor_type actuator, uint16_t address, uint32_t timeout);
//static uint32_t WriteFlashBLHeli(ioMem_t *ioMem);
//static uint32_t ReadEEpromAtmelBLHeli(ioMem_t *ioMem);
//static uint32_t ReadEEpromSiLabsBLHeli(motor_type actuator, uint32_t timeout);
//static uint32_t WriteEEpromAtmelBLHeli(ioMem_t *ioMem);
//static uint32_t WriteEEpromSiLabsBLHeli(ioMem_t *ioMem);
//static uint32_t PageEraseAtmelBLHeli(ioMem_t *ioMem);
//static uint32_t PageEraseSiLabsBLHeli(ioMem_t *ioMem);
//static uint32_t EepromEraseSiLabsBLHeli(ioMem_t *ioMem);


static uint32_t ConnectToBlheliBootloader(motor_type actuator, uint32_t timeout);
static uint32_t DisconnectBLHeli(motor_type actuator, uint32_t timeout);
//static uint32_t SendReadCommand(motor_type actuator, uint8_t cmd, uint16_t address, uint16_t length, uint32_t timeout);
static uint32_t WriteEEpromSiLabsBLHeli(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout);
static uint32_t ReadEEpromSiLabsBLHeli(motor_type actuator, uint8_t inBuffer[], uint32_t timeout);
static uint32_t WriteFlashSiLabsBLHeli(motor_type actuator, uint8_t outBuffer[], uint16_t address, uint16_t length, uint32_t timeout);
static uint32_t ReadFlashSiLabsBLHeli(motor_type actuator, uint8_t inBuffer[], uint16_t address, uint16_t length, uint32_t timeout);
static uint32_t PageEraseSiLabsBLHeli(motor_type actuator, uint16_t address, uint32_t timeout);
static uint32_t EepromEraseSiLabsBLHeli(motor_type actuator, uint32_t timeout);
static uint32_t SendCmdSetBuffer(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout)  __attribute__ ((unused));
static uint32_t SendCmdSetAddress(motor_type actuator, uint16_t address)  __attribute__ ((unused));

static const BLHeli_EEprom_t* GetBLHeliEEpromLayout(uint8_t data[]);

/*
const esc1WireProtocol_t BLHeliAtmelProtocol = {
//    .disconnect    = DisconnectBLHeli,
//    .pollReadReady = PollReadReadyBLHeli,
    .ReadFlash     = ReadFlashAtmelBLHeli,
//    .writeFlash    = WriteFlashBLHeli,
    .ReadEEprom    = ReadEEpromAtmelBLHeli,
//    .writeEEprom   = WriteEEpromAtmelBLHeli,
//    .pageErase     = PageEraseAtmelBLHeli,
//    .eepromErase   = PageEraseAtmelBLHeli,
};
*/

const esc1WireProtocol_t BLHeliSiLabsProtocol = {
    .Disconnect    = DisconnectBLHeli,
//    .pollReadReady = PollReadReadyBLHeli,
    .ReadFlash     = ReadFlashSiLabsBLHeli,
    .WriteFlash    = WriteFlashSiLabsBLHeli,
    .ReadEEprom    = ReadEEpromSiLabsBLHeli,
    .WriteEEprom   = WriteEEpromSiLabsBLHeli,
    .PageErase     = PageEraseSiLabsBLHeli,
    .EepromErase   = EepromEraseSiLabsBLHeli,
};

uint32_t ListAllEscHexesInFlash(void) {

	uint8_t  reportOut[50];
	uint32_t addressFlashStart = ADDRESS_ESC_START;
	uint32_t addressFlashEnd   = ADDRESS_FLASH_END;
	uint32_t wordOffset;
	uint32_t firmwareFinderData[5];
	uint8_t  firmwareFinderByteData[5 * 4];
	uint32_t x;
	uint32_t found = 1;
	uint8_t  escStart[3] = {0x02, 0x19, 0xFD};

	for (wordOffset = addressFlashStart; wordOffset < (addressFlashEnd - 0x1A6F); wordOffset++) { //scan up to 1mb of flash starting at after rfbl //todo:set per mcu
		//flash goes like this. RFBL -> FW -> ESCs -> fade43f4 a62fe81a -> RFBL SIZE in 16 bit hex variable -> fade43f4 a62fe81a -> RFBL
		memcpy( &firmwareFinderData, (char *) wordOffset, sizeof(firmwareFinderData) );
		for (x=0; x<sizeof(firmwareFinderData)/4; x++) {
			firmwareFinderData[x] = BigToLittleEndian32(firmwareFinderData[x]);
			firmwareFinderByteData[(x * 4)+0] = ((firmwareFinderData[x]>>24)&0xff);
			firmwareFinderByteData[(x * 4)+1] = ((firmwareFinderData[x]>>16)&0xff);
			firmwareFinderByteData[(x * 4)+2] = ((firmwareFinderData[x]>>8)&0xff);
			firmwareFinderByteData[(x * 4)+3] = ((firmwareFinderData[x]>>0)&0xff);
		}
		if (!strncmp((const char *)firmwareFinderByteData, (const char *)escStart, 3)) {
			memcpy( &firmwareFinderData, (char *) wordOffset+0x1A00+64, sizeof(firmwareFinderData) ); //get config data to look for ESC name
			for (x=0; x<sizeof(firmwareFinderData)/4; x++) {
				firmwareFinderData[x] = BigToLittleEndian32(firmwareFinderData[x]); //switch to little endian
				firmwareFinderByteData[(x * 4)+0] = ((firmwareFinderData[x]>>24)&0xff);
				firmwareFinderByteData[(x * 4)+1] = ((firmwareFinderData[x]>>16)&0xff);
				firmwareFinderByteData[(x * 4)+2] = ((firmwareFinderData[x]>>8)&0xff);
				firmwareFinderByteData[(x * 4)+3] = ((firmwareFinderData[x]>>0)&0xff);
			}
			firmwareFinderByteData[16] = 0;
			snprintf((char *)reportOut, 49, "ESC POSITION:%lu HEX:%s", found, (char *)firmwareFinderByteData);
			SendStatusReport((char *)reportOut);
			escHexByPosition[found].startAddress = wordOffset;
			escHexByPosition[found].endAddress = wordOffset + 0x1A6F;
			memcpy( &firmwareFinderData, (char *) wordOffset+0x1A00, sizeof(firmwareFinderData) ); //get versionnumber
			escHexByPosition[found].version = (uint16_t)( ((firmwareFinderData[0]<<8) & 0xff00) | ((firmwareFinderData[0]>>8) & 0x00ff) );
			found++;
		}
	}
	return (found);
}

void FindEscHexInFlashByName(uint8_t escStringName[], esc_hex_location *escHexLocation, uint32_t escNameStringSize) {

	uint32_t addressFlashStart = ADDRESS_ESC_START;
	uint32_t addressFlashEnd   = ADDRESS_FLASH_END;
	uint32_t wordOffset;
	uint32_t firmwareFinderData[5];
	uint8_t  firmwareFinderByteData[5 * 4];
	uint32_t x;
	uint8_t  escStart[3] = {0x02, 0x19, 0xFD};

	escHexLocation->startAddress = 0;
	escHexLocation->endAddress = 0;
	escHexLocation->version = 0;

	//from ESC start address the config starts at 0x1A00 and the ESC name is at 0x1A00 + 0x50. The firmware stops at 1A6F and includes 1A6F
	for (wordOffset = addressFlashStart; wordOffset < (addressFlashEnd - 0x1A6F); wordOffset++) //scan up to 1mb of flash starting at after rfbl //todo:set per mcu
	{
		//flash goes like this. RFBL -> FW -> ESCs -> fade43f4 a62fe81a -> RFBL SIZE in 16 bit hex variable -> fade43f4 a62fe81a -> RFBL
		memcpy( &firmwareFinderData, (char *) wordOffset, sizeof(firmwareFinderData) );
		for (x=0; x<sizeof(firmwareFinderData)/4; x++)
		{
			firmwareFinderData[x] = BigToLittleEndian32(firmwareFinderData[x]);
			firmwareFinderByteData[(x * 4)+0] = ((firmwareFinderData[x]>>24)&0xff);
			firmwareFinderByteData[(x * 4)+1] = ((firmwareFinderData[x]>>16)&0xff);
			firmwareFinderByteData[(x * 4)+2] = ((firmwareFinderData[x]>>8)&0xff);
			firmwareFinderByteData[(x * 4)+3] = ((firmwareFinderData[x]>>0)&0xff);
		}
		if (!strncmp((const char *)firmwareFinderByteData, (const char *)escStart, 3))
		{
			memcpy( &firmwareFinderData, (char *) wordOffset+0x1A00+64, sizeof(firmwareFinderData) ); //get config data to look for ESC name
			for (x=0; x<sizeof(firmwareFinderData)/4; x++)
			{
				firmwareFinderData[x] = BigToLittleEndian32(firmwareFinderData[x]); //switch to little endian
				firmwareFinderByteData[(x * 4)+0] = ((firmwareFinderData[x]>>24)&0xff);
				firmwareFinderByteData[(x * 4)+1] = ((firmwareFinderData[x]>>16)&0xff);
				firmwareFinderByteData[(x * 4)+2] = ((firmwareFinderData[x]>>8)&0xff);
				firmwareFinderByteData[(x * 4)+3] = ((firmwareFinderData[x]>>0)&0xff);
			}
			if (!strncmp((const char *)firmwareFinderByteData, (const char *)escStringName, escNameStringSize))
			{
				escHexLocation->startAddress = wordOffset;
				escHexLocation->endAddress = wordOffset + 0x1A6F;
				memcpy( &firmwareFinderData, (char *) wordOffset+0x1A00, sizeof(firmwareFinderData) ); //get versionnumber
				//firmwareFinderData[0] = BigToLittleEndian32(firmwareFinderData[0]);
				escHexLocation->version = (uint16_t)( ((firmwareFinderData[0]<<8) & 0xff00) | ((firmwareFinderData[0]>>8) & 0x00ff) );
				return;
			}
		}
	}
}

#ifdef STM32F446xx
extern uint32_t used1Wire;
#endif
uint32_t OneWireInit(void)
{
#ifdef STM32F446xx
	used1Wire = 1;
#endif

	uint8_t  reportOut[50];
	uint32_t x;
	uint32_t tries = 0;
	uint32_t allWork = 1;
	uint32_t atLeastOneWorks = 0;
	uint8_t  inBuffer[127];
	uint32_t dataCount;
	uint32_t outputNumber;

	oneWireOngoing = 1;
	//need to deinit RX, Gyro, Flash... basically anything that uses DMA. Should make a skip for USART if it's being used for communication
	DeinitFlight();

	DelayMs(500);

	while (tries < 5)
	{

		FeedTheDog();
		allWork = 1;
		tries++;

		for (x = 0; x < MAX_MOTOR_NUMBER; x++)
		{
			outputNumber = mainConfig.mixerConfig.motorOutput[x];
			InitDmaOutputForSoftSerial(DMA_OUTPUT_ESC_1WIRE, board.motors[outputNumber]);
			DelayMs(10);
			FeedTheDog();
		}

		DelayMs(1500);

		for (x = 0; x < MAX_MOTOR_NUMBER; x++)
		{
			FeedTheDog();
			outputNumber = mainConfig.mixerConfig.motorOutput[x];
			if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
			{
				FeedTheDog();
				escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled = 0;
				if (ConnectToBlheliBootloader(board.motors[outputNumber], 35))
				{
					FeedTheDog();
					dataCount = escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].esc1WireProtocol->ReadEEprom(board.motors[outputNumber], inBuffer, 150);
					if ( ProcessEEprom ( board.motors[outputNumber], inBuffer, dataCount) )
					{
						if (tries > 1)
						{
							FeedTheDog();
							snprintf((char *)reportOut, 49, "ESC:%lu Read Successful", board.motors[outputNumber].actuatorArrayNum);
							SendStatusReport((char *)reportOut);
						}
						atLeastOneWorks++;
					}
					else
					{
						if (tries > 1)
						{
							FeedTheDog();
							snprintf((char *)reportOut, 49, "ESC:%lu Read Failure", board.motors[outputNumber].actuatorArrayNum);
							SendStatusReport((char *)reportOut);
						}
						allWork = 0;
					}
				}
				FeedTheDog();
				DeInitDmaOutputForSoftSerial(board.motors[outputNumber]);
			}
		}

		if (allWork) //all active motors returned
		{
			return (atLeastOneWorks);
		}
		else if (atLeastOneWorks || tries < 3) //try at least twice, even if not a single ESC detected
		{
			InitActuators(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
			DelayMs(3500);
			snprintf((char *)reportOut, 49, "Reading ESCs...");
			SendStatusReport((char *)reportOut);
			DelayMs(3500);
			DeInitActuators();
			FeedTheDog();
		}
		else if (atLeastOneWorks == 0 && tries == 2 ) //tried twice, not a single ESC detected
		{
			return (0);
		}

	}

	return (atLeastOneWorks); //tried all used up, return what we've found.

}

static uint32_t ConnectToBlheliBootloader(motor_type actuator, uint32_t timeout) {

	//connect to ESC and get bootloader info

	memcpy(oneWireOutBuffer, bootInit, sizeof(bootInit));
	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(oneWireOutBuffer, sizeof(bootInit), oneWireInBuffer, actuator, timeout);

	escOneWireStatus[actuator.actuatorArrayNum].escBootloaderMode = 0;

	//if ( (oneWireInBuffer[0] == '4') && (oneWireInBuffer[1] == '7') && (oneWireInBuffer[2] == '1') )
	if ( !strncmp((char *)oneWireInBuffer, "471", 3) && (oneWireInBufferIdx > 7) )
	{
		//4, 7, 1, d, 0xE8, 0xB2, 0x06, 0x01, 0x30
		// Format = BootMsg("471c"), SIGNATURE_HIGH, SIGNATURE_LOW, BootVersion (always 6), BootPages (,ACK)
		escOneWireStatus[actuator.actuatorArrayNum].escSignature = ( (oneWireInBuffer[4] << 8) | oneWireInBuffer[5] );
		escOneWireStatus[actuator.actuatorArrayNum].bootVersion  = oneWireInBuffer[6];
		escOneWireStatus[actuator.actuatorArrayNum].bootPages    = oneWireInBuffer[7];

		if (SignatureMatch(escOneWireStatus[actuator.actuatorArrayNum].escSignature, signaturesSilabs, (sizeof(signaturesSilabs)/2)))
		{
			escOneWireStatus[actuator.actuatorArrayNum].escBootloaderMode = BLHBLM_BLHELI_SILABS;
			escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol  = &BLHeliSiLabsProtocol;

		}
		else if (SignatureMatch(escOneWireStatus[actuator.actuatorArrayNum].escSignature, signaturesAtmel, (sizeof(signaturesAtmel)/2)))
		{
//			escOneWireStatus[actuator.actuatorArrayNum].escBootloaderMode = BLHBLM_BLHELI_ATMEL;
//			escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol  = &BLHeliAtmelProtocol;
		}
		else
		{
			escOneWireStatus[actuator.actuatorArrayNum].escBootloaderMode = 0;
		}

	}

	return (escOneWireStatus[actuator.actuatorArrayNum].escBootloaderMode);

}

static uint32_t SendCmdSetAddress(motor_type actuator, uint16_t address) {

	if ((address == 0xffff)) {
		return (1);
	}

	oneWireOutBuffer[0] = CMD_SET_ADDRESS;
	oneWireOutBuffer[1] = 0;
	oneWireOutBuffer[2] = (address >> 8);
	oneWireOutBuffer[3] = (address & 0xff);

	AppendBlHeliCrc(oneWireOutBuffer, 4);

	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(oneWireOutBuffer, 6, oneWireInBuffer, actuator, 10); //150ms timeout is way more than we need

	if ( (oneWireInBufferIdx > 0) && (oneWireInBuffer[0] == RET_SUCCESS) ) {
		return (1);
	}

	return (0);
}

static uint32_t SendCmdSetBuffer(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout) {

	uint8_t cmdBuffer[] = {CMD_SET_BUFFER, 0, (length >> 8), (length & 0xff), 0, 0};

	AppendBlHeliCrc(cmdBuffer, 4);

	//send the command to write to buffer
	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(cmdBuffer, 6, oneWireInBuffer, actuator, 10);

	if ( oneWireInBufferIdx == 0 ) {

		AppendBlHeliCrc(outBuffer, length);

		//send the data
		oneWireInBufferIdx = SoftSerialSendReceiveBlocking(outBuffer, (length + 2), oneWireInBuffer, actuator, timeout); //150ms timeout is way more than we need

		if ( (oneWireInBufferIdx > 0) && (oneWireInBuffer[0] == RET_SUCCESS) ) {
			return (1);
		}

	}

	return (0);
}

/*
static uint32_t SendWriteCommand(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout) {

	uint8_t cmdBuffer[] = {CMD_PROG_FLASH, 0x01, 0, 0};

	AppendBlHeliCrc(cmdBuffer, 4);

	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(cmdBuffer, 6, oneWireInBuffer, actuator, 10);

	if ( oneWireInBufferIdx == 0 ) {

		AppendBlHeliCrc(outBuffer, length);

		oneWireInBufferIdx = SoftSerialSendReceiveBlocking(outBuffer, (length + 2), oneWireInBuffer, actuator, timeout); //150ms timeout is way more than we need

		if ( (oneWireInBufferIdx > 0) && (oneWireInBuffer[0] == RET_SUCCESS) ) {
			return (1);
		}

	}

	return (0);
}
*/

uint32_t OneWireSaveConfig(motor_type actuator) {

	uint32_t hasData;
	uint16_t checkCrc1;
	uint16_t checkCrc2;
	uint16_t retry;
	uint8_t inOutBuffer[127];

	const BLHeli_EEprom_t *layout = escOneWireStatus[actuator.actuatorArrayNum].BLHeliEEpromLayout;

	if (!escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.processed)
	{
		return (0);
	}

	retry = 10;

	while(retry > 0) {

		FeedTheDog();
		DelayMs(25);

		//read flash to inBuffer
		hasData = escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->ReadEEprom(actuator, inOutBuffer, 150);

		//if successful we change flash
		if ( (hasData > 121) && (inOutBuffer[122] == RET_SUCCESS) )
		{

			//check CRC:
			checkCrc1 = (uint16_t)( ( (inOutBuffer[120] & 0xFF) << 8) | ((inOutBuffer[121] >> 8) & 0xFF));
			AppendBlHeliCrc(inOutBuffer, 120);
			checkCrc2 = (uint16_t)( ( (inOutBuffer[120] & 0xFF) << 8) | ((inOutBuffer[121] >> 8) & 0xFF));

			//if CRC is valid we have good data
			if (checkCrc1 == checkCrc2)
			{
/*
                     <select class="multi_change" id="startup_tone" name="startup_tone" style="display: none;" disabled>
                      <option></option>
                      <option value="90">Normal</option>
                      <option value="91">Imperial March</option>
                      <option value="92">Crazy Start</option>
                      <option value="93">Game of Thrones</option>
                      <option value="94">Harrison Gale</option>
                      <option value="1">Stealth Mode</option>
                    </select>
 */
				//change flash to proper values
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_ENABLE_TX))]       = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.txprogramming;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEACON_DELAY))]    = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beacondelay;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEACON_STRENGTH))] = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beaconstrength;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEEP_STRENGTH))]   = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beepstrength;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BRAKE_ON_STOP))]   = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.brakeonstop;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_DEMAG_COMP))]      = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.demag;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_DIRECTION))]       = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.direction;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PWM_FREQ))]        = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.frequency;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MAX_THROTLE))] = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.maxthrottle;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MIN_THROTLE))] = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.minthrottle;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_CENTER))]      = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.midthrottle;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_STARTUP_PWR))]     = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.startuppower;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_TEMP_PROTECTION))] = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.tempprotection;
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_COMM_TIMING))]     = escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.timing;

				if (doingAutoA)
				{
					inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MAX_THROTLE))] = 219;
					inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MIN_THROTLE))] = 5;
				}
				//ninja update
				//inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BRAKE_ON_STOP))]   = 0x01; //enabled
				//inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_DEMAG_COMP))]      = 0x03; //high
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PWM_FREQ))]        = 0x03; //damped light
				//inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_STARTUP_PWR))]     = 0x05; //0.125
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_TEMP_PROTECTION))] = 0x00; //enabled
				inOutBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_CENTER))]      = (escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.maxthrottle - escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.minthrottle) / 2 + 1; //enabled

				//erase eeprom
				if (escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->EepromErase(actuator, 1000) &&
						escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->WriteEEprom(actuator, inOutBuffer, (BLHELI_END_DATA - 1), 150))
				{
					hasData = escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->ReadEEprom(actuator, inOutBuffer, 150);

					if (ProcessEEprom(actuator, inOutBuffer, hasData))
						return(1);

				}

			}

		}

		retry--;
	}

	return(0);

}

//1wire m2=upgrade
uint32_t BuiltInUpgradeSiLabsBLHeli(motor_type actuator, esc_hex_location escHexLocation)
{

	uint8_t  reportOut[50];
	uint32_t firmwareData32[32];
	uint8_t  firmwareData8[130]; //128b writes with room for crc
	uint32_t startAddress;
	uint32_t endAddress;
	uint16_t version;
	uint16_t writeLength;
	uint32_t wordOffset;
	uint32_t x;
	uint16_t y;
	uint16_t pageAddress;
	uint16_t currentPage;
	uint16_t retry;

	retry        = 10;
	retryFlash:
	currentPage  = 0;
	pageAddress  = 0;
	startAddress = escHexLocation.startAddress;
	endAddress   = escHexLocation.endAddress;
	version      = escHexLocation.version;

	// no ESC hex found
	if (!(version > 0)) {
		return (0);
	}

	if (retry < 1)
	{
		return (0);
	}
	wordOffset = startAddress;
	//one 8 bit address at a time
	for (y=0; y<(uint16_t)(endAddress - startAddress); y=y+128 ) {

		FeedTheDog();
		memcpy( &firmwareData32, (char *) wordOffset+y, sizeof(firmwareData32) );
		for (x=0; x<sizeof(firmwareData32)/4; x++) {
			firmwareData32[x] = BigToLittleEndian32(firmwareData32[x]);
			firmwareData8[(x * 4)+0] = ((firmwareData32[x]>>24)&0xff);
			firmwareData8[(x * 4)+1] = ((firmwareData32[x]>>16)&0xff);
			firmwareData8[(x * 4)+2] = ((firmwareData32[x]>>8)&0xff);
			firmwareData8[(x * 4)+3] = ((firmwareData32[x]>>0)&0xff);
		}
		writeLength = 128;
		if (y > (startAddress-endAddress)) {
			writeLength = writeLength - (y - (startAddress-endAddress));
		}
		//static uint32_t WriteEEpromSiLabsBLHeli(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout)
		//return ( WriteFlashSiLabsBLHeli(actuator, outBuffer, 0x1A00, length, timeout) );

		//each page needs to be erased.
		//check if this is a new page:
		//if so we erase it
		//page size is 512 bytes, which is hex 0x200
		pageAddress = 0x200 * (y / 0x200) + 1;
		if (pageAddress > currentPage) {
			snprintf((char *)reportOut, 49, "Flashing ESC:%lu: %lu percent done", actuator.actuatorArrayNum, (uint32_t)(( (float)y / (float)(endAddress - startAddress) ) * 100)  );
			SendStatusReport((char *)reportOut);
			if ( !escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->PageErase(actuator, y, 1000) ) {
				retry--;
				goto retryFlash;
			}
			currentPage = pageAddress;
		}

		//write data
		if ( !escOneWireStatus[actuator.actuatorArrayNum].esc1WireProtocol->WriteFlash(actuator, firmwareData8, y, writeLength, 200) ) {
			retry--;
			goto retryFlash;
		}
	}
	return ( OneWireSaveConfig(actuator) );
}

static uint32_t ProcessEEprom(motor_type actuator, uint8_t eepromBuffer[], uint32_t hasDataSize)
{

	uint16_t checkCrc1;
	uint16_t checkCrc2;
	const BLHeli_EEprom_t *layout;

	if ((hasDataSize > 121) && (eepromBuffer[122] == RET_SUCCESS))
	{

		//check CRC:
		checkCrc1 = (uint16_t)(((eepromBuffer[120] & 0xFF) << 8) | ((eepromBuffer[121] >> 8) & 0xFF));
		AppendBlHeliCrc(eepromBuffer, 120);
		checkCrc2 = (uint16_t)(((eepromBuffer[120] & 0xFF) << 8) | ((eepromBuffer[121] >> 8) & 0xFF));

		//if CRC is valid we have good data
		if (checkCrc1 == checkCrc2)
		{

			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.processed      = 0;
			escOneWireStatus[actuator.actuatorArrayNum].enabled = 1;
			bzero(escOneWireStatus[actuator.actuatorArrayNum].nameStr,      sizeof(escOneWireStatus[actuator.actuatorArrayNum].nameStr));
			bzero(escOneWireStatus[actuator.actuatorArrayNum].fwStr,        sizeof(escOneWireStatus[actuator.actuatorArrayNum].fwStr));
			bzero(escOneWireStatus[actuator.actuatorArrayNum].versionStr,   sizeof(escOneWireStatus[actuator.actuatorArrayNum].versionStr));
			memcpy( escOneWireStatus[actuator.actuatorArrayNum].nameStr,    eepromBuffer + 64, 16 );
			memcpy( escOneWireStatus[actuator.actuatorArrayNum].fwStr,      eepromBuffer + 80, 16 );
			memcpy( escOneWireStatus[actuator.actuatorArrayNum].versionStr, eepromBuffer + 95, 16 );
			escOneWireStatus[actuator.actuatorArrayNum].version = ( (eepromBuffer[0] << 8) | (eepromBuffer[1]) ) ;

			memcpy(escOneWireStatus[actuator.actuatorArrayNum].config, eepromBuffer, (BLHELI_END_DATA - 1));

			escOneWireStatus[actuator.actuatorArrayNum].BLHeliEEpromLayout = GetBLHeliEEpromLayout(eepromBuffer);

			FindEscHexInFlashByName( escOneWireStatus[actuator.actuatorArrayNum].nameStr, &escOneWireStatus[actuator.actuatorArrayNum].escHexLocation, 16);

			layout = escOneWireStatus[actuator.actuatorArrayNum].BLHeliEEpromLayout;
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.processed      = 1;
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.txprogramming  = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_ENABLE_TX))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beacondelay    = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEACON_DELAY))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beaconstrength = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEACON_STRENGTH))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beepstrength   = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BEEP_STRENGTH))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.brakeonstop    = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_BRAKE_ON_STOP))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.demag          = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_DEMAG_COMP))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.direction      = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_DIRECTION))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.frequency      = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PWM_FREQ))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.maxthrottle    = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MAX_THROTLE))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.minthrottle    = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_MIN_THROTLE))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.midthrottle    = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_PPM_CENTER))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.startuppower   = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_STARTUP_PWR))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.tempprotection = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_TEMP_PROTECTION))];
			escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.timing         = eepromBuffer[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + offsetof(BLHeli_EEprom_t, BL_COMM_TIMING))];

		}
		else
		{
			escOneWireStatus[actuator.actuatorArrayNum].enabled = 0;
		}
	}
	else
	{

		escOneWireStatus[actuator.actuatorArrayNum].enabled = 0;

	}

	return(escOneWireStatus[actuator.actuatorArrayNum].enabled);

}

static uint32_t ReadEEpromSiLabsBLHeli(motor_type actuator, uint8_t inBuffer[], uint32_t timeout) {

	// SiLabs has no EEPROM, just a flash section at 0x1A00

	return (ReadFlashSiLabsBLHeli(actuator, inBuffer, 0x1A00, 120, timeout));

}

/*
static uint32_t SendReadCommand(motor_type actuator, uint8_t cmd, uint16_t address, uint16_t length, uint32_t timeout) {

	uint8_t cmdBuffer[] = {cmd,(length & 0xff), 0, 0};

	if (!SendCmdSetAddress(actuator, address))
			return (0);

	AppendBlHeliCrc(cmdBuffer, 2);

	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(cmdBuffer, 4, oneWireInBuffer, actuator, timeout); //19200, reading 258 bytes should take around 135 ms

	//fills oneWireInBuffer to length 256 + 2 crc bytes
	if ( oneWireInBufferIdx == (uint32_t)(length + 2) ) { //did we get the amount of data we expected?
//		if (CheckCrc(oneWireInBuffer, length, (uint16_t)(oneWireInBuffer[length-1] >> 8) | (oneWireInBuffer[length] & 0xff) ) ) {
			//check the CRC

//cruiser
//		}
	}

//    return readBufBLHeli(esc, ioMem->data, len, true);

}
*/

static uint32_t DisconnectBLHeli(motor_type actuator, uint32_t timeout)
{
	oneWireOutBuffer[0] = CMD_RUN;
	oneWireOutBuffer[1] = 0;

	AppendBlHeliCrc(oneWireOutBuffer, 2);

	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(oneWireOutBuffer, 4, oneWireInBuffer, actuator, timeout);

	return (oneWireInBufferIdx);
}

static uint32_t ReadFlashSiLabsBLHeli(motor_type actuator, uint8_t inBuffer[], uint16_t address, uint16_t length, uint32_t timeout)
{

	uint8_t outBuffer[4];

	if (!SendCmdSetAddress(actuator, address))
		return (0);

	outBuffer[0] = CMD_READ_FLASH_SIL;
	outBuffer[1] = (length & 0xff);

	AppendBlHeliCrc(outBuffer, 2);

	//bzero(inBuffer, length);
	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(outBuffer, 4, inBuffer, actuator, timeout);

	inBuffer[123]=77;
	inBuffer[124]=77;
	return (oneWireInBufferIdx);
}

static uint32_t PageEraseSiLabsBLHeli(motor_type actuator, uint16_t address, uint32_t timeout)
{
	//set address of page to be erased
	if (!SendCmdSetAddress(actuator, address))
		return (0);

	//Send Erase command
	oneWireOutBuffer[0] = CMD_ERASE_FLASH;
	oneWireOutBuffer[1] = 0x01;

	//Append CRC
	AppendBlHeliCrc(oneWireOutBuffer, 2);

	//Send command and get result
	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(oneWireOutBuffer, 4, oneWireInBuffer, actuator, timeout);

	//check result
	if ( (oneWireInBufferIdx > 0) && (oneWireInBuffer[0] == RET_SUCCESS) ) {
		return (1);
	}

	return (0);

}

static uint32_t EepromEraseSiLabsBLHeli(motor_type actuator, uint32_t timeout)
{
	// SiLabs has no EEPROM, just a flash section at 0x1A00
	return( PageEraseSiLabsBLHeli(actuator, 0x1A00, timeout) );
}

static uint32_t WriteEEpromSiLabsBLHeli(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout)
{
	// SiLabs has no EEPROM, just a flash section at 0x1A00
	return ( WriteFlashSiLabsBLHeli(actuator, outBuffer, 0x1A00, length, timeout) );
}

static uint32_t WriteFlashSiLabsBLHeli(motor_type actuator, uint8_t outBuffer[], uint16_t address, uint16_t length, uint32_t timeout)
{

	//set address we want to write to
	if (!SendCmdSetAddress(actuator, address))
		return (0);

	//send length of data as well as data to be sent. This sends two commands.
	if (!SendCmdSetBuffer(actuator, outBuffer, length, timeout))
		return (0);

	uint8_t cmdBuffer[] = {CMD_PROG_FLASH, 0x01, 0, 0};

	AppendBlHeliCrc(cmdBuffer, 2);

	//Send the actual write command
	oneWireInBufferIdx = SoftSerialSendReceiveBlocking(cmdBuffer, 4, oneWireInBuffer, actuator, 30);

	if ( (oneWireInBufferIdx > 0) && (oneWireInBuffer[0] == RET_SUCCESS) ) {
		return (1);
	}

	return (0);

}

void OneWireDeinit(void) {

	uint32_t x;
	uint32_t outputNumber;

	oneWireHasRun = 1;
	for (x = 0; x < MAX_MOTOR_NUMBER; x++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[x];
		if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
		{

			escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].esc1WireProtocol->Disconnect(board.motors[outputNumber], 40);

			DeInitDmaOutputForSoftSerial(board.motors[outputNumber]);

		}

	}

	DelayMs(5);
	InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);

}











static const BLHeli_EEprom_t* GetBLHeliEEpromLayout(uint8_t data[]) {

	uint32_t layoutVersion;

    layoutVersion = data[2];

    if (layoutVersion == 32 || layoutVersion == 33)
    {
        return &BLHeliS32_EEprom;
    }
    else if (layoutVersion == 20)
    {
        return &BLHeli20_EEprom;
    }
    else if (layoutVersion == 21)
    {
        return &BLHeli21_EEprom;
    }

    return NULL;

}

static uint32_t SignatureMatch(uint16_t signature, const uint16_t *list, uint32_t listSize)
{

	uint32_t x;

	for (x=0;x<listSize;x++)
	{
	    if (signature == list[x]) {
            return(x+1);
        }
    }

	return(0);

}

static uint16_t Crc16Byte(uint16_t from, uint8_t byte) {
    uint16_t crc16 = from;
    for (int i = 0; i < 8; i++) {
        if (((byte & 0x01) ^ (crc16 & 0x0001)) != 0) {
            crc16 >>= 1;
            crc16 ^= 0xA001;
        } else {
            crc16 >>= 1;
        }
        byte >>= 1;
    }
    return crc16;
}

static void AppendBlHeliCrc(uint8_t outBuffer[], uint32_t len) {

	uint16_t crc = 0;
    uint32_t i;

    for(i = 0; i < len; i++) {
        crc = Crc16Byte(crc, outBuffer[i]);
    }

	outBuffer[i] = (crc & 0xff);
	outBuffer[i+1] = (crc >> 8);

}









int16_t Esc1WireSetParameter(motor_type actuator, const oneWireParameter_t *parameter, uint8_t buf[], int16_t value)
{

	(void)(buf);
	const BLHeli_EEprom_t *layout = escOneWireStatus[actuator.actuatorArrayNum].BLHeliEEpromLayout;

    if (layout == NULL)
    {
        return (-1);
    }

    if (!strcmp("txprogramming", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.txprogramming = value;
    if (!strcmp("beacondelay", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beacondelay = value;
    if (!strcmp("beaconstrength", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beaconstrength = value;
    if (!strcmp("beepstrength", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.beepstrength = value;
    if (!strcmp("brakeonstop", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.brakeonstop = value;
    if (!strcmp("demag", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.demag = value;
    if (!strcmp("direction", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.direction = value;
    if (!strcmp("frequency", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.frequency = value;
    if (!strcmp("maxthrottle", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.maxthrottle = value;
    if (!strcmp("minthrottle", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.minthrottle = value;
    if (!strcmp("startuppower", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.startuppower = value;
    if (!strcmp("tempprotection", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.tempprotection = value;
    if (!strcmp("timing", parameter->name))
    	escOneWireStatus[actuator.actuatorArrayNum].oneWireCurrentValues.timing = value;

    return (1);

}

int16_t Esc1WireParameterFromDump(motor_type actuator, const oneWireParameter_t *parameter, uint8_t buf[])
{

	const BLHeli_EEprom_t *layout = escOneWireStatus[actuator.actuatorArrayNum].BLHeliEEpromLayout;

    if (layout == NULL)
    {
        return (-1);
    }

    return ( buf[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + parameter->offset)] );

}

const char* OneWireParameterValueToName(const oneWireParameterValue_t *valuesList, uint8_t value)
{
    while (valuesList->name)
    {
    	FeedTheDog();
        if (value == valuesList->value)
        {
            return (valuesList->name);
        }
        valuesList++;
    }
    return ("NOT FOUND");
}

int16_t OneWireParameterNameToValue(const oneWireParameterValue_t *valuesList, const char *name)
{
    while (valuesList->name)
    {
    	FeedTheDog();
        if (strncasecmp(valuesList->name, name, strlen(valuesList->name)) == 0)
        {
            return valuesList->value;
        }
        valuesList++;
    }
    return (-1);
}

int16_t OneWireParameterValueToNumber(const oneWireParameterNumerical_t *numerical, uint8_t value)
{
    return ( numerical->step * ((int16_t)value) + numerical->offset );
}

uint8_t OneWireParameterNumberToValue(const oneWireParameterNumerical_t *numerical, int16_t value)
{
    return ( (uint8_t)((value - numerical->offset) / numerical->step) );
}

//int16_t esc1WireGetParameter(motor_type actuator, const oneWireParameter_t *parameter) {

//    escHardware_t *esc = &escHardware[escIndex];
//    const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
//    if (proto == NULL) {
//        return (-1);
//    }
//    const BLHeli_EEprom_t *layout = escDevice[escIndex].layout;
//    if (layout == NULL) {
//        return (-1);
//    }
//    return (readValueFromEEprom(esc, proto, *((uint8_t*)layout + parameter->offset)) );

//}

////////////////////////////////////////////////////////
// BLHeli parameter definitions
const oneWireParameterValue_t motorOnOffParameterList[] = {
    {0x00, "disable"},
    {0x01, "enable"},
    {0, NULL},
};
const oneWireParameter_t motorEnableTxParameter = {
    .name = "txprogramming",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_ENABLE_TX),
};
const oneWireParameterValue_t motorBeaconDelayList[] = {
	{0x01, "1min"},
	{0x02, "2min"},
	{0x03, "5min"},
	{0x04, "10min"},
	{0x05, "off"},
	{0, NULL},
};
const oneWireParameter_t motorBeaconDelayParameter = {
	.name = "beacondelay",
	.parameterNamed = motorBeaconDelayList,
	.offset = offsetof(BLHeli_EEprom_t, BL_BEACON_DELAY)
};
const oneWireParameterNumerical_t motorBeaconStrengthValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 0,
    .step = 1,
};
const oneWireParameter_t motorBeaconStrengthParameter = {
	.name = "beaconstrength",
	.parameterNamed = NULL,
    .parameterNumerical = &motorBeaconStrengthValues,
	.offset = offsetof(BLHeli_EEprom_t, BL_BEACON_STRENGTH)
};
const oneWireParameterNumerical_t motorBeepStrengthValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 0,
    .step = 1,
};
const oneWireParameter_t motorBeepStrengthParameter = {
	.name = "beepstrength",
	.parameterNamed = NULL,
    .parameterNumerical = &motorBeepStrengthValues,
	.offset = offsetof(BLHeli_EEprom_t, BL_BEEP_STRENGTH)
};
const oneWireParameter_t motorBrakeOnStopParameter = {
    .name = "brakeonstop",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_BRAKE_ON_STOP)
};
const oneWireParameterValue_t motorDemagParameterList[] = {
    {0x01, "off"},
    {0x02, "low"},
    {0x03, "high"},
    {0, NULL},
};
const oneWireParameter_t motorDemagParameter = {
    .name = "demag",
    .parameterNamed = motorDemagParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_DEMAG_COMP)
};
const oneWireParameterValue_t motorDirectionParameterList[] = {
    {0x01, "normal"},
    {0x02, "reversed"},
    {0x03, "bidirectional"},
    {0x04, "bidirectional_reversed"},
    {0, NULL},
};
const oneWireParameter_t motorDirectionParameter = {
    .name = "direction",
    .parameterNamed = motorDirectionParameterList,
	.offset = offsetof(BLHeli_EEprom_t, BL_DIRECTION)
};
const oneWireParameterValue_t motorFrequencyParameterList[] = {
    {0x01, "high"},
    {0x02, "low"},
    {0x03, "damped light"},
    {0, NULL},
};
const oneWireParameter_t motorFrequencyParameter = {
    .name = "frequency",
    .parameterNamed = motorFrequencyParameterList,
	.offset = offsetof(BLHeli_EEprom_t, BL_PWM_FREQ)
};
const oneWireParameterNumerical_t motorThrottleValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 1000,
    .step = 4,
};
const oneWireParameter_t motorMaxThrottleParameter = {
    .name = "maxthrottle",
    .parameterNamed = NULL,
    .parameterNumerical = &motorThrottleValues,
    .offset = offsetof(BLHeli_EEprom_t, BL_PPM_MAX_THROTLE)
};
const oneWireParameter_t motorMinThrottleParameter = {
    .name = "minthrottle",
    .parameterNamed = NULL,
    .parameterNumerical = &motorThrottleValues,
    .offset = offsetof(BLHeli_EEprom_t, BL_PPM_MIN_THROTLE)
};
const oneWireParameter_t motorMidThrottleParameter = {
    .name = "midthrottle",
    .parameterNamed = NULL,
    .parameterNumerical = &motorThrottleValues,
    .offset = offsetof(BLHeli_EEprom_t, BL_PPM_CENTER)
};
const oneWireParameterValue_t motorStartupPowerParameterList[] = {
    {0x01, "0.031"},
    {0x02, "0.047"},
    {0x03, "0.063"},
    {0x04, "0.094"},
    {0x05, "0.125"},
    {0x06, "0.188"},
    {0x07, "0.25"},
    {0x08, "0.38"},
    {0x09, "0.50"},
    {0x0A, "0.75"},
    {0x0B, "1.00"},
    {0x0C, "1.25"},
    {0x0D, "1.50"},
    {0, NULL},
};
const oneWireParameter_t motorStartupPowerParameter = {
    .name = "startuppower",
    .parameterNamed = motorStartupPowerParameterList,
	.offset = offsetof(BLHeli_EEprom_t, BL_STARTUP_PWR)
};
const oneWireParameter_t motorTempProtectionParameter = {
    .name = "tempprotection",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_TEMP_PROTECTION)
};
const oneWireParameterValue_t motorTimingParameterList[] = {
    {0x01, "low"},
    {0x02, "medlow"},
    {0x03, "medium"},
    {0x04, "medhigh"},
    {0x05, "high"},
    {0, NULL},
};
const oneWireParameter_t motorTimingParameter = {
    .name = "timing",
    .parameterNamed = motorTimingParameterList,
	.offset = offsetof(BLHeli_EEprom_t, BL_COMM_TIMING)
};
