#include "includes.h"

const unsigned char crc8tab[256] = { 0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2, 0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64, 0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9}; 
 
#define CRSF_OUT_BUFFER_SIZE 32

uint8_t crsfOutBuffer[CRSF_OUT_BUFFER_SIZE];

static uint8_t CrsfAttitudeFrame(void);
static uint8_t CrsfBattFrame(void);
static uint8_t CrsfFlightModeFrame(void);
static uint8_t CrsfGpsFrame(void);

int InitCrsfTelemetry(int usartNumber)
{

    if (
        (mainConfig.rcControlsConfig.rxProtcol == USING_CRSF_B)
    )
    {
        //no need to do anything, crsf is set to both and is already setup. We use this usart
        return(1);
    }

    if (
        (mainConfig.rcControlsConfig.rxProtcol == USING_CRSF_T) ||
        (mainConfig.rcControlsConfig.rxProtcol == USING_CRSF_R)
    )
    {
        // R and T won't work is crsf usart is the same as crsf tlemetry
        //this should also be checked when serial is enabled
        if((uint32_t)usartNumber == mainConfig.rcControlsConfig.rxUsart)
        {
            //)invalid setup. set error mask
            ErrorHandler(BAD_TELEMETRY_SETUP);
            return(0);
        }
        else
        {
            //we setup the usart
            board.serials[usartNumber].enabled   = 1;
            board.serials[usartNumber].Protocol  = USING_CRSF_TELEM;
            board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 1;
            UsartDeInit(usartNumber); //deinits serial and associated pins and DMAs
            UsartInit(usartNumber);   //inits serial and associated pins and DMAs if used. Serial settings are set in serial.c
        }
    }

    return(1);
}


uint8_t CrsfCrc8(uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    
    for (uint8_t i=0; i<len; i++)
    {
         crc = crc8tab[crc ^ *ptr++];
    }
    
    return(crc);
}

static uint8_t CrsfAttitudeFrame(void)
{
    int16_t temp; //set

    //0x08 Battery sensor
    //Payload:
    //● int16_t Pitch angle ( rad / 10000 )
    //● int16_t Roll angle ( rad / 10000 )
    //● int16_t Yaw angle ( rad / 10000 ) 

    //dst/sync
    //length
    //type
    //payload
    //crc
    
    crsfOutBuffer[0] = CRSF_ADD_BROADCAST;
    crsfOutBuffer[1] = CRSF_ATTITUDE_PAYLOAD_SIZE + 2; //payload + frame type + crc
    crsfOutBuffer[2] = CRSF_TYPE_ATTITUDE;
    temp = (uint16_t)(InlineDegreesToRadians(pitchAttitude) * 10000);
    crsfOutBuffer[3] = (uint8_t)(temp >> 8);
    crsfOutBuffer[4] = (uint8_t)(temp);
    temp = (uint16_t)(InlineDegreesToRadians(rollAttitude)  * 10000);
    crsfOutBuffer[5] = (uint8_t)(temp >> 8);
    crsfOutBuffer[6] = (uint8_t)(temp);
    temp = (uint16_t)(InlineDegreesToRadians(yawAttitude)   * 10000);
    crsfOutBuffer[7] = (uint8_t)(temp >> 8);
    crsfOutBuffer[8] = (uint8_t)(temp);

    crsfOutBuffer[9] = CrsfCrc8(&crsfOutBuffer[2], CRSF_ATTITUDE_PAYLOAD_SIZE+1);

    return(10);
}

static uint8_t CrsfBattFrame(void)
{
    //0x08 Battery sensor
    //Payload:
    //● uint16_t Voltage ( mV * 100 )
    //● uint16_t Current ( mA * 100 )
    //● uint24_t Capacity ( mAh )
    //● uint8_t Battery remaining ( percent )  

    //dst/sync
    //length
    //type
    //payload
    //crc

    crsfOutBuffer[0] = CRSF_ADD_BROADCAST;
    crsfOutBuffer[1] = CRSF_BATT_PAYLOAD_SIZE + 2; //payload + frame type + crc
    crsfOutBuffer[2] = CRSF_TYPE_BATT;
    //voltage (16bit mV * 100)
    crsfOutBuffer[3] = (uint8_t)((uint16_t)(averageVoltage*10.0f) >> 8);
    crsfOutBuffer[4] = (uint8_t)((uint16_t)(averageVoltage*10.0f));
    //current (16bit mA * 100)
    crsfOutBuffer[5] = (uint8_t)((uint16_t)(adcCurrent*10.0f) >> 8);
    crsfOutBuffer[6] = (uint8_t)((uint16_t)(adcCurrent*10.0f));
    //capacity (24bit mAh)
    if (!mainConfig.telemConfig.crsfOtxCurHack)
    {
        crsfOutBuffer[7] = (uint8_t)(mainConfig.telemConfig.batSize >> 16);
        crsfOutBuffer[8] = (uint8_t)(mainConfig.telemConfig.batSize >> 8);
        crsfOutBuffer[9] = (uint8_t)(mainConfig.telemConfig.batSize);
        //capacity used (8bit percent)
        //crsfOutBuffer[10] = (uint8_t)(50);
        crsfOutBuffer[10] = (uint8_t)(((float)mainConfig.telemConfig.batSize - adcMAh) * 100.0f / (float)mainConfig.telemConfig.batSize);
    }
    else
    {
        crsfOutBuffer[7] = (uint8_t)((uint32_t)adcMAh >> 16);
        crsfOutBuffer[8] = (uint8_t)((uint32_t)adcMAh >> 8);
        crsfOutBuffer[9] = (uint8_t)((uint32_t)adcMAh);
        //capacity used (8bit percent)
        //crsfOutBuffer[10] = (uint8_t)(50);
        crsfOutBuffer[10] = (uint8_t)(100);
    }
    
    
    crsfOutBuffer[11] = CrsfCrc8(&crsfOutBuffer[2], CRSF_BATT_PAYLOAD_SIZE+1);

    return(12);
}

static uint8_t CrsfFlightModeFrame(void)
{
    //Payload:
    //char[] Flight mode ( Null-terminated string ) 

    //dst/sync
    //length
    //type
    //payload
    //crc

    bzero(crsfOutBuffer, CRSF_OUT_BUFFER_SIZE);
    crsfOutBuffer[0] = CRSF_ADD_BROADCAST;
    crsfOutBuffer[2] = CRSF_TYPE_FLIGHT_MD;

    if (ModeActive(M_ATTITUDE))
    {
        snprintf( (char *)&crsfOutBuffer[3], CRSF_OUT_BUFFER_SIZE-5, "RF1 LEVEL" );
        crsfOutBuffer[1] = 12; //payload + frame type + crc
        crsfOutBuffer[13] = CrsfCrc8(&crsfOutBuffer[2], 11);
        return(14);
    }
    else if (ModeActive(M_HORIZON))
    {
        snprintf( (char *)&crsfOutBuffer[3], CRSF_OUT_BUFFER_SIZE-5, "RF1 ACRO-LEVEL" );
        crsfOutBuffer[1] = 17; //payload + frame type + crc
        crsfOutBuffer[18] = CrsfCrc8(&crsfOutBuffer[2], 16);
        return(19);
    }
    else if (ModeActive(M_QUOPA))
    {
        snprintf( (char *)&crsfOutBuffer[3], CRSF_OUT_BUFFER_SIZE-5, "RF1 QUOPA" );
        crsfOutBuffer[1] = 17; //payload + frame type + crc
        crsfOutBuffer[18] = CrsfCrc8(&crsfOutBuffer[2], 16);
        return(19);
    }
    else
    {
        snprintf( (char *)&crsfOutBuffer[3], CRSF_OUT_BUFFER_SIZE-5, "RF1 ACRO" );
        crsfOutBuffer[1] = 11; //payload + frame type + crc
        crsfOutBuffer[12] = CrsfCrc8(&crsfOutBuffer[2], 10);
        return(13);
    }
}

static uint8_t CrsfGpsFrame(void)
{

    uint32_t temp = 0;
    //Payload:
    //● int32_t Latitude ( degree / 10`000`000 )
    //● int32_t Longitude (degree / 10`000`000 )
    //● uint16_t Groundspeed ( km/h / 100 )
    //● uint16_t GPS heading ( degree / 100 )
    //● uint16 Altitude ( meter - 1000m offset )
    //● uint8_t Satellites in use ( counter )

    //dst/sync
    //length
    //type
    //payload
    //crc
    
    crsfOutBuffer[0]  = CRSF_ADD_BROADCAST;
    crsfOutBuffer[1]  = CRSF_GPS_PAYLOAD_SIZE + 2; //payload + frame type + crc
    crsfOutBuffer[2]  = CRSF_TYPE_GPS;
    temp = (uint32_t)0; //LAT ( degree / 10`000`000 )
    crsfOutBuffer[3]  = (uint8_t)(temp >> 24);
    crsfOutBuffer[4]  = (uint8_t)(temp >> 16);
    crsfOutBuffer[5]  = (uint8_t)(temp >> 8);
    crsfOutBuffer[6]  = (uint8_t)(temp);
    temp = (uint32_t)0; //LAT ( degree / 10`000`000 )
    crsfOutBuffer[7]  = (uint8_t)(temp >> 24);
    crsfOutBuffer[8]  = (uint8_t)(temp >> 16);
    crsfOutBuffer[9]  = (uint8_t)(temp >> 8);
    crsfOutBuffer[10] = (uint8_t)(temp);
    temp = (uint32_t)0; //GS ( km/h / 100 )
    crsfOutBuffer[11] = (uint8_t)(temp >> 8);
    crsfOutBuffer[12] = (uint8_t)(temp);
    temp = (uint32_t)0; //HEAD ( degree / 100 )
    crsfOutBuffer[13] = (uint8_t)(temp >> 8);
    crsfOutBuffer[14] = (uint8_t)(temp);
    temp = (uint32_t)0; //ALT ( meter - 1000m offset )
    crsfOutBuffer[15] = (uint8_t)(temp >> 8);
    crsfOutBuffer[16] = (uint8_t)(temp);
    temp = (uint32_t)0; //SATs ( counter )
    static uint8_t tester = 0;
    tester++;
    temp = tester;
    crsfOutBuffer[17] = (uint8_t)(temp);

    crsfOutBuffer[18] = CrsfCrc8(&crsfOutBuffer[2], CRSF_GPS_PAYLOAD_SIZE+1);

    return(19);
}

void SendCrsfTelem(void)
{
    uint16_t packetSize; //set
    static int telemPacketNumber = 0;


    if (!telemEnabled)
		return;

    
    switch (telemPacketNumber)
    {
        case 0:
            packetSize = CrsfBattFrame();
            telemPacketNumber++;
            break;
        case 1:
            packetSize = CrsfAttitudeFrame();
            telemPacketNumber++;
            break;
        case 2:
            packetSize = CrsfFlightModeFrame();
            telemPacketNumber++;
            break;
        case 3:
            packetSize = CrsfGpsFrame();
            telemPacketNumber = 0;
            break;

    }
    

	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemCrsf) )
		{
            if (board.serials[serialNumber].Protocol == USING_CRSF_TELEM)
            {
                HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)crsfOutBuffer, packetSize);
                //HAL_UART_Transmit(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)crsfOutBuffer, packetSize, 10);
                break;
            }
            else if (board.serials[serialNumber].Protocol == USING_CRSF_B)
            {
                HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)crsfOutBuffer, packetSize);
                break;
            }
		}
	}
}