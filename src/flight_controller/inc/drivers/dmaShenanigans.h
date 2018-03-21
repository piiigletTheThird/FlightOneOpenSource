#define WS2812_MAX_LEDS 64
#define MAX_LED_COLORS 7


#define DMA_INACTIVE           0
#define DMA_OUTPUT_WS2812_LEDS 1
#define DMA_OUTPUT_ESC_1WIRE   2
#define DMA_OUTPUT_DSHOT       3
#define DMA_OUTPUT_SPORT       4
#define DMA_SPI_GYRO           5
#define DMA_SPI_FLASH          5
#define DMA_USART_RX           6
#define DMA_USART_TX           7
#define DMA_USART_TELEM        8
#define DMA_ADC                9


typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} ws2812Led_t;


typedef struct {
    uint32_t ledCount;
    uint32_t ledRed;
    uint32_t ledGreen;
    uint32_t ledBlue;
    uint32_t ledMode;
    uint32_t ledOnWithUsb;
} led_config;


typedef struct {
	uint32_t enabled;
	motor_type ws2812Actuator;
} ws2812_led_record;

extern uint32_t ddshot48To49[];
extern ws2812_led_record ws2812LedRecord;


extern motor_type ws2812Actuator;
extern ws2812Led_t WS2812_IO_colors[];


extern void OutputDDShotDma(motor_type actuator, int reverse, int digitalThrottle);
extern void InitDshotOutputOnMotors(uint32_t usedFor);
extern void InitDmaInputOnMotors(motor_type actuator);
extern void InitDmaOutputOnMotors(uint32_t usedFor);
extern void Ws2812LedInit(void);
extern void ws2812_led_update(uint32_t nLeds);
extern void SetLEDColor(uint8_t newColor);
extern void OutputSerialDmaByte(uint8_t *serialOutBuffer, uint32_t outputLength, motor_type actuator, uint32_t msb, uint32_t sendFrame, uint32_t noEndPadding);
extern int  InitWs2812(void);


extern void     InitDmaOutputForSoftSerial(uint32_t usedFor, motor_type actuator);
extern void     DeInitDmaOutputForSoftSerial(motor_type actuator);
extern uint32_t IsDshotEnabled(void);
extern uint32_t IsDshotActiveOnActuator(motor_type actuator);
//extern void     InitAllowedSoftOutputs(void);
extern void     DeInitAllowedSoftOutputs(void);
extern uint32_t DoesDmaConflictWithActiveDmas(motor_type actuator);
extern void     SetActiveDmaToActuatorDma(motor_type actuator);
