#pragma once



extern volatile uint8_t rfVtxRxBuffer[];

extern void     InitRfVtx(uint32_t usartNumber);
extern uint32_t RfVtxOff(void);
extern uint32_t RfVtxBaud(void);
extern uint32_t RfVtxOn25(void);
extern uint32_t RfVtxOn200(void);
extern uint32_t RfVtxBand(uint32_t band);
extern uint32_t RfVtxChannel(uint32_t channel);
