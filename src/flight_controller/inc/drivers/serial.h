#pragma once
#include "includes.h"

extern uint32_t lastRXPacket;
extern volatile int32_t processRxCodeNow;

unsigned char txTransimissionReady;

extern void UsartInit(uint32_t serialNumber);
extern void UsartDeInit(uint32_t serialNumber);
extern void UsartDmaInit(uint32_t serialNumber);
extern void InitBoardUsarts(void);
extern void DeInitBoardUsarts(void);
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern void USARTx_DMA_RX_IRQHandler(void);
extern void USARTx_DMA_TX_IRQHandler(void);
extern void USARTx_IRQHandler(void);
extern void SerialTxCallback(uint32_t callbackNumber);
extern void ProcessSerialRx(void);