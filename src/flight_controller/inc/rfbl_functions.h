#pragma once

extern uint32_t rfblVersion;
extern uint32_t cfg1Version;
extern uint32_t rebootAddress;
extern uint32_t bootCycles;
extern uint32_t bootDirection;

void WriteRfblBkRegs (void);
void ReadRfblBkRegs (void);

void upgradeRfbl(void);
void eraseRfbl(uint32_t firmwareSize);
extern int HandleRfbl(void);
extern int HandleFcStartupReg(void);
void HandleRfblDisasterPrevention (void);
