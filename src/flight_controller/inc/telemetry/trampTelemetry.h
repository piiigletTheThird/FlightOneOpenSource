#pragma once

extern int  InitTrampTelemetry(int usartNumber);
extern void DeInitTrampTelemetry(void);
extern void TrampSetFreq(int freq);
extern void TrampSetRfPower(int power);
extern int  TrampSetBandChannel(int bandChannel);
extern int  TrampGetSettings(void);
extern int  TrampSetPower(int power);
extern int  TrampSetPit(int pit);