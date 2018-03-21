#pragma once

extern uint32_t LoggingEnabled;
extern uint32_t LogThisIteration;
extern uint32_t flashCountdownFake;

extern int InitFlightLogger(void);
extern void EnableLogging(void);
extern void DisableLogging(void);
extern int DumbWriteString(char *string, int sizeOfString);
extern void UpdateBlackbox(pid_output flightPids[], float flightSetPoints[], float dpsGyroArray[], float filteredGyroData[], float filteredAccData[] );
extern void InlineWrite16To8 (int16_t data);
extern void FinishPage(void);
extern void WriteByteToFlash (uint8_t data);
