#ifndef REALDASH_H
#define REALDASH_H

#include <Arduino.h>


extern byte CAN0Operational;  // 0 - CAN0 не работает; 1 - CAN0 работает
extern byte CAN1Operational;  // 0 - CAN1 не работает; 1 - CAN1 работает
extern byte sht31Operational;// 0 означает, что датчик не работает; 1 означает, что работает.
extern byte sht31StateBits;// 0 означает, что состояние не изменилось; 1 означает, что изменилось. 

extern int seconds, pw1, pw2, rpm;
extern float temp;
extern float hum;

void SendCANFramesToRealDash();
void SendCANFrameToRealDash(unsigned long canFrameId, const byte* frameData);

#endif
