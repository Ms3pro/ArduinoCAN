#ifndef REALDASH_H
#define REALDASH_H

#include "Arduino.h"


extern byte CAN0Operational;  // 0 - CAN0 не работает; 1 - CAN0 работает
extern byte CAN1Operational;  // 0 - CAN1 не работает; 1 - CAN1 работает
extern byte sht31Operational;// 0 означает, что датчик не работает; 1 означает, что работает.
extern byte sht31StateBits;// 0 означает, что состояние не изменилось; 1 означает, что изменилось. 
extern int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
extern byte digin22, digin23, digin24, digin25, digin26, digin27, digin28, digin29, digin30, digin31, digin32, digin33, digin34, digin35, digin36, digin37, digin38, 
digin39, digin40, digin41, digin42, digin43, digin44, digin45, digin46, digin47, digin48, digin49;
extern int16_t digout3, digout4, digout5, digout6, digout7;
extern int seconds, pw1, pw2, rpm;
extern float temp;
extern float hum;

void SendCANFramesToRealDash();
void SendCANFrameToRealDash(unsigned long canFrameId, const byte* frameData);

#endif
