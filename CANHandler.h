#ifndef CANHandler_h
#define CANHandler_h

#include "Arduino.h"
#include <mcp_can.h>

#define CAN0_CS 53  // Замените 10 на номер пина, который вы используете для CS CAN0
#define CAN0_INT 2  // Замените 2 на номер пина, который вы используете для INT CAN0

extern int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
extern byte digin22, digin23, digin24, digin25, digin26, digin27, digin28, digin29, digin30, digin31, digin32, digin33, digin34, digin35, digin36, digin37, digin38, 
digin39, digin40, digin41, digin42, digin43, digin44, digin45, digin46, digin47, digin48, digin49;
extern int16_t digout3, digout4, digout5, digout6, digout7;
extern float temp;
extern float hum;

class CANHandler {
public:
    CANHandler();
    void CAN0_INT_routine();
    void initialiseCAN0();
    void Send_CAN0_message(uint16_t theaddress, byte* thedata);
    void receive_CAN0_message();
    void sendAnalogData();
private:
    MCP_CAN CAN0;
 
};


#endif
