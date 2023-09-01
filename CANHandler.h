#ifndef CANHandler_h
#define CANHandler_h

#include "Arduino.h"
#include <mcp_can.h>

#define CAN0_CS 53  // Замените 10 на номер пина, который вы используете для CS CAN0
#define CAN0_INT 3  // Замените 2 на номер пина, который вы используете для INT CAN0


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
