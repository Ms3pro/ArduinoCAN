#include "CANHandler.h"


CANHandler::CANHandler() : CAN0(CAN0_CS) {
   
}

void CANHandler::CAN0_INT_routine() {
    receive_CAN0_message();
   
}

void CANHandler::initialiseCAN0() {
    uint8_t tryInit0 = 0;
    START_INIT0:
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);

}
}

void CANHandler::Send_CAN0_message(uint16_t theaddress, byte* thedata) {
    CAN0.sendMsgBuf(theaddress, 0, 8, thedata);
}

void CANHandler::receive_CAN0_message() {
   
}

void CANHandler::sendAnalogData() {
    byte thedata[8] = {0};

    
    thedata[0] = analogRead(A0) >> 8;
    thedata[1] = analogRead(A0) & 0xFF;
    thedata[2] = analogRead(A1) >> 8;
    thedata[3] = analogRead(A1) & 0xFF;
    thedata[4] = analogRead(A2) >> 8;
    thedata[5] = analogRead(A2) & 0xFF;
    Send_CAN0_message(0x690, thedata);

  
    thedata[0] = analogRead(A3) >> 8;
    thedata[1] = analogRead(A3) & 0xFF;
    thedata[2] = analogRead(A4) >> 8;
    thedata[3] = analogRead(A4) & 0xFF;
    thedata[4] = analogRead(A5) >> 8;
    thedata[5] = analogRead(A5) & 0xFF;
    Send_CAN0_message(0x691, thedata);
}
