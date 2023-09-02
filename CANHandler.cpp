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

    
    thedata[0] = adc0 >> 8;
    thedata[1] = adc0 & 0xFF;
    thedata[2] = adc1 >> 8;
    thedata[3] = adc1 & 0xFF;
    thedata[4] = adc2 >> 8;
    thedata[5] = adc2 & 0xFF;
    thedata[6] = adc3 & 0xFF;
    thedata[7] = adc3 >> 8;
   
    Send_CAN0_message(0x690, thedata);

  
    thedata[0] = adc4 >> 8;
    thedata[1] = adc4 & 0xFF;
    thedata[2] = adc5 >> 8;
    thedata[3] = adc5 & 0xFF;
    thedata[4] = 0x00;
    thedata[5] = 0x00;
    thedata[6] = 0x00;
    thedata[7] = 0x00;
    
    Send_CAN0_message(0x691, thedata);
}
