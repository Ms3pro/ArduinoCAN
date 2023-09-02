#include "RealDash.h"

void SendCANFramesToRealDash() {
    byte buf[8];
 /* memcpy(buf, &seconds, 2);
    memcpy(buf + 2, &pw1, 2);
    memcpy(buf + 4, &pw2, 2);
    memcpy(buf + 6, &rpm, 2);

    SendCANFrameToRealDash(3200, buf);
*/
    memcpy(buf, &adc0, 2);
    memcpy(buf + 2, &adc1, 2);
    memcpy(buf + 4, &adc2, 2);
    memcpy(buf + 6, &adc3, 2);

    SendCANFrameToRealDash(3200, buf);


}

void SendCANFrameToRealDash(unsigned long canFrameId, const byte* frameData) {
    const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
    Serial.write(serialBlockTag, 4);
    Serial.write((const byte*)&canFrameId, 4);
    Serial.write(frameData, 8); 
}
