#ifndef SHT31_HELPER_H
#define SHT31_HELPER_H

#include <Adafruit_SHT31.h>

extern Adafruit_SHT31 sht31;

bool initSHT31();
float readTemperature();
float readHumidity();

#endif
