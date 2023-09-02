#include "SHT31Helper.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

bool initSHT31() {
  return sht31.begin(0x44);
}

float readTemperature() {
  return sht31.readTemperature();
}

float readHumidity() {
  return sht31.readHumidity();
}
