#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <mcp_can.h>
#include <TaskScheduler.h>
#include "RealDash.h"
#include "SHT31Helper.h"
#include "display.h"
#include "CANHandler.h"

#define TFT_LED  8  // Пин для управления подсветкой дисплея

#define EGT1_LED A10
#define EGT2_LED A11
#define EGT3_LED A12
#define EGT4_LED A13
#define EGT5_LED A14
#define EGT6_LED A15

#if !defined(__AVR_ATmega2560__)
#error "This code is intended for an Arduino Mega 2560 only!"
#endif

#if DISP1_ACTIVE && defined DISP1_USE_ST7735_SPI
 Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#endif

CANHandler can;
Scheduler ts;

float prevTemp = 0.0;
float prevHum = 0.0;
unsigned long lastUpdateTime = 0;

const int TOTAL_DIGITAL_PINS = 49;
const int TOTAL_ANALOG_PINS = 9;

int digitalPins[TOTAL_DIGITAL_PINS] = {0};
int analogPins[TOTAL_ANALOG_PINS] = {0};

int digitalPinsBuffer[TOTAL_DIGITAL_PINS] = {0};
int analogPinsBuffer[TOTAL_ANALOG_PINS] = {0};

byte sht31Operational = 0;  // 0 означает, что датчик не работает; 1 означает, что работает.
byte sht31StateChanged = 0;  // 0 означает, что состояние не изменилось; 1 означает, что изменилось.
byte CAN0Operational = 0;  // 0 - CAN0 не работает; 1 - CAN0 работает
byte CAN1Operational = 0;  // 0 - CAN1 не работает; 1 - CAN1 работает

void sendAnalogDataToCAN0() {
    // Отправка аналоговых данных на CAN0
    // Например, считывание сенсоров и отправка данных на шину CAN0
}

void initialiseRealDash() {
  // Здесь добавьте код инициализации для RealDash, если это необходимо.
  // Например, настройка пинов или инициализация специфических параметров.
}

void updateRealDashData() {
  // Здесь добавьте код для обновления или отправки данных в RealDash.
}

bool checkDigitalPinsChanged() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    if (digitalPinsBuffer[i] != digitalPins[i]) {
      return true;
    }
  }
  return false;
}

bool checkAnalogPinsChanged() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    if (analogPinsBuffer[i] != analogPins[i]) {
      return true;
    }
  }
  return false;

}

void updateDigitalBufferAndSave() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    digitalPinsBuffer[i] = digitalPins[i];
  }
  saveStateToEEPROM();
}

void updateAnalogBufferAndSave() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    analogPinsBuffer[i] = analogPins[i];
  }
  //saveAnalogStatesToEEPROM();
}

void saveStateToEEPROM() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    EEPROM.write(i - 22, digitalPins[i]);
  }
}

void saveAnalogStatesToEEPROM() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    int value = analogPins[i];
    // EEPROM.write(28 + i * 2, value & 0xFF);
    // EEPROM.write(28 + i * 2 + 1, (value >> 8) & 0xFF);
  }
}

void RealDashUpdateTask() {
  // Здесь вызывайте функцию или функции, которые обновляют данные для RealDash.
  updateRealDashData();
}

void setup() {
 
    can.initialiseCAN0();
  
  if (!initSHT31()) {
    // Обработка ошибки инициализации SHT31
  }

  // Инициализация для RealDash:
  initialiseRealDash();

  const int SERIAL_SPEED = 115200;
  Serial.begin(SERIAL_SPEED);
  Serial1.begin(SERIAL_SPEED);
  Serial2.begin(SERIAL_SPEED);
  Serial3.begin(SERIAL_SPEED);
  
  pinMode(TFT_LED, OUTPUT);      // Устанавливаем пин подсветки как выход
  // Начнем с выключенной подсветки
  analogWrite(TFT_LED, 0);
  initialise_display();
  
  pinMode(EGT1_LED, OUTPUT);
  pinMode(EGT2_LED, OUTPUT);
  pinMode(EGT3_LED, OUTPUT);
  pinMode(EGT4_LED, OUTPUT);
  pinMode(EGT5_LED, OUTPUT);
  pinMode(EGT6_LED, OUTPUT);

  digitalWrite(EGT1_LED, LOW);
  digitalWrite(EGT2_LED, LOW);
  digitalWrite(EGT3_LED, LOW);
  digitalWrite(EGT4_LED, LOW);
  digitalWrite(EGT5_LED, LOW);
  digitalWrite(EGT6_LED, LOW);

  pinMode(CAN0_INT, INPUT);

  for (int i = A0; i < A0 + TOTAL_ANALOG_PINS; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    pinMode(i, INPUT);
  }
  
}

void loop() {
  can.CAN0_INT_routine();
  can.sendAnalogData();
  ts.execute();
}
 void driveDisplayTask() {
  driveDisplay();
 }

void ReadDigitalStatuses() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    digitalPins[i] = digitalRead(i);
  }
  if (checkDigitalPinsChanged()) {
    updateDigitalBufferAndSave();
  }
}

void ReadAnalogStatuses() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    analogPins[i] = analogRead(i);
  }
  if (checkAnalogPinsChanged()) {
    updateAnalogBufferAndSave();
   
  }
}

void SHT31Task() {
  float temp = readTemperature();
  float hum = readHumidity();

  // Проверка, работает ли датчик
  if (temp == -1 || hum == -1) {  // Предположим, что значения -1 означают ошибку чтения.
    sht31Operational = 0;
  } else {
    sht31Operational = 1;
  }

  // Проверка изменения состояния
  if (abs(temp - prevTemp) >= 0.1 || abs(hum - prevHum) >= 0.1) {
    sht31StateChanged = 1;
  } else {
    sht31StateChanged = 0;
  }

  if (abs(temp - prevTemp) >= 0.1) {
    prevTemp = temp;
    //displayTemperature(temp);
  }

  if (abs(hum - prevHum) >= 0.1) {
    prevHum = hum;
    //displayHumidity(hum);
  }

}

Task t1(500, TASK_FOREVER, &driveDisplayTask, &ts, true);
Task t2(100, TASK_FOREVER, &sendAnalogDataToCAN0, &ts, true);  
Task t3(500, TASK_FOREVER, &ReadDigitalStatuses, &ts, true);
Task t4(100, TASK_FOREVER, &ReadAnalogStatuses, &ts, true);
Task t5(500, TASK_FOREVER, &SHT31Task, &ts, true);
Task t6(100, TASK_FOREVER, &RealDashUpdateTask, &ts, true);
