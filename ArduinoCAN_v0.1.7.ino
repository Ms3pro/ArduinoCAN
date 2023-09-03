#include <EEPROM.h>
#include <TaskScheduler.h>
#include "pidautotuner.h"
#include "RealDash.h"
#include "SHT31Helper.h"
#include "display.h"
#include "CANHandler.h"

#define TFT_LED  8  // Пин для управления подсветкой дисплея
void initTFTBacklight() __attribute__((constructor));
void initTFTBacklight() {
    DDRH |= (1 << PH5);  // Устанавливаем пин 8 (PH5) как выходной на Arduino Mega
    PORTH &= ~(1 << PH5);  // Устанавливаем низкий уровень на пине 8 (PH5)
}

CANHandler can;
Scheduler ts;

#if !defined(__AVR_ATmega2560__)
#error "This code is intended for an Arduino Mega 2560 only!"
#endif

//СВЕТОДИОДЫ ПЕРЕГРЕВА ЕСЛИ ЕГТ БОЛЬШЕ 800 ГРАДУСОВ 
#define EGT1_LED A10
#define EGT2_LED A11
#define EGT3_LED A12
#define EGT4_LED A13
#define EGT5_LED A14
#define EGT6_LED A15

//ПРОВЕРКА РАБОТЫ СВЕТОДИОДОВ  ЕГТ КОГДА АРДУИНО ВКЛЮЧАЕТСЯ ИЛИ ПЕРЕЗАГРУЖАЕТСЯ 
const int EGT_LEDS[] = {EGT1_LED, EGT2_LED, EGT3_LED, EGT4_LED, EGT5_LED, EGT6_LED};
const unsigned long interval = 500;  // задержка в 500 миллисекунд
unsigned long previousMillis = 0;
int ledIndex = 0;
bool turningOn = true;  // флаг для проверки, включаем ли мы LED или выключаем

float prevTemp = 0.0;
float prevHum = 0.0;
unsigned long lastUpdateTime = 0;

const int TOTAL_DIGITAL_PINS = 28;
const int TOTAL_ANALOG_PINS = 16;
const int TOTAL_DIGITAL_OUTPUT_PINS = 5;
const int digitalOutputPins[TOTAL_DIGITAL_OUTPUT_PINS] = {3, 4, 5, 6, 7};

int digitalPins[TOTAL_DIGITAL_PINS];
int analogPins[TOTAL_ANALOG_PINS];

volatile byte diginBuff[TOTAL_DIGITAL_PINS];
volatile uint16_t adcBuff[TOTAL_ANALOG_PINS];
volatile int16_t digoutBuff[TOTAL_DIGITAL_OUTPUT_PINS];

int16_t adc0 = 0, adc1 = 0, adc2 = 0, adc3 = 0, adc4 = 0, adc5 = 0, adc6 = 0, adc7 = 0,
        adc8 = 0, adc9 = 0, adc10, adc11, adc12, adc13, adc14, adc15;

void initializeADCs() {
    adc0 = (adcBuff[0] > 0) ? adcBuff[0] : 0;
    adc1 = (adcBuff[1] > 0) ? adcBuff[1] : 0;
    adc2 = (adcBuff[2] > 0) ? adcBuff[2] : 0;
    adc3 = (adcBuff[3] > 0) ? adcBuff[3] : 0;
    adc4 = (adcBuff[4] > 0) ? adcBuff[4] : 0;
    adc5 = (adcBuff[5] > 0) ? adcBuff[5] : 0;
    adc6 = (adcBuff[6] > 0) ? adcBuff[6] : 0;
    adc7 = (adcBuff[7] > 0) ? adcBuff[7] : 0;
    adc8 = (adcBuff[8] > 0) ? adcBuff[8] : 0;
    adc9 = (adcBuff[9] > 0) ? adcBuff[9] : 0;
    adc10 = (adcBuff[10] > 0) ? adcBuff[10] : 0;
    adc11 = (adcBuff[11] > 0) ? adcBuff[11] : 0;
    adc12 = (adcBuff[12] > 0) ? adcBuff[12] : 0;
    adc13 = (adcBuff[13] > 0) ? adcBuff[13] : 0;
    adc14 = (adcBuff[14] > 0) ? adcBuff[14] : 0;
    adc15 = (adcBuff[15] > 0) ? adcBuff[15] : 0;

}

byte digin22, digin23, digin24, digin25, digin26, digin27, digin28, digin29, digin30,
        digin31, digin32, digin33, digin34, digin35, digin36, digin37, digin38, digin39, 
        digin40, digin41, digin42, digin43, digin44, digin45, digin46, digin47, digin48, digin49;

void initializeDigin() {
    digin22 = diginBuff[22];
    digin23 = diginBuff[23];
    digin24 = diginBuff[24];
    digin25 = diginBuff[25];
    digin26 = diginBuff[26];
    digin27 = diginBuff[27];
    digin28 = diginBuff[28];
    digin29 = diginBuff[29];
    digin30 = diginBuff[30];
    digin31 = diginBuff[31];
    digin32 = diginBuff[32];
    digin33 = diginBuff[33];
    digin34 = diginBuff[34];
    digin35 = diginBuff[35];
    digin36 = diginBuff[36];
    digin37 = diginBuff[37];
    digin38 = diginBuff[38];
    digin39 = diginBuff[39];
    digin40 = diginBuff[40];
    digin41 = diginBuff[41];
    digin42 = diginBuff[42];
    digin43 = diginBuff[43];
    digin44 = diginBuff[44];
    digin45 = diginBuff[45];
    digin46 = diginBuff[46];
    digin47 = diginBuff[47];
    digin48 = diginBuff[48];
    digin49 = diginBuff[49];
}

int16_t digout3, digout4, digout5, digout6, digout7;

void initializeDigitalOutputs() {
    digout3 = digoutBuff[3];
    digout4 = digoutBuff[4];
    digout5 = digoutBuff[5];
    digout6 = digoutBuff[6];
    digout7 = digoutBuff[7];
}

byte sht31Operational = 0;  // 0 означает, что датчик не работает; 1 означает, что работает.
byte sht31StateChanged = 0;  // 0 означает, что состояние не изменилось; 1 означает, что изменилось.
byte CAN0Operational = 0;  // 0 - CAN0 не работает; 1 - CAN0 работает

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
    if (diginBuff[i] != digitalPins[i]) {
      return true;
    }
  }
  return false;
}

bool checkAnalogPinsChanged() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    if (adcBuff[i] != analogPins[i]) {
      return true;
    }
  }
  return false;

}

void updatediginBuffAndSave() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    diginBuff[i] = digitalPins[i];
  }
  saveStateToEEPROM();
}

void updateadcBuffAndSave() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    adcBuff[i] = analogPins[i];
  }
  //saveAnalogStatesToEEPROM();
}

// Оптимизация сохранения состояний в EEPROM с использованием битовых операций
void saveStateToEEPROM() {
    byte buffByte = 0;
    for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
        buffByte |= digitalPins[i] << (i % 8);
        if ((i + 1) % 8 == 0 || i == TOTAL_DIGITAL_PINS) {
            EEPROM.write((i - 22) / 8, buffByte);
            buffByte = 0;
        }
    }
}

void saveAnalogStatesToEEPROM() {
    for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
        int value = analogPins[i];
        //EEPROM.write(28 + i * 2, value & 0xFF);
        //EEPROM.write(28 + i * 2 + 1, (value >> 8) & 0xFF);
    }
}

void initializeDigout() {
    for (int i = 0; i < TOTAL_DIGITAL_OUTPUT_PINS; i++) {
        digoutBuff[i] = digitalPins[digitalOutputPins[i]];
    }
}

void RealDashUpdateTask() {
  // Здесь вызывайте функцию или функции, которые обновляют данные для RealDash.
  updateRealDashData();
}

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#if DISP1_ACTIVE && defined DISP1_USE_ST7735_SPI
 Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#endif

void setup() {

//ПРОВЕРКА РАБОТЫ СВЕТОДИОДОВ  ЕГТ КОГДА АРДУИНО ВКЛЮЧАЕТСЯ ИЛИ ПЕРЕЗАГРУЖАЕТСЯ   
  for (int i = 0; i < 6; i++) {
    pinMode(EGT_LEDS[i], OUTPUT);
  }
  
  while(ledIndex < 6) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      if (turningOn) {
        digitalWrite(EGT_LEDS[ledIndex], HIGH);
        ledIndex++;
        if (ledIndex == 6) {
          turningOn = false;
          ledIndex--;
        }
      } else {
        digitalWrite(EGT_LEDS[ledIndex], LOW);
        ledIndex--;
        if (ledIndex < 0) {
          break;  // Завершаем цикл, когда все светодиоды выключены
        }
      }
    }
  }  

  const int RealDash = 115200;
  const int SPEEDUINO = 115200;
  Serial.begin(RealDash);
  Serial3.begin(SPEEDUINO);
 
  pinMode(CAN0_INT, INPUT);

  for (int i = A0; i < A0 + TOTAL_ANALOG_PINS; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    pinMode(i, INPUT);
  } 

for (int i = 3; i < TOTAL_DIGITAL_OUTPUT_PINS; i++) {
    pinMode(digitalOutputPins[i], OUTPUT);
}

  if (!initSHT31()) {
    // Обработка ошибки инициализации SHT31
  }

  // Инициализация для RealDash:
  initialiseRealDash();
  initialise_display();
}

void loop() {
  
  ts.execute();
  can.CAN0_INT_routine();
  can.sendAnalogData();
  processDigitalOutputs();

}

void ReadAnalogStatuses() {
  for (int i = 0; i < TOTAL_ANALOG_PINS; i++) {
    analogPins[i] = analogRead(i);
  }  
  if (checkAnalogPinsChanged()) {
    updateadcBuffAndSave();   
  }
initializeADCs(); // Здесь вызываем функцию инициализации переменных adcX  
}

void ReadDigitalStatuses() {
  for (int i = 22; i <= TOTAL_DIGITAL_PINS; i++) {
    digitalPins[i] = digitalRead(i);
  }
  if (checkDigitalPinsChanged()) {
    updatediginBuffAndSave();
  }
initializeDigin();  // инициализируем значения переменных diginXX из буфера
}

// Rest of your methods

void processDigitalOutputs() {
    for (int i = 0; i < TOTAL_DIGITAL_OUTPUT_PINS; i++) {
        if (digoutBuff[i] == HIGH) {
            digitalWrite(digitalOutputPins[i], HIGH);
        } else {
            digitalWrite(digitalOutputPins[i], LOW);
        }
    }
}

void readDigitalOutputs() {
    for (int i = 0; i < TOTAL_DIGITAL_OUTPUT_PINS; i++) {
        digoutBuff[i] = digitalRead(digitalOutputPins[i]);
    }
}

void SHT31Task() {
    float temp = readTemperature();
    float hum = readHumidity();

    // Используем битовые операции для проверки и изменения состояний датчика
    sht31Operational = (temp != -1 && hum != -1) ? 1 : 0;
    sht31StateChanged = (abs(temp - prevTemp) >= 0.1 || abs(hum - prevHum) >= 0.1);

    if (abs(temp - prevTemp) >= 0.1) {
        prevTemp = temp;
        //displayTemperature(temp);
    }

    if (abs(hum - prevHum) >= 0.1) {
        prevHum = hum;
        //displayHumidity(hum);
    }
}

void driveDisplayTask() {
  driveDisplay();
 }

Task t1(50, TASK_FOREVER, &ReadAnalogStatuses, &ts, true);
Task t2(50, TASK_FOREVER, &ReadDigitalStatuses, &ts, true);
Task t3(50, TASK_FOREVER, &readDigitalOutputs, &ts, true);
Task t4(50, TASK_FOREVER, &sendAnalogDataToCAN0, &ts, true); 
Task t5(50, TASK_FOREVER, &SHT31Task, &ts, true);
Task t6(50, TASK_FOREVER, &RealDashUpdateTask, &ts, true);
Task t7(50, TASK_FOREVER, &driveDisplayTask, &ts, true);
