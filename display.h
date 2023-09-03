#ifndef DISPLAY_H
#define DISPLAY_H

#include "Arduino.h"
#include "BitmapHelpers.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//display module options .  
//setup display 1
#define DISP1_ACTIVE     1             // 0 == DISABLED , 1 == ENABLED
//#define DISP1_SPI_I2C    0             // 0 == SPI , 1 == I2C

//if I2C set these next three values
//#define DISP1_ADDRESS    0x78          //direct i2c == 0 , specified address == screen address
//#define DISP1_I2C_ROUTE  0             // direct i2c == 0 , via mux == mux address
//#define DISP1_ROUTE_OUT  0             // the output port on the tca9548 screen is connected to if mux in use

//only choose one of the following defines for display type selection  , comment out the unused ones
#define DISP1_USE_ST7735_SPI
//#define DISP1_USE_SSH1106_SPI

//set pins for screens in spi mode
//display 1
#define TFT_CS     9
#define TFT_RST    11  
#define TFT_DC     10
#define TFT_SCLK 13  
#define TFT_MOSI 12 



#define LTBLUE    0xB6DF
#define LTTEAL    0xBF5F
#define LTGREEN         0xBFF7
#define LTCYAN    0xC7FF
#define LTRED           0xFD34
#define LTMAGENTA       0xFD5F
#define LTYELLOW        0xFFF8
#define LTORANGE        0xFE73
#define LTPINK          0xFDDF
#define LTPURPLE  0xCCFF
#define LTGREY          0xE71C
#define BROWN            0XBC40 
#define BLUE            0x001F
#define TEAL    0x0438
#define GREEN           0x07E0
#define CYAN          0x07FF
#define RED           0xF800
#define MAGENTA       0xF81F
#define YELLOW        0xFFE0
#define ORANGE        0xFD20
#define PINK          0xF81F
#define PURPLE    0x801F
#define GREY        0xC618
#define WHITE         0xFFFF
#define BLACK         0x0000

#define DKBLUE        0x000D
#define DKTEAL    0x020C
#define DKGREEN       0x03E0
#define DKCYAN        0x03EF
#define DKRED         0x6000
#define DKMAGENTA       0x8008
#define DKYELLOW        0x8400
#define DKORANGE        0x8200
#define DKPINK          0x9009
#define DKPURPLE      0x4010
#define DKGREY        0x4A49
#define BRRED            0XFC07 
#define GRAY             0X8430 //ЧТО ТО ТИПО АСФАЛЬТ 
#define MAGENTA          0xF81F //ЗЕЛЕНЫЙ
//---------------------------------------------------------------------------------------------------------

//setup display 2
//#define DISP2_ACTIVE     0             // 0 == DISABLED , 1 == ENABLED
//#define DISP2_SPI_I2C    1             // 0 == SPI , 1 == I2C

//if I2C set these next three values
//#define DISP2_ADDRESS    0             //direct i2c == 0 , specified address == screen address
//#define DISP2_I2C_ROUTE  0             // direct i2c == 0 , via mux == mux address
//#define DISP2_ROUTE_OUT  1             // the output port on the tca9548 screen is connected to if mux in use

//only choose one of the following defines for display type selection  , comment out the unused ones
//#define DISP2_USE_SSD1106_SPI
//#define DISP2_USE_SSH1106_SPI

//set pins for screens in spi mode
//display 2
//#define DISPLAY2_CS      7             //pin to use for CS
//#define DISPLAY2_DC      6             //pin to use for DC
//#define DISPLAY2_RESET   8

//-----------------------------------------------------------------------------------------------------

//set display size
//#define USE_128x64
//#define USE_128x128

//SPI NOTES:
//spi on mega2560 mosi == 51, clk(sck) == 52
//I2C NOTES:
// i2c on the mega2560 sda == 20, scl == 21
extern Adafruit_ST7735 tft;
void driveDisplay();
void initialise_display();
void tcaselect(uint8_t i, uint8_t dispnum);

void readEGTValues();
double rawEgt1,rawEgt2,rawEgt3,rawEgt4,rawEgt5,rawEgt6;
double egt1, egt2, egt3, egt4, egt5, egt6;
bool barBlinking[6] = {false, false, false, false, false, false};
unsigned long lastBlinkTime[6] = {0, 0, 0, 0, 0, 0};

extern int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
extern byte digin22, digin23, digin24, digin25, digin26, digin27, digin28, digin29, digin30, digin31, digin32, digin33, digin34, digin35, digin36, digin37, digin38, 
digin39, digin40, digin41, digin42, digin43, digin44, digin45, digin46, digin47, digin48, digin49;
extern int16_t digout3, digout4, digout5, digout6, digout7;
extern float temp;
extern float hum;
extern int seconds, pw1, pw2, rpm;
extern float temp;
extern float hum;


boolean graph_1 = true;
boolean graph_2 = true;
boolean graph_3 = true;
boolean graph_4 = true;
boolean graph_5 = true;
boolean graph_6 = true;
boolean graph_7 = true;
#endif
