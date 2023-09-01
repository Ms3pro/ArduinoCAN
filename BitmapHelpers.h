// BitmapHelpers.h

#ifndef BitmapHelpers_h
#define BitmapHelpers_h

#include <Adafruit_ST7735.h>
#include <avr/pgmspace.h>

extern Adafruit_ST7735 tft; 

extern const uint16_t epd_bitmap_BMW_1[] PROGMEM;
extern const uint16_t epd_bitmap_BMW_2[] PROGMEM;
extern const uint16_t epd_bitmap_BMW_3[] PROGMEM;
extern const uint16_t epd_bitmap_BMW_4[] PROGMEM;

void drawRGBBitmapFromProgmem(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h);

#endif
