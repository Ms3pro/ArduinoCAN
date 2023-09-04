#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// Подключаем файл с определениями пинов
#include "pin_definitions.h"

void configurePins() {
    // Настройка пинов A0-A9 как входы
    DDRC &= ~0b00111111;

    // Настройка пинов A10-A15 как выходы
    DDRA |= 0b111111 << 2;

    // Настройка пинов 22-49 как входы
    DDRB &= ~0b11111111;
    DDRC &= ~0b11000000;
    DDRD &= ~0b11111111;

    // Настройка пинов 3-7 как выходы
    DDRE |= 0b11111;
}

#endif

