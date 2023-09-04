#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

// Определение масок для порта A, B, C и L
#define PORT_A_MASK 0b00111111
#define PORT_B_MASK 0b11111111
#define PORT_C_MASK 0b11111111
#define PORT_L_MASK 0b11111000

// Функция для настройки пинов
void configurePins();

#endif
