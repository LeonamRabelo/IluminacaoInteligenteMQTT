#ifndef MATRIZ_LEDS_H
#define MATRIZ_LEDS_H

#include <stdint.h>

#define NUM_NUMBERS 11
#define NUM_PIXELS 25   //Quantidade de LEDs na matriz

//Função pública
void set_one_led(uint8_t r, uint8_t g, uint8_t b, int numero);

#endif