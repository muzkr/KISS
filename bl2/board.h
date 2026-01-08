#ifndef _BOARD_H
#define _BOARD_H

#include <stdint.h>
#include <stdbool.h>

void systick_init();
void systick_disable();
uint32_t systick_get_ticks_ms();

void backlight_init();
void backlight_on(uint32_t dur);
void backlight_off();
void backlight_update();

#endif