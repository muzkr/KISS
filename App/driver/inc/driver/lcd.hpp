#ifndef _DRIVER_LCD_HPP
#define _DRIVER_LCD_HPP

#include <stdint.h>
#include "FreeRTOS.h"

namespace driver::lcd
{
    enum
    {
        LCD_PAGES = 8,
        LCD_WIDTH = 128,
        LCD_HEIGHT = LCD_PAGES * 8,
    };

    void init(bool inverse = false);
    bool lock(TickType_t block = portMAX_DELAY);
    void unlock();
    void fill(uint8_t px = 0);
    void blit(uint32_t page, uint32_t col, const uint8_t *buf, uint32_t size);
    bool is_inverse();
    void set_inverse(bool inverse = true);
}

#endif // _DRIVER_LCD_HPP
