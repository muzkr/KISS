#ifndef _DRIVER_LCD_HPP
#define _DRIVER_LCD_HPP

#include <stdint.h>

namespace driver::lcd
{
    enum
    {
        LCD_WIDTH = 128,
        LCD_HEIGHT = 64,
        LCD_PAGES = 8,
    };

    void init(bool inverse = false);
    void fill(uint8_t px = 0);
}

#endif // _DRIVER_LCD_HPP
