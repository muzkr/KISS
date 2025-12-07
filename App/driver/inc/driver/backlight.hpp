#ifndef _DRIVER_BACKLIGHT_HPP
#define _DRIVER_BACKLIGHT_HPP

#include <stdint.h>

namespace driver::backlight
{
    enum
    {
        BACKLIGHT_LEVELS = 8,
    };

    void init(uint32_t level = 0);
    uint32_t get_current_level();
    void set_level(uint32_t level);
}

#endif // _DRIVER_BACKLIGHT_HPP
