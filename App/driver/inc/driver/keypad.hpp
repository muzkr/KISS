
#ifndef _DRIVER_KEYPAD_HPP
#define _DRIVER_KEYPAD_HPP

#include <stdint.h>
#include "FreeRTOS.h"

namespace driver::keypad
{
    enum key_code
    {
        // Keypad
        KEY_0 = 0,
        KEY_1,
        KEY_2,
        KEY_3,
        KEY_4,
        KEY_5,
        KEY_6,
        KEY_7,
        KEY_8,
        KEY_9,
        KEY_MENU,
        KEY_UP,
        KEY_DOWN,
        KEY_EXIT,
        KEY_STAR,
        KEY_F,

        // Side keys
        KEY_PTT,
        KEY_SIDE1,
        KEY_SIDE2,

        // Alias
        KEY_LEFT = KEY_UP,
        KEY_RIGHT = KEY_DOWN,
    };

    enum event_type
    {
        KEY_PRESSED = 1,
        KEY_SHORT_PRESS = 1 << 1,
        KEY_LONG_PRESS = 1 << 2,
        KEY_LONG_PRESS_REPEAT = 1 << 3,
        KEY_RELEASED = 1 << 4,

        // Combinations
        EVENT_ANY = KEY_PRESSED | KEY_SHORT_PRESS | KEY_LONG_PRESS | KEY_LONG_PRESS_REPEAT | KEY_RELEASED,
    };

    using key_event = uint16_t;

    static inline key_code get_key_code(key_event e)
    {
        return (key_code)(e >> 8);
    }

    static inline event_type get_event_type(key_event e)
    {
        return (event_type)(0xff & e);
    }

    void init();
    BaseType_t receive_event(key_event *e, TickType_t wait);
}

#endif // _DRIVER_KEYPAD_HPP
