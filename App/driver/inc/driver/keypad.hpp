
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

        //
        _KEY_MAX,

        // Alias
        KEY_LEFT = KEY_UP,
        KEY_RIGHT = KEY_DOWN,

    };

    enum event_type
    {
        KEY_PRESSED,
        KEY_SHORT_PRESS,
        KEY_LONG_PRESS,
        KEY_LONG_PRESS_REPEAT,
        KEY_RELEASED,
        _EVENT_TYPE_MAX,
    };

    static_assert(_KEY_MAX <= 0b10'0000);
    static_assert(_EVENT_TYPE_MAX <= 0b1000);

    using key_event = uint8_t;

    static inline key_event make_event(event_type type, key_code key)
    {
        return (key_event)((key << 3) | (0b111 & type));
    }

    static inline key_code get_key_code(key_event e)
    {
        return (key_code)(e >> 3);
    }

    static inline event_type get_event_type(key_event e)
    {
        return (event_type)(0b111 & e);
    }

    void init();
    BaseType_t receive_event(key_event *e, TickType_t wait);
}

#endif // _DRIVER_KEYPAD_HPP
