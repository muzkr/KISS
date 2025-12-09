#ifndef _DRIVER_FLASHLIGHT_HPP
#define _DRIVER_FLASHLIGHT_HPP

namespace driver::flashlight
{
    void init();
    void turn_on(bool on = true);
    void toggle();
}

#endif // _DRIVER_FLASHLIGHT_HPP
