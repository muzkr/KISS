#ifndef _DRIVER_ADC_HPP
#define _DRIVER_ADC_HPP

#include <stdint.h>

namespace driver::adc
{
    void init();
    void calib();
    uint16_t get_voltage();
}

#endif // _DRIVER_ADC_HPP
