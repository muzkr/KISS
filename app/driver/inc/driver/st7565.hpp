#ifndef _DRIVER_ST7565_HPP
#define _DRIVER_ST7565_HPP

#include <stdint.h>

namespace driver::st7565
{
    constexpr uint8_t cmd_display_on(bool on)
    {
        return 0b1010'1110 | (on ? 1 : 0);
    }

    constexpr uint8_t cmd_set_start_line(uint8_t n)
    {
        return 0b0100'0000 | (0b11'1111 & n);
    }

    constexpr uint8_t cmd_set_page_addr(uint8_t n)
    {
        return 0b1011'0000 | (0b1111 & n);
    }

    constexpr uint8_t cmd_set_col_addr_MSB(uint8_t n)
    {
        return 0b0001'0000 | (0b1111 & (n >> 4));
    }

    constexpr uint8_t cmd_set_col_addr_LSB(uint8_t n)
    {
        return 0b0000'0000 | (0b1111 & n);
    }

    constexpr uint8_t cmd_inverse(bool inverse)
    {
        return 0b1010'0110 | (inverse ? 1 : 0);
    }

    constexpr uint8_t cmd_all_pixel_on(bool on)
    {
        return 0b1010'0100 | (on ? 1 : 0);
    }

    constexpr uint8_t cmd_reset() { return 0b1110'0010; }

    constexpr uint8_t cmd_regulation_ratio(uint8_t n)
    {
        return 0b0010'0000 | (0b111 & n);
    }

    constexpr uint8_t cmd_power_control(uint8_t n)
    {
        return 0b0010'1000 | (0b111 & n);
    }

    constexpr uint8_t cmd_MY(bool MY)
    {
        return 0b1100'0000 | ((MY ? 1 : 0) << 3);
    }

    constexpr uint8_t cmd_MX(bool MX)
    {
        return 0b1010'0000 | (MX ? 1 : 0);
    }

    constexpr uint8_t cmd_set_EV() { return 0b1000'0001; }

    constexpr uint8_t cmd_bias_select(uint8_t n)
    {
        return 0b1010'0010 | (1 & n);
    }

    constexpr uint8_t cmd_nop() { return 0b1110'0011; }

} // namespace

#endif // _DRIVER_ST7565_HPP
