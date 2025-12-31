#ifndef _DRIVER_SPI_FLASH_HPP
#define _DRIVER_SPI_FLASH_HPP

#include <stdint.h>
#include "driver/py25q16.hpp"

namespace driver::spi_flash
{
    using driver::py25q16::CHIP_SIZE;
    using driver::py25q16::SECTOR_SIZE;

    void init();
    void read(uint32_t addr, uint8_t *buf, uint32_t size);
    void write(uint32_t addr, const uint8_t *buf, uint32_t size, bool append);
    void erase_sector(uint32_t addr);
}

#endif // _DRIVER_SPI_FLASH_HPP
