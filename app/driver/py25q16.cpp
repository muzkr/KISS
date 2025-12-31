
#include "driver/py25q16.hpp"

using namespace driver::py25q16;

static void write_cmd_addr(uint8_t cmd, uint32_t addr);

static driver::py25q16::spi_callbacks spi_cb = {0};

void driver::py25q16::set_spi_callbacks(spi_callbacks cb) { spi_cb = cb; }
void driver::py25q16::write_enable() { spi_cb.write_byte(CMD_WRITE_ENABLE); }
void driver::py25q16::write_disable() { spi_cb.write_byte(CMD_WRITE_DISABLE); }

uint8_t driver::py25q16::read_status_reg()
{
    spi_cb.write_byte(CMD_READ_STATUS_REG);
    return spi_cb.write_byte(0);
}

void driver::py25q16::sector_erase(uint32_t addr) { write_cmd_addr(CMD_SECTOR_ERASE, addr); }
void driver::py25q16::page_program(uint32_t addr) { write_cmd_addr(CMD_PAGE_PROGRAM, addr); }

void driver::py25q16::fast_read(uint32_t addr)
{
    write_cmd_addr(CMD_FAST_READ, addr);
    spi_cb.write_byte(0);
}

void driver::py25q16::read(uint32_t addr) { write_cmd_addr(CMD_READ, addr); }

static inline void write_cmd_addr(uint8_t cmd, uint32_t addr)
{
    spi_cb.write_byte(cmd);
    spi_cb.write_byte(0xff & (addr >> 16));
    spi_cb.write_byte(0xff & (addr >> 8));
    spi_cb.write_byte(0xff & addr);
}
