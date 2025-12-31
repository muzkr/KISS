#ifndef _DRIVER_PY25Q16_HPP
#define _DRIVER_PY25Q16_HPP

#include <stdint.h>

namespace driver::py25q16
{
    // Constants
    enum
    {
        PAGE_SIZE = 256,
        SECTOR_SIZE = 0x1000, // 4 KB
        CHIP_SIZE = 0x200000, // 2 MB

        // Commands
        CMD_READ = 0x03,
        CMD_FAST_READ = 0x0b,
        CMD_SECTOR_ERASE = 0x20,
        CMD_PAGE_PROGRAM = 0x02,
        CMD_WRITE_ENABLE = 0x06,
        CMD_WRITE_DISABLE = 0x04,
        CMD_READ_STATUS_REG = 0x05,   // Read status register bits 7:0
        CMD_READ_STATUS_REG_1 = 0x35, // Read status register bits 15:8
        CMD_READ_CONFIG_REG = 0x15,
        CMD_RESET_ENABLE = 0x66,
        CMD_RESET = 0x99,
        CMD_DEEP_POWER_DOWN = 0xb9,
        CMD_RELEASE_DEEP_POWER_DOWN = 0xab,
        CMD_READ_UID = 0x4b,      // Read 128-bit unique ID
        CMD_READ_ID = 0x9f,       // Read 1-byte manufacturer ID and 2-byte device ID
        CMD_READ_JEDEC_ID = 0x90, // Read JEDEC assigned manufacturer ID (1 byte) and device ID (1 byte)

        //
        STATUS_REG_BIT_WIP = 0,
        STATUS_REG_MASK_WIP = 1 << STATUS_REG_BIT_WIP,
    };

    using spi_write_byte_func = uint8_t (*)(uint8_t n);
    using spi_write_data_func = void (*)(const uint8_t *buf, uint32_t size);
    using spi_read_data_func = void (*)(uint8_t *buf, uint32_t size);

    struct spi_callbacks
    {
        spi_write_byte_func write_byte;
        // spi_write_data_func write_data;
        // spi_read_data_func read_data;
    };

    void set_spi_callbacks(spi_callbacks cb);
    void write_enable();
    void write_disable();
    uint8_t read_status_reg();
    void sector_erase(uint32_t addr);
    void page_program(uint32_t addr);
    void fast_read(uint32_t addr);
    void read(uint32_t addr);

    static inline bool check_WIP() { return 0 != (STATUS_REG_MASK_WIP & read_status_reg()); }
}

#endif // _DRIVER_PY25Q16_HPP
