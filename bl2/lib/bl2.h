#ifndef _BL2_H
#define _BL2_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

    enum
    {
        BL2_SIZE = 0x400 * 16, // BL2 size, 16 KB
    };

    // Boot modes
    enum
    {
        BL2_BOOT_USB_DISK = 0xdd,
        BL2_BOOT_USB_DISK_FMT = 0xdf,
        BL2_BOOT_DFU = 0xaa,
        _BL2_BOOT_MODE_INVALID = 0xff,
    };

    bool bl2_get_boot_mode(uint8_t *boot_mode, uint32_t *param);
    void bl2_set_boot_mode(uint8_t boot_mode, uint32_t param);

    static inline void bl2_set_boot_usb_disk(bool fmt, uint16_t skip_sectors)
    {
        bl2_set_boot_mode(fmt ? BL2_BOOT_USB_DISK_FMT : BL2_BOOT_USB_DISK, skip_sectors);
    }

    static inline void bl2_set_boot_dfu(uint16_t skip_sectors, uint16_t first_sector)
    {
        bl2_set_boot_mode(BL2_BOOT_DFU, (((uint32_t)first_sector) << 16) | skip_sectors);
    }

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _BL2_H
