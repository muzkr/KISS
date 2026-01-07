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
        BL2_BOOT_MODE_USB_DISK = 0xb0,
        BL2_BOOT_MODE_USB_DISK_FMT,
    };

    bool bl2_get_boot_mode(uint8_t *boot_mode, uint32_t *param);
    void bl2_set_boot_mode(uint8_t boot_mode, uint32_t param);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _BL2_H
