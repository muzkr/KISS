#ifndef _FW_H
#define _FW_H

#include <stdbool.h>

#define FW_ADDR (0x08002800 + 0x400 * 16)   // bl2 = 16 KB

void fw_boot();

#endif // _FW_H
