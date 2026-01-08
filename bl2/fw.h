#ifndef _FW_H
#define _FW_H

#include <stdbool.h>
#include "bl2.h"

#define FW_ADDR (0x08002800 + BL2_SIZE)

void fw_boot();

#endif // _FW_H
