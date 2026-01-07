#include "fw.h"
#include <stdint.h>
#include "py32f0xx.h"

static void fw_boot0(const uint32_t *vec) __attribute__((naked));

void fw_boot()
{
    const uint32_t *vec = (uint32_t *)FW_ADDR;
    fw_boot0(vec);
}

static void fw_boot0(const uint32_t *vec)
{
    __asm__ volatile(
        ".syntax unified                    \n"
        "                                   \n"
        "ldr r1, [r0]                       \n"
        "msr msp, r1                        \n"
        "ldr r0, [r0, #4]                   \n"
        "bx r0                              \n"
        "b .                                \n"
        // End
    );
}
