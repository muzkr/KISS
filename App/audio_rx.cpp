#include "audio_rx.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"

#include "driver/bk4819.h"

#define ENABLE_TAIL_TONE 1

void audio_rx_enter()
{
    // Enable receiver
    BK4819_RX_TurnOn();
    BK4819_WriteRegister(BK4819_REG_3F,
                         BK4819_REG_3F_CxCSS_TAIL |
                             BK4819_REG_3F_SQUELCH_FOUND |
                             BK4819_REG_3F_SQUELCH_LOST);

    AUDIO_AudioPathOn();
    BK4819_WriteRegister(BK4819_REG_48,
                         (11u << 12) | (0u << 10) | (58u << 4) | (8u << 0));
}

void audio_rx_exit()
{
    // Disable receiver
    BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    AUDIO_AudioPathOff();

    // Turn off receiver
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_Sleep();
}

void audio_rx_process()
{
    if (BK4819_ReadRegister(BK4819_REG_0C) & 1u)
    {
        BK4819_WriteRegister(BK4819_REG_02, 0);
        uint16_t interrupt_status = BK4819_ReadRegister(BK4819_REG_02);

        uint16_t raw_rssi = BK4819_GetRSSI();
        int16_t rssi_dbm = BK4819_GetRSSI_dBm();

        bool squelch_found = (interrupt_status & BK4819_REG_02_SQUELCH_FOUND);
        bool squelch_lost = (interrupt_status & BK4819_REG_02_SQUELCH_LOST);
        bool css_tail_found = (interrupt_status & BK4819_REG_02_CxCSS_TAIL);

        printf("RSSI: %u (%d dBm), INT: 0x%04X, SQL_FOUND: %d, SQL_LOST: %d\n",
               raw_rssi, rssi_dbm, interrupt_status,
               (int)squelch_found, (int)squelch_lost);

        if (css_tail_found)
        {
            printf("CTCSS Tail detected - preparing mute\n");
            BK4819_SetAF(BK4819_AF_MUTE);
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        }
#if !(ENABLE_TAIL_TONE)
        else if (squelch_found)
        {
            BK4819_SetAF(BK4819_AF_MUTE);
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        }
#endif
        else if (squelch_lost)
        {
            BK4819_SetAF(BK4819_AF_FM);
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
            printf("-> SQUELCH OPEN - Audio enabled\n");
        }
    }
}
