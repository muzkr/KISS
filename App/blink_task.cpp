#include "blink_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "driver/flashlight.hpp"

#include "driver/bk4819.h"

static void blink(void *arg);
static StaticTask_t blink_task;
static TaskHandle_t blink_task_handle;

void blink_task_init()
{
    blink_task_handle = xTaskCreateStatic(blink, "blink", NULL, 1, &blink_task);
}

#define ENABLE_TAIL_TONE 1

static void blink(void *arg)
{
    printf("BK4829 Standalone Test\n");

    // Single function call handles everything
    BK4819_Init();

    // Apply crystal calibration
    BK4819_WriteRegister(BK4819_REG_3B, 22656 + 2);

    // Verify communication
    uint16_t chip_id = BK4819_ReadRegister(BK4819_REG_00);
    printf("Chip ID: 0x%04X\n", chip_id);

    // Start RSSI monitoring
    BK4819_SetFrequency(14500000);

    // Configure squelch before enabling receiver
    BK4819_SetupSquelch(
        72, // RSSI open threshold (-124 dBm)
        70, // RSSI close threshold (-125 dBm)
        46, // Noise open threshold
        47, // Noise close threshold
        8,  // Glitch close threshold
        8   // Glitch open threshold
    );

    BK4819_RX_TurnOn();

    // Enable FM audio output
    // BK4819_SetAF(BK4819_AF_FM);

    // Enable squelch interrupts in mask register
    BK4819_WriteRegister(BK4819_REG_3F,
                         BK4819_REG_3F_CxCSS_TAIL |        // Add CTCSS tail detection
                             BK4819_REG_3F_SQUELCH_FOUND | //
                             BK4819_REG_3F_SQUELCH_LOST    //
    );

    // Set up CTCSS tail detection (55Hz tone)
    BK4819_SetCTCSSFrequency(550); // 55.0Hz tail tone
    BK4819_SetTailDetection(550);

    //  Enable audio path and amplifier
    AUDIO_AudioPathOn(); // Enable audio path to speaker

    // Configure audio gain (optional - uses default values)
    BK4819_WriteRegister(BK4819_REG_48,
                         (11u << 12) |    // ??? bits
                             (0u << 10) | // AF Rx Gain-1 (0dB)
                             (58u << 4) | // AF Rx Gain-2 (volume)
                             (8u << 0));  // AF DAC Gain

    printf("Audio path and speaker enabled\n");

    while (1)
    {
        // Check if interrupt is pending
        if (BK4819_ReadRegister(BK4819_REG_0C) & 1u)
        {
            // Clear interrupt
            BK4819_WriteRegister(BK4819_REG_02, 0);
            // Read interrupt status
            uint16_t interrupt_status = BK4819_ReadRegister(BK4819_REG_02);

            uint16_t raw_rssi = BK4819_GetRSSI();
            int16_t rssi_dbm = BK4819_GetRSSI_dBm();

            // Check squelch interrupts
            bool squelch_found = (interrupt_status & BK4819_REG_02_SQUELCH_FOUND);
            bool squelch_lost = (interrupt_status & BK4819_REG_02_SQUELCH_LOST);
            bool css_tail_found = (interrupt_status & BK4819_REG_02_CxCSS_TAIL);

            printf("RSSI: %u (%d dBm), INT: 0x%04X, SQL_FOUND: %d, SQL_LOST: %d\n",
                   raw_rssi, rssi_dbm, interrupt_status,
                   (int)squelch_found, (int)squelch_lost);

            // Control audio based on squelch
            if (css_tail_found)
            {
                // Tail tone detected - prepare to mute gracefully
                printf("CTCSS Tail detected - preparing mute\n");
                BK4819_SetAF(BK4819_AF_MUTE);
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false); // Turn OFF green LED
            }
#if !(ENABLE_TAIL_TONE)
            else if (squelch_found)
            {
                BK4819_SetAF(BK4819_AF_MUTE);                         // Signal lost, mute audio
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false); // Turn OFF green LED
                printf("-> SQUELCH CLOSED - Audio muted\n");
            }
#endif
            else if (squelch_lost)
            {
                BK4819_SetAF(BK4819_AF_FM);                          // Signal found, enable audio
                BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true); // Turn ON green LED
                printf("-> SQUELCH OPEN - Audio enabled\n");
            }
        }

        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}
