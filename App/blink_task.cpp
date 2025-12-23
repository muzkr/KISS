#include "blink_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "driver/flashlight.hpp"
#include "driver/keypad.hpp"

#include "driver/bk4819.h"
#include "audio_rx.hpp"
#include "audio_tx.hpp"

static void blink(void *arg);
static StaticTask_t blink_task;
static TaskHandle_t blink_task_handle;

void blink_task_init()
{
    blink_task_handle = xTaskCreateStatic(blink, "blink", NULL, 1, &blink_task);
}

static void radio_init()
{
    // Single function call handles everything
    BK4819_Init();

    // Apply crystal calibration
    BK4819_WriteRegister(BK4819_REG_3B, 22656 + 2);

    // Verify communication
    uint16_t chip_id = BK4819_ReadRegister(BK4819_REG_00);
    printf("Chip ID: 0x%04X\n", chip_id);

    // Add proper microphone AGC configuration
    BK4819_WriteRegister(BK4819_REG_19, 0b0001000001000001);

    // DC filter
    uint16_t reg7e = BK4819_ReadRegister(BK4819_REG_7E);
    BK4819_WriteRegister(BK4819_REG_7E, (reg7e & ~(0b111 << 3)) | (4u << 3)); // DC filter = 4

    // microphone high-pass filtering
    // reg7e = BK4819_ReadRegister(BK4819_REG_7E);
    // BK4819_WriteRegister(BK4819_REG_7E, reg7e | (1u << 6)); // Enable mic high-pass filter

    // Configure microphone during initialization
    BK4819_WriteRegister(BK4819_REG_7D, 0xE940 | 4); // Base value + sensitivity

    BK4819_SetFrequency(40975000);

    // Configure squelch
    BK4819_SetupSquelch(72, 70, 46, 47, 8, 8);

    BK4819_SetCTCSSFrequency(550);
    BK4819_SetTailDetection(550);
}

static void blink(void *arg)
{
    printf("BK4829 Standalone Test\n");

    // Initialize
    radio_init();

    bool is_receive_mode = true;

    audio_rx_enter();

    while (1)
    {
        driver::keypad::key_event e;
        if (driver::keypad::receive_event(&e, 0))
        {
            if (driver::keypad::is_key_pressed(e, driver::keypad::KEY_PTT))
            {
                // PTT is pressed
                printf("PTT pressed - switching to TX mode\n");
                audio_rx_exit();
                audio_tx_enter();
                is_receive_mode = false;
            }
            else if (driver::keypad::is_key_released(e, driver::keypad::KEY_PTT))
            {
                // PTT is released
                printf("PTT released - switching to RX mode\n");
                audio_tx_exit();
                audio_rx_enter();
                is_receive_mode = true;
            }
        }

        // Let others run
        vPortYield();

        // Only process receiver interrupts when in receive mode
        if (is_receive_mode)
        {
            audio_rx_process();
        }
    } // while
}
