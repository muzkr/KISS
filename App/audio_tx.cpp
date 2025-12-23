#include "audio_tx.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "driver/bk4819.h"

void audio_tx_enter()
{
    // Mute audio first to prevent click
    BK4819_EnterTxMute();
    BK4819_SetAF(BK4819_AF_MUTE);
    AUDIO_AudioPathOff();

    // Configure transmitter
    BK4819_SetFrequency(40975000);
    BK4819_PrepareTransmit();

    BK4819_SetFilterBandwidth(BK4819_FILTER_BW_NARROW, false);

    BK4819_PickRXFilterPathBasedOnFrequency(40975000);

    // Add compander for microphone audio processing
    BK4819_SetCompander(1);
    BK4819_WriteRegister(BK4819_REG_29,
                         (1u << 14) |     // 2:1 compression ratio
                             (86u << 7) | // 0dB point
                             (95u << 0)); // Higher noise threshold

    // Enable power amplifier
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, true);
    BK4819_SetupPowerAmplifier(40, 40975000);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Add CTCSS tone encoding
    uint16_t ctcss_55hz_control = 1134;
    BK4819_SetCTCSSFrequency(ctcss_55hz_control);
    BK4819_WriteRegister(BK4819_REG_51, 0x904A);

    // Enable TX audio path
    BK4819_EnableTXLink();

    // CRITICAL: Re-enable microphone ADC after TX link setup
    BK4819_WriteRegister(BK4819_REG_30,
                         BK4819_REG_30_ENABLE_VCO_CALIB |
                             BK4819_REG_30_ENABLE_UNKNOWN |
                             BK4819_REG_30_DISABLE_RX_LINK |
                             BK4819_REG_30_ENABLE_AF_DAC |
                             BK4819_REG_30_ENABLE_DISC_MODE |
                             BK4819_REG_30_ENABLE_PLL_VCO |
                             BK4819_REG_30_ENABLE_PA_GAIN |
                             BK4819_REG_30_ENABLE_MIC_ADC | // Enable microphone ADC
                             BK4819_REG_30_ENABLE_TX_DSP |
                             BK4819_REG_30_DISABLE_RX_DSP);

    // Unmute for transmission
    BK4819_ExitTxMute();

    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
}

void audio_tx_exit()
{

    // Send CTCSS tail tone for squelch crash elimination
    BK4819_PlayCTCSSTail();

    vTaskDelay(pdMS_TO_TICKS(200));

    // Disable TX audio path
    BK4819_EnterTxMute();

    // Turn off power amplifier
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);

    // Disable transmitter
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_Sleep();

    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
}
