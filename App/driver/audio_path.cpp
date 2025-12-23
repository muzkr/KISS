#include "driver/audio_path.hpp"

#include "py32f071_ll_gpio.h"
#include "py32f071_ll_bus.h"

// Audio PA: PA8
#define AUDIO_PA_GPIOx GPIOA
#define AUDIO_PA_PIN LL_GPIO_PIN_8

// using namespace driver::audio_path;

void driver::audio_path::init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    off();

    LL_GPIO_InitTypeDef init_struct;
    init_struct.Mode = LL_GPIO_MODE_OUTPUT;
    init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    // init_struct.Pull = LL_GPIO_PULL_UP;
    init_struct.Pull = LL_GPIO_PULL_NO;
    init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    // PA8
    init_struct.Pin = LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOA, &init_struct);
}

void driver::audio_path::on()
{
    LL_GPIO_SetOutputPin(AUDIO_PA_GPIOx, AUDIO_PA_PIN);
}

void driver::audio_path::off()
{
    LL_GPIO_ResetOutputPin(AUDIO_PA_GPIOx, AUDIO_PA_PIN);
}
