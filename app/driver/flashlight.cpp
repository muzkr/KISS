
#include "driver/flashlight.hpp"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_bus.h"

// PC13
#define GPIOx GPIOC
#define PINx LL_GPIO_PIN_13

void driver::flashlight::init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

    LL_GPIO_ResetOutputPin(GPIOx, PINx);

    LL_GPIO_InitTypeDef init_struct;
    init_struct.Pin = PINx;
    init_struct.Mode = LL_GPIO_MODE_OUTPUT;
    init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    init_struct.Pull = LL_GPIO_PULL_NO;
    init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    LL_GPIO_Init(GPIOx, &init_struct);
}

void driver::flashlight::turn_on(bool on)
{
    if (on)
    {
        LL_GPIO_SetOutputPin(GPIOx, PINx);
    }
    else
    {
        LL_GPIO_ResetOutputPin(GPIOx, PINx);
    }
}

void driver::flashlight::toggle()
{
    LL_GPIO_TogglePin(GPIOx, PINx);
}
