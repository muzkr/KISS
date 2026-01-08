

#include "board.h"
#include "main.h"

// #include "printf.h"

static volatile uint32_t ticks_ms = 0;

void systick_init()
{
    NVIC_DisableIRQ(SysTick_IRQn);
    SysTick_Config(SystemCoreClock / 1000); // 1 ms
    NVIC_SetPriority(SysTick_IRQn, 3);
    NVIC_EnableIRQ(SysTick_IRQn);
}

void systick_disable()
{
    NVIC_DisableIRQ(SysTick_IRQn);
    SysTick->CTRL = 0;
}

uint32_t systick_get_ticks_ms()
{
    return ticks_ms;
}

void SysTick_Handler()
{
    // printf(".");
    ticks_ms++;
}

// --------------------

// PF8
#define BL_GPIOx GPIOF
#define BL_PIN LL_GPIO_PIN_8

static volatile struct
{
    bool on;
    uint32_t dur;
    uint32_t on_time;
} bl_state = {0};

void backlight_init()
{
    // PF8
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

    LL_GPIO_ResetOutputPin(BL_GPIOx, BL_PIN);

    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.Pin = BL_PIN;
        init_struct.Mode = LL_GPIO_MODE_OUTPUT;
        init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        init_struct.Pull = LL_GPIO_PULL_NO;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        LL_GPIO_Init(BL_GPIOx, &init_struct);
    } while (0);
}

void backlight_on(uint32_t dur)
{
    // printf("backlight_on: %d\n", dur);

    __disable_irq();
    bl_state.on = true;
    bl_state.on_time = systick_get_ticks_ms();
    bl_state.dur = dur;
    LL_GPIO_SetOutputPin(BL_GPIOx, BL_PIN);
    __enable_irq();
}

void backlight_off()
{
    __disable_irq();
    bl_state.on = false;
    LL_GPIO_ResetOutputPin(BL_GPIOx, BL_PIN);
    __enable_irq();
}

void backlight_update()
{
    __disable_irq();
    if (bl_state.on && bl_state.dur > 0)
    {
        uint32_t dt = systick_get_ticks_ms() - bl_state.on_time;
        // printf("backlight_update: dt = %d\n", dt);

        if (dt >= bl_state.dur)
        {
            // printf("backlight_update: on = %d, dt = %d, dur = %d, turn off\n", bl_state.on_time, dt, bl_state.dur);
            bl_state.on = false;
            LL_GPIO_ResetOutputPin(BL_GPIOx, BL_PIN);
        }
    }
    __enable_irq();
}
