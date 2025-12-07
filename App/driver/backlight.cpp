#include "driver/backlight.hpp"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_system.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_tim.h"
#include <string.h>

// PF8
#define GPIOx GPIOF
#define BL_PIN LL_GPIO_PIN_8

#define PIN_SET BL_PIN
#define PIN_RESET (BL_PIN << 16)

#define DMAx DMA1
#define DMA_CHN LL_DMA_CHANNEL_3

#define TIMx TIM6

#define REFRESH_RATE 100

using namespace driver::backlight;

static uint32_t duty_cycle[BACKLIGHT_LEVELS];
static uint32_t curr_level;

static inline uint32_t calc_ARR(uint32_t f)
{
    return f / (REFRESH_RATE * BACKLIGHT_LEVELS) - 1;
}

static void config_DMA()
{
    LL_DMA_ConfigTransfer(DMAx, DMA_CHN,                    //
                          LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                              | LL_DMA_MODE_CIRCULAR        //
                              | LL_DMA_PERIPH_NOINCREMENT   //
                              | LL_DMA_MEMORY_INCREMENT     //
                              | LL_DMA_PDATAALIGN_WORD      //
                              | LL_DMA_MDATAALIGN_WORD      //
                              | LL_DMA_PRIORITY_MEDIUM      //
    );
    LL_DMA_SetMemoryAddress(DMAx, DMA_CHN, (uint32_t)duty_cycle);
    LL_DMA_SetPeriphAddress(DMAx, DMA_CHN, (uint32_t)&GPIOx->BSRR);
    LL_DMA_SetDataLength(DMAx, DMA_CHN, sizeof(duty_cycle) / sizeof(uint32_t));
}

static void set_level(uint32_t level)
{
    LL_DMA_DisableChannel(DMAx, DMA_CHN);
    LL_TIM_DisableCounter(TIMx);

    if (0 == level)
    {
        LL_GPIO_ResetOutputPin(GPIOx, BL_PIN);
        return;
    }
    if (BACKLIGHT_LEVELS - 1 == level)
    {
        LL_GPIO_SetOutputPin(GPIOx, BL_PIN);
        return;
    }

    for (uint32_t i = 0; i < level; i++)
    {
        duty_cycle[i] = PIN_SET;
    }
    for (uint32_t i = level; i < BACKLIGHT_LEVELS; i++)
    {
        duty_cycle[i] = PIN_RESET;
    }

    config_DMA();
    LL_DMA_EnableChannel(DMAx, DMA_CHN);
    LL_TIM_EnableCounter(TIMx);
}

void driver::backlight::init(uint32_t level)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_GPIO_ResetOutputPin(GPIOx, BL_PIN);

    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.Pin = BL_PIN;
        init_struct.Mode = LL_GPIO_MODE_OUTPUT;
        init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        init_struct.Pull = LL_GPIO_PULL_NO;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        LL_GPIO_Init(GPIOx, &init_struct);
    } while (false);

    LL_DMA_DisableChannel(DMAx, DMA_CHN);
    LL_SYSCFG_SetDMARemap(DMAx, DMA_CHN, LL_SYSCFG_DMA_MAP_TIM6_UP);

    LL_TIM_DisableCounter(TIMx);

    do
    {
        LL_TIM_InitTypeDef init_struct;
        init_struct.CounterMode = LL_TIM_COUNTERMODE_UP;
        init_struct.Prescaler = 999;
        init_struct.Autoreload = calc_ARR(SystemCoreClock / 1000);
        LL_TIM_Init(TIMx, &init_struct);
    } while (false);

    LL_TIM_EnableDMAReq_UPDATE(TIMx);
    LL_TIM_EnableUpdateEvent(TIMx);

    if (level >= BACKLIGHT_LEVELS)
    {
        level = BACKLIGHT_LEVELS - 1;
    }

    ::set_level(level);
    curr_level = level;
}

uint32_t driver::backlight::get_current_level()
{
    return curr_level;
}

void driver::backlight::set_level(uint32_t level)
{
    if (level >= BACKLIGHT_LEVELS)
    {
        level = BACKLIGHT_LEVELS - 1;
    }

    if (curr_level == level)
    {
        return;
    }

    ::set_level(level);
    curr_level = level;
}
