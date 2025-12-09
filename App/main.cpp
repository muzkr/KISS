/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by Puya under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include <string.h>
#include "driver/keypad.hpp"
#include "driver/lcd.hpp"
#include "driver/backlight.hpp"
#include "driver/serial.hpp"

static void APP_SystemClockConfig();
static void blink(void *arg);

static StaticTask_t blink_task;
static TaskHandle_t blink_task_handle;

void _putchar(char c)
{
    driver::serial::send((uint8_t *)&c, 1);
}

/**
 * @brief  Main program.
 * @retval int
 */
int main()
{
    /* Enable SYSCFG and PWR clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* Clock initialization, configure the system clock as HSI */
    APP_SystemClockConfig();

    driver::serial::init();
    driver::lcd::init();
    driver::keypad::init();
    driver::backlight::init(5);

    blink_task_handle = xTaskCreateStatic(blink, "blink", NULL, 1, &blink_task);

    vTaskStartScheduler();

    while (1)
    {
    }
}

static void blink(void *arg)
{
    // PC13

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

    printf("hello!\n");

    while (1)
    {
        // LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
        // vTaskDelay(pdMS_TO_TICKS(500));

        using namespace driver::keypad;

        key_event e;
        if (receive_event(&e, portMAX_DELAY))
        {
            printf("keypad event: %x %x %x\n", e, get_event_type(e), get_key_code(e));

            if (KEY_PRESSED == get_event_type(e))
            {
                driver::lcd::fill((uint8_t)get_key_code(e));

                if (KEY_UP == get_key_code(e))
                {
                    driver::backlight::set_level(1 + driver::backlight::get_current_level());
                }
                else if (KEY_DOWN == get_key_code(e) && driver::backlight::get_current_level() > 0)
                {
                    driver::backlight::set_level(driver::backlight::get_current_level() - 1);
                }
            }
        }

        // for (uint32_t px = 0; px < 0xf; px++)
        // {
        //     driver::lcd::fill(px);
        //     vTaskDelay(pdMS_TO_TICKS(5000));
        // }
    }
}

/**
 * @brief  System clock configuration function
 * @param  None
 * @retval None
 */
static void APP_SystemClockConfig()
{
    // LL_SetSystemCoreClock(48000000);
    SystemCoreClockUpdate();
    LL_Init1msTick(SystemCoreClock);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void APP_ErrorHandler(void)
{
    /* Infinite loop */
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* Infinite loop */
    while (1)
    {
    }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
