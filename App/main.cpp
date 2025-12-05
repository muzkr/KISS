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

static void APP_SystemClockConfig();
static void init_usart();
static void usart_tx(const uint8_t *buf, uint32_t size);
static void blink(void *arg);

static inline void usart_tx_str(const char *str)
{
    usart_tx((const uint8_t *)str, strlen(str));
}

static StaticTask_t blink_task;
static TaskHandle_t blink_task_handle;

void _putchar(char c)
{
    usart_tx((uint8_t *)&c, 1);
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

    init_usart();

    printf("hello!\n");

    driver::lcd::init();
    driver::keypad::init();

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
            }
        }

        // for (uint32_t px = 0; px < 0xf; px++)
        // {
        //     driver::lcd::fill(px);
        //     vTaskDelay(pdMS_TO_TICKS(5000));
        // }
    }
}

static void usart_tx(const uint8_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        while (!LL_USART_IsActiveFlag_TXE(USART1))
        {
        }
        LL_USART_TransmitData8(USART1, buf[i]);
    }
}

static void init_usart()
{
    // USART1
    // PA9, PA10

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    LL_USART_Disable(USART1);

    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
        init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
        init_struct.Alternate = LL_GPIO_AF1_USART1;
        init_struct.Pull = LL_GPIO_PULL_UP;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        LL_GPIO_Init(GPIOA, &init_struct);
    } while (0);

    do
    {
        LL_USART_InitTypeDef init_struct;
        LL_USART_StructInit(&init_struct);
        init_struct.BaudRate = 38400;
        init_struct.DataWidth = LL_USART_DATAWIDTH_8B;
        init_struct.Parity = LL_USART_PARITY_NONE;
        init_struct.StopBits = LL_USART_STOPBITS_1;
        init_struct.TransferDirection = LL_USART_DIRECTION_TX_RX;
        LL_USART_Init(USART1, &init_struct);
    } while (0);

    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_Enable(USART1);
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
