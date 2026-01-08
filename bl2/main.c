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
#include "board.h"
#include "printf.h"
#include "bl2.h"
#include "fw.h"
#include "py25q16.h"
#include "usb_config.h"

#define ENABLE_PRINTF 1

#if ENABLE_PRINTF

#define USARTx USART1

static void init_usart();
void _putchar(char c)
{
    while (!LL_USART_IsActiveFlag_TXE(USARTx))
    {
    }
    LL_USART_TransmitData8(USARTx, (uint8_t)c);
}

#endif // ENABLE_PRINTF

static void APP_SystemClockConfig();

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

#if ENABLE_PRINTF
    init_usart();
#endif
    printf("hello!\n");

    uint8_t boot_mode;
    uint32_t boot_param;
    if (!bl2_get_boot_mode(&boot_mode, &boot_param))
    {
        boot_mode = _BL2_BOOT_MODE_INVALID;
    }
    printf("boot mode = %x, param = %x (%d)\n", boot_mode, boot_param, boot_param);

    // Test
    boot_mode = BL2_BOOT_USB_DISK;
    boot_param = 512;

    if (BL2_BOOT_USB_DISK == boot_mode || BL2_BOOT_USB_DISK_FMT == boot_mode)
    {
        systick_init();
        backlight_init();
        py25q16_init();

        msc_init(BL2_BOOT_USB_DISK_FMT == boot_mode, 0xffff & boot_param);

        while (1)
        {
            backlight_update();
            if (msc_update_alive())
            {
                // TODO:
            }
        }
    }
    else if (BL2_BOOT_DFU == boot_mode)
    {
        backlight_init();
        py25q16_init();
        // TODO
    }
    else
    {
        fw_boot();
        while (1)
        {
        }
    }
}

#if ENABLE_PRINTF
static void init_usart()
{
    // USART1
    // TX: PA9
    // RX: PA10

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    LL_USART_Disable(USARTx);

    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
        init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
        init_struct.Alternate = LL_GPIO_AF1_USART1;
        init_struct.Pull = LL_GPIO_PULL_UP;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        LL_GPIO_Init(GPIOA, &init_struct);
    } while (0);

    do
    {
        LL_USART_InitTypeDef init_struct;
        LL_USART_StructInit(&init_struct);
        init_struct.BaudRate = 38400;
        init_struct.TransferDirection = LL_USART_DIRECTION_TX;
        LL_USART_Init(USARTx, &init_struct);
    } while (0);

    LL_USART_Enable(USARTx);
}
#endif // ENABLE_PRINTF

/**
 * @brief  System clock configuration function
 * @param  None
 * @retval None
 */
static void APP_SystemClockConfig()
{
    // LL_SetSystemCoreClock(48000000);
    SystemCoreClockUpdate();
    // LL_Init1msTick(SystemCoreClock);
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
