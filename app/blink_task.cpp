#include "blink_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "driver/flashlight.hpp"
#include "driver/keypad.hpp"
#include "driver/spi_flash.hpp"

static void blink(void *arg);
static StaticTask_t blink_task;
static TaskHandle_t blink_task_handle;

void blink_task_init()
{
    blink_task_handle = xTaskCreateStatic(blink, "blink", NULL, 1, &blink_task);
}

namespace spi_flash = driver::spi_flash;

static void test_erase()
{
    printf("erase sector 0x2000\n");
    spi_flash::erase_sector(0x2000);
}

static void test_read()
{
    printf("read data at 0x2000\n");

    uint8_t buf[256];
    spi_flash::read(0x2000, buf, sizeof(buf));

    for (uint32_t i = 0; i < sizeof(buf); i++)
    {
        if (i > 0 && 0 == i % 16)
        {
            printf("\n");
        }
        printf(" %02x", buf[i]);
    }

    printf("\n");
}

static void test_write1()
{
    printf("write at offset 0x10\n");

    const char *str = "0123456789";
    spi_flash::write(0x2010, (const uint8_t *)str, 10, false);
}

static void test_write2()
{
    printf("write at offset 236 (256 - 20)\n");

    const char *str = "abcdefghijklmnopqrst";
    spi_flash::write(0x2000 + 236, (const uint8_t *)str, 20, true);
}

static void blink(void *arg)
{
    printf("hello\n");

    test_read();
    test_erase();
    test_read();
    test_write1();
    test_read();
    test_write2();
    test_read();

    while (true)
    {
        taskYIELD();
    }
}
