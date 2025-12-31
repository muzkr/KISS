#include "main_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "driver/keypad.hpp"
#include "driver/lcd.hpp"
#include "driver/backlight.hpp"
#include "driver/serial.hpp"
#include "driver/adc.hpp"
#include "driver/flashlight.hpp"
#include "driver/spi_flash.hpp"

#include "blink_task.hpp"

static StaticTask_t main_task_obj;
static TaskHandle_t main_task;

void _putchar(char c)
{
    driver::serial::send((uint8_t *)&c, 1);
}

static void main_task_run(void *arg)
{
    driver::serial::init();
    driver::lcd::init();
    driver::keypad::init();
    driver::backlight::init(5);
    driver::adc::init();
    driver::flashlight::init();
    driver::spi_flash::init();

    blink_task_init();

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void main_task_init()
{
    main_task = xTaskCreateStatic(main_task_run, "main", NULL, 1, &main_task_obj);
}
