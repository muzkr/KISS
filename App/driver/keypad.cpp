
#include "driver/keypad.hpp"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// #include "printf.h"

#define GPIOx GPIOB
#define PTT_PIN LL_GPIO_PIN_10
#define ROW_PINS (LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12)
#define COL_PINS (LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3)

#define ROW_PIN(n) ((uint32_t)(1u << (15 - (n))))
#define COL_PIN(n) ((uint32_t)(1u << (6 - (n))))

#define ROW_PTT -1
#define COL_SIDE -1

#define DEBOUNCE_IN_TIMEOUT 30
#define DEBOUNCE_OUT_TIMEOUT 30
#define LONG_PRESS_TIMEOUT 400
#define LONG_PRESS_REPEAT_TIMEOUT 200
// #define POLL_DELAY 10

#define EVENT_QUEUE_LEN 8

using namespace driver::keypad;

static const uint8_t KEY_CODES[5][4] = {
    {KEY_SIDE1, KEY_SIDE2},            //
    {KEY_MENU, KEY_1, KEY_4, KEY_7},   //
    {KEY_UP, KEY_2, KEY_5, KEY_8},     //
    {KEY_DOWN, KEY_3, KEY_6, KEY_9},   //
    {KEY_EXIT, KEY_STAR, KEY_0, KEY_F} //
};

static TaskHandle_t keypad_task;
static StaticTask_t keypad_task_obj;
static QueueHandle_t event_queue;
static StaticQueue_t event_queue_obj;
static uint8_t event_queue_buf[EVENT_QUEUE_LEN];

static inline key_code get_key(uint32_t row, int32_t col)
{
    return (key_code)KEY_CODES[col - COL_SIDE][row];
}

static inline void set_cols()
{
    LL_GPIO_SetOutputPin(GPIOx, COL_PINS);
}

static inline void reset_cols()
{
    LL_GPIO_ResetOutputPin(GPIOx, COL_PINS);
}

static inline void reset_col(uint32_t col)
{
    LL_GPIO_ResetOutputPin(GPIOx, COL_PIN(col));
}

static inline bool is_key_pressed(uint32_t pin)
{
    return !LL_GPIO_IsInputPinSet(GPIOx, pin);
}

static void raise_event(event_type type, key_code key)
{
    key_event e = make_event(type, key);

    // printf("keypad: %x %x %x\n", e, get_event_type(e), get_key_code(e));

    xQueueSend(event_queue, &e, 0);
}

static bool scan_rows(int32_t *row)
{
    // Check them all: return fast
    if (!is_key_pressed(PTT_PIN | ROW_PINS))
    {
        return false;
    }

    // PTT
    if (is_key_pressed(PTT_PIN))
    {
        *row = ROW_PTT;
        return true;
    }

    // Keypad
    for (uint32_t i = 0; i < 4; i++)
    {
        if (is_key_pressed(ROW_PIN(i)))
        {
            *row = i;
            return true;
        }
    }

    return false;
}

static inline void debounce_in()
{
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_IN_TIMEOUT));
}

static bool scan_cols(uint32_t row, int32_t *col)
{
    const uint32_t row_pin = ROW_PIN(row);

    for (int32_t i = COL_SIDE; i < 4; i++)
    {
        set_cols();
        if (i >= 0)
        {
            reset_col(i);
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        if (is_key_pressed(row_pin))
        {
            *col = i;
            return true;
        }
    }

    return false;
}

static inline bool check_released(int32_t row, int32_t col)
{
    return ROW_PTT == row ? !is_key_pressed(PTT_PIN)
                          : !is_key_pressed(ROW_PIN((uint32_t)row));
}

static void debounce_out()
{
    reset_cols();
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_OUT_TIMEOUT));
}

static void task_run(void *arg)
{
    reset_cols();
    vTaskDelay(pdMS_TO_TICKS(1));

    while (true)
    {
        int32_t row;
        int32_t col;
        key_code key;

        if (!scan_rows(&row))
        {
            vPortYield();
            continue;
        }

        debounce_in();

        if (ROW_PTT == row)
        {
            key = KEY_PTT;
        }
        else if (scan_cols((uint32_t)row, &col))
        {
            key = get_key((uint32_t)row, col);
            // printf("scan_cols: %d, %d, %d\n", row, col, key);
        }
        else
        {
            debounce_out();
            continue;
        }

        raise_event(KEY_PRESSED, key);
        vPortYield();

        bool long_press = false;
        for (const TickType_t t0 = xTaskGetTickCount();;)
        {
            if (check_released(row, col))
            {
                break;
            }

            TickType_t dt = xTaskGetTickCount() - t0;
            if (dt >= pdMS_TO_TICKS(LONG_PRESS_TIMEOUT))
            {
                long_press = true;
                break;
            }

            // vTaskDelay(pdMS_TO_TICKS(POLL_DELAY));
            vPortYield();
        }

        if (!long_press)
        {
            raise_event(KEY_SHORT_PRESS, key);
            vPortYield();
            debounce_out();
            continue;
        }

        // Long press ----------

        raise_event(KEY_LONG_PRESS, key);
        vPortYield();

        for (TickType_t t0 = xTaskGetTickCount();;)
        {
            if (check_released(row, col))
            {
                break;
            }

            TickType_t dt = xTaskGetTickCount() - t0;
            if (dt >= pdMS_TO_TICKS(LONG_PRESS_REPEAT_TIMEOUT))
            {
                t0 += pdMS_TO_TICKS(LONG_PRESS_REPEAT_TIMEOUT);
                raise_event(KEY_LONG_PRESS_REPEAT, key);
                // vPortYield();
            }

            // vTaskDelay(pdMS_TO_TICKS(POLL_DELAY));
            vPortYield();
        }

        raise_event(KEY_RELEASED, key);
        debounce_out();

    } // while(true)
}

void driver::keypad::init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    // LL_GPIO_SetOutputPin(GPIOx, COL_PINS);
    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

        // Input ----

        init_struct.Pin = PTT_PIN | ROW_PINS;
        init_struct.Mode = LL_GPIO_MODE_INPUT;
        init_struct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(GPIOx, &init_struct);

        // Output ----

        init_struct.Pin = COL_PINS;
        init_struct.Mode = LL_GPIO_MODE_OUTPUT;
        init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        LL_GPIO_Init(GPIOx, &init_struct);
    } while (false);

    // Kernel objects
    event_queue = xQueueCreateStatic(EVENT_QUEUE_LEN, 1, event_queue_buf, &event_queue_obj);
    keypad_task = xTaskCreateStatic(task_run, "keypad", NULL, 1, &keypad_task_obj);
}

BaseType_t driver::keypad::receive_event(key_event *e, TickType_t wait)
{
    return xQueueReceive(event_queue, e, wait);
}
