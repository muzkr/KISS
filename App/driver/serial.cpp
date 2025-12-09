
#include "driver/serial.hpp"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_usart.h"
#include "py32f071_ll_gpio.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#if KISS_ENABLE_SERIAL

#define USARTx USART1

#if KISS_ENABLE_SERIAL_TX

#if defined(KISS_SERIAL_TX_BUF_SIZE) && KISS_SERIAL_TX_BUF_SIZE > 0
#define TX_BUF_SIZE KISS_SERIAL_TX_BUF_SIZE
#else
#define TX_BUF_SIZE 0x100
#endif

static uint8_t tx_buf[TX_BUF_SIZE];
static StaticQueue_t tx_queue_obj;
static QueueHandle_t tx_queue;
static StaticSemaphore_t tx_mutex_obj;
static SemaphoreHandle_t tx_mutex;
static StaticTask_t tx_task_obj;
static TaskHandle_t tx_task;

static void tx_task_run(void *arg)
{
    while (true)
    {
        uint8_t b;
        if (!xQueueReceive(tx_queue, &b, portMAX_DELAY))
        {
            continue;
        }

        while (!LL_USART_IsActiveFlag_TXE(USARTx))
        {
            vPortYield();
        }

        LL_USART_TransmitData8(USARTx, b);
    }
}

#endif // KISS_ENABLE_SERIAL_TX

void driver::serial::init()
{
    // PA9, PA10

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    LL_USART_Disable(USARTx);

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
        init_struct.TransferDirection = LL_USART_DIRECTION_NONE
#if KISS_ENABLE_SERIAL_TX
                                        | LL_USART_DIRECTION_TX
#endif
#if KISS_ENABLE_SERIAL_RX
                                        | LL_USART_DIRECTION_RX
#endif
            ;

        LL_USART_Init(USARTx, &init_struct);
    } while (0);

    LL_USART_ConfigAsyncMode(USARTx);
    LL_USART_Enable(USARTx);

#if KISS_ENABLE_SERIAL_TX
    tx_queue = xQueueCreateStatic(sizeof(tx_buf), 1, tx_buf, &tx_queue_obj);
    tx_mutex = xSemaphoreCreateMutexStatic(&tx_mutex_obj);
    tx_task = xTaskCreateStatic(tx_task_run, "serial_tx", NULL, 1, &tx_task_obj);
#endif
}

#if KISS_ENABLE_SERIAL_TX

bool driver::serial::lock_tx(TickType_t block)
{
    return pdTRUE == xSemaphoreTake(tx_mutex, block);
}

void driver::serial::unlock_tx()
{
    xSemaphoreGive(tx_mutex);
}

uint32_t driver::serial::send(const uint8_t *buf, uint32_t size, TickType_t block)
{
    uint32_t size1 = 0;
    TickType_t t0 = xTaskGetTickCount();
    while (size > 0)
    {
        if (!xQueueSend(tx_queue, buf, block))
        {
            break;
        }

        buf++;
        size--;
        size1++;

        if (portMAX_DELAY != block)
        {
            TickType_t t1 = xTaskGetTickCount();
            TickType_t dt = t1 - t0;
            if (dt >= block)
            {
                break;
            }

            block -= dt;
            t0 = t1;
        }
    }

    return size1;
}

#endif // KISS_ENABLE_SERIAL_TX

#endif // KISS_ENABLE_SERIAL
