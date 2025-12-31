
#include "driver/lcd.hpp"
#include "driver/st7565.hpp"

#include "py32f071_ll_spi.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_utils.h"
#include "py32f071_ll_system.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "printf.h"

// PB2
#define CS_PORT GPIOB
#define CS_PIN LL_GPIO_PIN_2

// PA6
#define A0_PORT GPIOA
#define A0_PIN LL_GPIO_PIN_6

#define SPIx SPI1
#define DMA_CHANNEL_TX LL_DMA_CHANNEL_5
#define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
#define DMA_PRIORITY LL_DMA_PRIORITY_HIGH
#define DMA_CHANNEL_IRQn DMA1_Channel1_IRQn
#define DMA_CHANNEL_IRQ_PRIORITY 3
#define DMA_CHANNEL_IRQHandler DMA1_Channel1_IRQHandler
#define DMA_ENABLE_IT_TC() LL_DMA_EnableIT_TC(DMA1, DMA_CHANNEL_RX)
#define DMA_DISABLE_IT_TC() LL_DMA_DisableIT_TC(DMA1, DMA_CHANNEL_RX)
#define DMA_CHECK_TC() LL_DMA_IsActiveFlag_TC1(DMA1)
#define DMA_CLEAR_TC() LL_DMA_ClearFlag_TC1(DMA1)
#define DMA_CLEAR_GI() LL_DMA_ClearFlag_GI1(DMA1)

#define COL_ADDR_OFFSET 4
#define DMA_TX_MIN 32

static uint8_t dummy;
static bool inverse;
static StaticSemaphore_t TC_semphr_obj;
static SemaphoreHandle_t TC_semphr;
static StaticSemaphore_t lcd_mutex_obj;
static SemaphoreHandle_t lcd_mutex;

static inline void CS_assert()
{
    LL_GPIO_ResetOutputPin(CS_PORT, CS_PIN);
}

static inline void CS_release()
{
    LL_GPIO_SetOutputPin(CS_PORT, CS_PIN);
}

static inline void A0_cmd_mode()
{
    LL_GPIO_ResetOutputPin(A0_PORT, A0_PIN);
}

static inline void A0_data_mode()
{
    LL_GPIO_SetOutputPin(A0_PORT, A0_PIN);
}

static void SPI_init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    CS_release();

    do
    {
        LL_GPIO_InitTypeDef init_struct;
        init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        init_struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

        // Output ---

        init_struct.Mode = LL_GPIO_MODE_OUTPUT;
        init_struct.Pull = LL_GPIO_PULL_UP;

        // CS: PB2
        init_struct.Pin = CS_PIN;
        LL_GPIO_Init(CS_PORT, &init_struct);

        // A0: PA6
        init_struct.Pin = A0_PIN;
        LL_GPIO_Init(A0_PORT, &init_struct);

        // SPI ----

        init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
        init_struct.Alternate = LL_GPIO_AF0_SPI1;

        // SCK: PA5
        init_struct.Pin = LL_GPIO_PIN_5;
        init_struct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(GPIOA, &init_struct);

        // MOSI: PA7
        init_struct.Pin = LL_GPIO_PIN_7;
        init_struct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOA, &init_struct);

    } while (false);

    NVIC_DisableIRQ(DMA_CHANNEL_IRQn);
    NVIC_SetPriority(DMA_CHANNEL_IRQn, DMA_CHANNEL_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA_CHANNEL_IRQn);

    LL_SYSCFG_SetDMARemap(DMA1, DMA_CHANNEL_RX, LL_SYSCFG_DMA_MAP_SPI1_RD);
    LL_SYSCFG_SetDMARemap(DMA1, DMA_CHANNEL_TX, LL_SYSCFG_DMA_MAP_SPI1_WR);

    LL_SPI_Disable(SPIx);

    do
    {
        LL_SPI_InitTypeDef init_struct;
        init_struct.Mode = LL_SPI_MODE_MASTER;
        init_struct.TransferDirection = LL_SPI_FULL_DUPLEX;
        init_struct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
        init_struct.BitOrder = LL_SPI_MSB_FIRST;
        init_struct.ClockPolarity = LL_SPI_POLARITY_HIGH;
        init_struct.ClockPhase = LL_SPI_PHASE_2EDGE;
        init_struct.NSS = LL_SPI_NSS_SOFT;
        init_struct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
        init_struct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
        LL_SPI_Init(SPIx, &init_struct);
    } while (false);

    LL_SPI_Enable(SPIx);
}

static void SPI_write_byte(uint8_t n)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPIx))
    {
    }
    LL_SPI_TransmitData8(SPIx, n);

    while (!LL_SPI_IsActiveFlag_RXNE(SPIx))
    {
    }

    LL_SPI_ReceiveData8(SPIx);
}

static void SPI_DMA_write_buf(const uint8_t *buf, uint32_t size)
{
    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_RX);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_TX);

    DMA_CLEAR_GI();
    xSemaphoreTake(TC_semphr, 0);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_TX,             //
                              LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_INCREMENT     //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_TX, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_TX, (uint32_t)buf);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_TX, size);
    } while (false);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_RX,             //
                              LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_NOINCREMENT   //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_RX, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_RX, (uint32_t)&dummy);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_RX, size);
    } while (false);

    DMA_ENABLE_IT_TC();
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_TX);
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_RX);

    LL_SPI_EnableDMAReq_RX(SPIx);
    LL_SPI_Enable(SPIx);
    LL_SPI_EnableDMAReq_TX(SPIx);

    xSemaphoreTake(TC_semphr, portMAX_DELAY);

    while (LL_SPI_TX_FIFO_EMPTY != LL_SPI_GetTxFIFOLevel(SPIx))
    {
    }
    while (LL_SPI_IsActiveFlag_BSY(SPIx))
    {
    }
    while (LL_SPI_RX_FIFO_EMPTY != LL_SPI_GetRxFIFOLevel(SPIx))
    {
    }

    LL_SPI_DisableDMAReq_TX(SPIx);
    LL_SPI_DisableDMAReq_RX(SPIx);
}

static void SPI_DMA_fill(uint8_t n, uint32_t size)
{
    // TODO: investigate this!
    volatile auto n1 = n;

    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_RX);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_TX);

    DMA_CLEAR_GI();
    xSemaphoreTake(TC_semphr, 0);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_TX,             //
                              LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_NOINCREMENT   //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_TX, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_TX, (uint32_t)&n1);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_TX, size);
    } while (false);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_RX,             //
                              LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_NOINCREMENT   //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_RX, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_RX, (uint32_t)&dummy);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_RX, size);
    } while (false);

    DMA_ENABLE_IT_TC();
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_TX);
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_RX);

    LL_SPI_EnableDMAReq_RX(SPIx);
    LL_SPI_Enable(SPIx);
    LL_SPI_EnableDMAReq_TX(SPIx);

    xSemaphoreTake(TC_semphr, portMAX_DELAY);

    while (LL_SPI_TX_FIFO_EMPTY != LL_SPI_GetTxFIFOLevel(SPIx))
    {
    }
    while (LL_SPI_IsActiveFlag_BSY(SPIx))
    {
    }
    while (LL_SPI_RX_FIFO_EMPTY != LL_SPI_GetRxFIFOLevel(SPIx))
    {
    }

    LL_SPI_DisableDMAReq_TX(SPIx);
    LL_SPI_DisableDMAReq_RX(SPIx);
}

extern "C"
{
    void DMA_CHANNEL_IRQHandler()
    {
        if (DMA_CHECK_TC())
        {
            DMA_DISABLE_IT_TC();
            DMA_CLEAR_TC();
            xSemaphoreGiveFromISR(TC_semphr, NULL);
        }
    }
}

namespace st7565 = driver::st7565;
using namespace driver::lcd;

void driver::lcd::init(bool inverse)
{
    SPI_init();

    LL_mDelay(1);

    CS_assert();

    do
    {
        A0_cmd_mode();

        SPI_write_byte(st7565::cmd_reset());
        LL_mDelay(120);
        SPI_write_byte(st7565::cmd_bias_select(0));
        SPI_write_byte(st7565::cmd_MY(false));
        SPI_write_byte(st7565::cmd_MX(true));
        SPI_write_byte(st7565::cmd_inverse(inverse));
        SPI_write_byte(st7565::cmd_all_pixel_on(false));
        SPI_write_byte(st7565::cmd_regulation_ratio(4));
        SPI_write_byte(st7565::cmd_set_EV());
        SPI_write_byte(0x1f);
        SPI_write_byte(st7565::cmd_power_control(0b011));
        LL_mDelay(1);
        SPI_write_byte(st7565::cmd_power_control(0b110));
        LL_mDelay(1);
        for (uint32_t i = 0; i < 4; i++)
        {
            SPI_write_byte(st7565::cmd_power_control(0b111));
        }
        LL_mDelay(40);
        SPI_write_byte(st7565::cmd_set_start_line(0));
        SPI_write_byte(st7565::cmd_display_on(true));
    } while (false);

    for (uint32_t page = 0; page < LCD_PAGES; page++)
    {
        A0_cmd_mode();
        SPI_write_byte(st7565::cmd_set_page_addr(page));
        SPI_write_byte(st7565::cmd_set_col_addr_MSB(COL_ADDR_OFFSET));
        SPI_write_byte(st7565::cmd_set_col_addr_LSB(COL_ADDR_OFFSET));

        A0_data_mode();
        for (uint32_t col = 0; col < LCD_WIDTH; col++)
        {
            SPI_write_byte(0);
        }
    }

    CS_release();

    ::inverse = inverse;

    TC_semphr = xSemaphoreCreateBinaryStatic(&TC_semphr_obj);
    lcd_mutex = xSemaphoreCreateMutexStatic(&lcd_mutex_obj);
}

bool driver::lcd::lock(TickType_t block)
{
    return pdTRUE == xSemaphoreTake(lcd_mutex, block);
}

void driver::lcd::unlock()
{
    xSemaphoreGive(lcd_mutex);
}

void driver::lcd::fill(uint8_t px)
{
    CS_assert();

    for (uint32_t page = 0; page < LCD_PAGES; page++)
    {
        A0_cmd_mode();
        SPI_write_byte(st7565::cmd_set_page_addr(page));
        SPI_write_byte(st7565::cmd_set_col_addr_MSB(COL_ADDR_OFFSET));
        SPI_write_byte(st7565::cmd_set_col_addr_LSB(COL_ADDR_OFFSET));

        A0_data_mode();
        SPI_DMA_fill(px, LCD_WIDTH);
    }

    CS_release();
}

void driver::lcd::blit(uint32_t page, uint32_t col, const uint8_t *buf, uint32_t size)
{
    CS_assert();

    A0_cmd_mode();
    SPI_write_byte(st7565::cmd_set_page_addr(page));
    SPI_write_byte(st7565::cmd_set_col_addr_MSB(col + COL_ADDR_OFFSET));
    SPI_write_byte(st7565::cmd_set_col_addr_LSB(col + COL_ADDR_OFFSET));

    A0_data_mode();
    if (size < DMA_TX_MIN)
    {
        while (size > 0)
        {
            SPI_write_byte(*buf);
            buf++;
            size--;
        }
    }
    else
    {
        SPI_DMA_write_buf(buf, size);
    }

    CS_release();
}

bool driver::lcd::is_inverse()
{
    return ::inverse;
}

void driver::lcd::set_inverse(bool inverse)
{
    if (::inverse != inverse)
    {
        CS_assert();
        A0_cmd_mode();
        SPI_write_byte(st7565::cmd_inverse(inverse));
        CS_release();
        ::inverse = inverse;
    }
}
