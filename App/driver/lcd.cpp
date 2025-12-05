
#include "driver/lcd.hpp"
#include "py32f071_ll_spi.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_utils.h"
#include "py32f071_ll_system.h"

#include "FreeRTOS.h"
#include "task.h"

// PB2
#define CS_PORT GPIOB
#define CS_PIN LL_GPIO_PIN_2

// PA6
#define A0_PORT GPIOA
#define A0_PIN LL_GPIO_PIN_6

#define SPIx SPI1
// #define DMAx DMA1
// #define DMA_CHANNEL LL_DMA_CHANNEL_2
// #define DMA_PRIORITY LL_DMA_PRIORITY_MEDIUM
// #define DMA_CHANNEL_IRQn DMA1_Channel2_3_IRQn
// #define DMA_CHANNEL_IRQ_PRIORITY 1
// #define DMA_CHANNEL_IRQHandler DMA1_Channel2_3_IRQHandler

#define COL_ADDR_OFFSET 4

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
    // LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

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

    // do
    // {
    //     LL_DMA_InitTypeDef init_struct;
    //     init_struct.Mode = LL_DMA_MODE_NORMAL;
    //     init_struct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    //     init_struct.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(SPIx);
    //     init_struct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    //     init_struct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    //     init_struct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    //     init_struct.Priority = DMA_PRIORITY;
    //     LL_DMA_Init(DMAx, DMA_CHANNEL, &init_struct);
    // } while (false);

    // LL_DMA_DisableChannel(DMAx, DMA_CHANNEL);
    // LL_SYSCFG_SetDMARemap(DMAx, DMA_CHANNEL, LL_SYSCFG_DMA_MAP_SPI1_WR);
    // NVIC_SetPriority(DMA_CHANNEL_IRQn, DMA_CHANNEL_IRQ_PRIORITY);
    // NVIC_EnableIRQ(DMA_CHANNEL_IRQn);
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

static void SPI_write_byte_yield(uint8_t n)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPIx))
    {
        vPortYield();
    }
    LL_SPI_TransmitData8(SPIx, n);

    while (!LL_SPI_IsActiveFlag_RXNE(SPIx))
    {
        vPortYield();
    }

    LL_SPI_ReceiveData8(SPIx);
}

namespace st7565
{
    constexpr uint8_t cmd_display_on(bool on)
    {
        return 0b1010'1110 | (on ? 1 : 0);
    }

    constexpr uint8_t cmd_set_start_line(uint8_t n)
    {
        return 0b0100'0000 | (0b11'1111 & n);
    }

    constexpr uint8_t cmd_set_page_addr(uint8_t n)
    {
        return 0b1011'0000 | (0b1111 & n);
    }

    constexpr uint8_t cmd_set_col_addr_MSB(uint8_t n)
    {
        return 0b0001'0000 | (0b1111 & (n >> 4));
    }

    constexpr uint8_t cmd_set_col_addr_LSB(uint8_t n)
    {
        return 0b0000'0000 | (0b1111 & n);
    }

    constexpr uint8_t cmd_inverse(bool inv)
    {
        return 0b1010'0110 | (inv ? 1 : 0);
    }

    constexpr uint8_t cmd_all_pixel_on(bool on)
    {
        return 0b1010'0100 | (on ? 1 : 0);
    }

    constexpr uint8_t cmd_reset = 0b1110'0010;

    constexpr uint8_t cmd_regulation_ratio(uint8_t n)
    {
        return 0b0010'0000 | (0b111 & n);
    }

    constexpr uint8_t cmd_power_control(uint8_t n)
    {
        return 0b0010'1000 | (0b111 & n);
    }

    constexpr uint8_t cmd_MY(bool MY)
    {
        return 0b1100'0000 | ((MY ? 1 : 0) << 3);
    }

    constexpr uint8_t cmd_MX(bool MX)
    {
        return 0b1010'0000 | (MX ? 1 : 0);
    }

    constexpr uint8_t cmd_set_EV = 0b1000'0001;

    constexpr uint8_t cmd_bias_select(uint8_t n)
    {
        return 0b1010'0010 | (1 & n);
    }

} // namespace st7565

using namespace driver::lcd;

void driver::lcd::init(bool inverse)
{
    SPI_init();

    LL_mDelay(2);

    CS_assert();

    do
    {
        A0_cmd_mode();

        SPI_write_byte(st7565::cmd_reset);
        LL_mDelay(120);
        SPI_write_byte(st7565::cmd_bias_select(0));
        SPI_write_byte(st7565::cmd_MY(false));
        SPI_write_byte(st7565::cmd_MX(true));
        SPI_write_byte(st7565::cmd_inverse(inverse));
        SPI_write_byte(st7565::cmd_all_pixel_on(false));
        SPI_write_byte(st7565::cmd_regulation_ratio(4));
        SPI_write_byte(st7565::cmd_set_EV);
        SPI_write_byte(0x1f);
        SPI_write_byte(st7565::cmd_power_control(0x3));
        LL_mDelay(1);
        SPI_write_byte(st7565::cmd_power_control(0x6));
        LL_mDelay(1);
        SPI_write_byte(st7565::cmd_power_control(0x7));
        SPI_write_byte(st7565::cmd_power_control(0x7));
        SPI_write_byte(st7565::cmd_power_control(0x7));
        SPI_write_byte(st7565::cmd_power_control(0x7));
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
}

void driver::lcd::fill(uint8_t px)
{
    CS_assert();

    for (uint32_t page = 0; page < LCD_PAGES; page++)
    {
        A0_cmd_mode();
        SPI_write_byte_yield(st7565::cmd_set_page_addr(page));
        SPI_write_byte_yield(st7565::cmd_set_col_addr_MSB(COL_ADDR_OFFSET));
        SPI_write_byte_yield(st7565::cmd_set_col_addr_LSB(COL_ADDR_OFFSET));
        // SPI_write_byte(st7565::cmd_set_page_addr(page));
        // SPI_write_byte(st7565::cmd_set_col_addr_MSB(COL_ADDR_OFFSET));
        // SPI_write_byte(st7565::cmd_set_col_addr_LSB(COL_ADDR_OFFSET));

        A0_data_mode();
        for (uint32_t col = 0; col < LCD_WIDTH; col++)
        {
            SPI_write_byte_yield(px);
            // SPI_write_byte(px);
        }
    }

    CS_release();
}
