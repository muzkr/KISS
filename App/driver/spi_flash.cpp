
#include "driver/spi_flash.hpp"
#include "py32f071_ll_spi.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_system.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define SPIx SPI2
#define DMA_CHANNEL_READ LL_DMA_CHANNEL_2
#define DMA_CHANNEL_WRITE LL_DMA_CHANNEL_3
#define DMA_PRIORITY LL_DMA_PRIORITY_HIGH
#define DMA_CHANNEL_IRQn DMA1_Channel2_3_IRQn
#define DMA_CHANNEL_IRQ_PRIORITY 3
#define DMA_CHANNEL_IRQHandler DMA1_Channel2_3_IRQHandler
#define DMA_ENABLE_IT_TC() LL_DMA_EnableIT_TC(DMA1, DMA_CHANNEL_READ)
#define DMA_DISABLE_IT_TC() LL_DMA_DisableIT_TC(DMA1, DMA_CHANNEL_READ)
#define DMA_CHECK_TC() LL_DMA_IsActiveFlag_TC2(DMA1)
#define DMA_CLEAR_TC() LL_DMA_ClearFlag_TC2(DMA1)
#define DMA_CLEAR_GI() LL_DMA_ClearFlag_GI2(DMA1)

// CS: PA3
#define CS_GPIOx GPIOA
#define CS_PIN LL_GPIO_PIN_3

namespace py25q16 = driver::py25q16;
using namespace driver::spi_flash;

using py25q16::PAGE_SIZE;

static void spi_init();
static uint8_t spi_write_byte(uint8_t n);
static void spi_read(uint8_t *buf, uint32_t size);
static void spi_write(const uint8_t *buf, uint32_t size);
static void spi_read_DMA(uint8_t *buf, uint32_t size);
static void spi_write_DMA(const uint8_t *buf, uint32_t size);

static inline void CS_assert() { LL_GPIO_ResetOutputPin(CS_GPIOx, CS_PIN); }
static inline void CS_release() { LL_GPIO_SetOutputPin(CS_GPIOx, CS_PIN); }

extern "C"
{
    void DMA_CHANNEL_IRQHandler();
}

static inline uint32_t sector_addr(uint32_t addr) { return (addr / SECTOR_SIZE) * SECTOR_SIZE; }
static inline uint32_t page_addr(uint32_t addr) { return (addr / PAGE_SIZE) * PAGE_SIZE; }

static void wait_WIP();
static void write_enable();
static void sector_erase(uint32_t addr);
static void page_program(uint32_t addr, const uint8_t *buf, uint32_t size);
static void write_sector(uint32_t addr, uint32_t off, const uint8_t *buf, uint32_t size, bool append);
static void program_sector(uint32_t addr, const uint8_t *buf, uint32_t size);

static uint8_t sector_cache[SECTOR_SIZE];
static uint32_t sector_cache_addr = CHIP_SIZE;
static uint8_t dummy;

static StaticSemaphore_t TC_semphr_obj;
static SemaphoreHandle_t TC_semphr;

void driver::spi_flash::init()
{
    spi_init();

    py25q16::spi_callbacks spi_cb =
        {
            .write_byte = spi_write_byte,
            // .write_data = spi_write,
            // .read_data = spi_read,
        };
    py25q16::set_spi_callbacks(spi_cb);

    TC_semphr = xSemaphoreCreateBinaryStatic(&TC_semphr_obj);
}

void driver::spi_flash::read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    if (0 == size)
    {
        return;
    }

    CS_assert();
    py25q16::read(addr);
    spi_read(buf, size);
    CS_release();
}

void driver::spi_flash::write(uint32_t addr, const uint8_t *buf, uint32_t size, bool append)
{
    if (0 == size)
    {
        return;
    }

    uint32_t sec_addr = sector_addr(addr);
    uint32_t sec_off = addr - sec_addr;
    uint32_t sec_size = SECTOR_SIZE - sec_off;

    for (;;)
    {
        if (sec_size > size)
        {
            sec_size = size;
        }

        size -= sec_size;

        write_sector(sec_addr, sec_off, buf, sec_size, append && (0 == size));

        if (0 == size)
        {
            break;
        }

        buf += sec_size;
        sec_addr += SECTOR_SIZE;
        sec_off = 0;
        sec_size = SECTOR_SIZE;
    }
}

void driver::spi_flash::erase_sector(uint32_t addr)
{
    sector_erase(addr);

    addr = sector_addr(addr);
    if (addr == sector_cache_addr)
    {
        memset(sector_cache, 0xff, SECTOR_SIZE);
    }
}

static void write_sector(uint32_t addr, uint32_t off, const uint8_t *buf, uint32_t size, bool append)
{
    if (addr != sector_cache_addr)
    {
        read(addr, sector_cache, SECTOR_SIZE);
        sector_cache_addr = addr;
    }

    if (0 == memcmp(sector_cache + off, buf, size))
    {
        return;
    }

    bool erase = false;
    for (uint32_t i = 0; i < size; i++)
    {
        const uint8_t b1 = sector_cache[off + i];
        if (0xff != b1 && buf[i] != b1)
        {
            erase = true;
            break;
        }
    }

    memcpy(sector_cache + off, buf, size);

    if (erase)
    {
        sector_erase(addr);

        if (append)
        {
            program_sector(addr, sector_cache, off + size);
            memset(sector_cache + (off + size), 0xff, SECTOR_SIZE - (off + size));
        }
        else
        {
            program_sector(addr, sector_cache, SECTOR_SIZE);
        }
    }
    else
    {
        program_sector(addr + off, buf, size);
    }
}

static void program_sector(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    if (0 == size)
    {
        return;
    }

    uint32_t page_addr = ::page_addr(addr);
    uint32_t page_off = addr - page_addr;
    uint32_t page_size = PAGE_SIZE - page_off;

    for (;;)
    {
        if (page_size > size)
        {
            page_size = size;
        }

        page_program(page_addr + page_off, buf, page_size);

        size -= page_size;
        if (0 == size)
        {
            break;
        }

        buf += page_size;
        page_addr += PAGE_SIZE;
        page_off = 0;
        page_size = PAGE_SIZE;
    }
}

static void wait_WIP()
{
    // CS_assert();
    // uint8_t s = py25q16::read_status_reg();
    // while (1 & s)
    // {
    //     s = spi_write_byte(0);
    // }
    // CS_release();

    for (;;)
    {
        CS_assert();
        bool b = py25q16::check_WIP();
        CS_release();

        if (b)
        {
            // Busy wait ?
            // for (uint32_t i = 0; i < 48; i++)
            // {
            //     __NOP();
            // }
            // taskYIELD();
        }
        else
        {
            break;
        }
    }
}

static void write_enable()
{
    CS_assert();
    py25q16::write_enable();
    CS_release();
}

static void sector_erase(uint32_t addr)
{
    write_enable();

    wait_WIP();

    CS_assert();
    py25q16::sector_erase(addr);
    CS_release();

    wait_WIP();
}

static void page_program(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    write_enable();
    wait_WIP();

    CS_assert();
    py25q16::page_program(addr);
    spi_write(buf, size);
    CS_release();

    wait_WIP();
}

static void spi_init()
{
    // CS: PA3
    // SCK: PA0
    // MOSI: PA1
    // MISO: PA2

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
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

        // CS
        init_struct.Pin = CS_PIN;
        LL_GPIO_Init(CS_GPIOx, &init_struct);

        // SPI ----

        init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
        init_struct.Pull = LL_GPIO_PULL_UP;

        // SCK: PA0
        init_struct.Alternate = LL_GPIO_AF8_SPI2;
        init_struct.Pin = LL_GPIO_PIN_0;
        // init_struct.Pull = LL_GPIO_PULL_UP;
        LL_GPIO_Init(GPIOA, &init_struct);

        // MOSI: PA1
        // MISO: PA2
        init_struct.Alternate = LL_GPIO_AF9_SPI2;
        init_struct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
        // init_struct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOA, &init_struct);

    } while (false);

    NVIC_DisableIRQ(DMA_CHANNEL_IRQn);
    NVIC_SetPriority(DMA_CHANNEL_IRQn, DMA_CHANNEL_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA_CHANNEL_IRQn);

    LL_SYSCFG_SetDMARemap(DMA1, DMA_CHANNEL_READ, LL_SYSCFG_DMA_MAP_SPI2_RD);
    LL_SYSCFG_SetDMARemap(DMA1, DMA_CHANNEL_WRITE, LL_SYSCFG_DMA_MAP_SPI2_WR);

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
        init_struct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
        LL_SPI_Init(SPIx, &init_struct);
    } while (false);

    LL_SPI_Enable(SPIx);
}

static uint8_t spi_write_byte(uint8_t n)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPIx))
    {
    }
    LL_SPI_TransmitData8(SPIx, n);

    while (!LL_SPI_IsActiveFlag_RXNE(SPIx))
    {
    }

    return LL_SPI_ReceiveData8(SPIx);
}

static void spi_write(const uint8_t *buf, uint32_t size)
{
    if (size < 16)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            spi_write_byte(buf[i]);
        }
    }
    else
    {
        spi_write_DMA(buf, size);
    }
}

static void spi_read(uint8_t *buf, uint32_t size)
{
    if (size < 16)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            buf[i] = spi_write_byte(0);
        }
    }
    else
    {
        spi_read_DMA(buf, size);
    }
}

static void spi_read_DMA(uint8_t *buf, uint32_t size)
{
    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_READ);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_WRITE);

    DMA_CLEAR_GI();
    xSemaphoreTake(TC_semphr, 0);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_WRITE,          //
                              LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_NOINCREMENT   //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_WRITE, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_WRITE, (uint32_t)&dummy);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_WRITE, size);
    } while (false);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_READ,           //
                              LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_INCREMENT     //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_READ, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_READ, (uint32_t)buf);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_READ, size);
    } while (false);

    DMA_ENABLE_IT_TC();
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_WRITE);
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_READ);

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

static void spi_write_DMA(const uint8_t *buf, uint32_t size)
{
    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_READ);
    LL_DMA_DisableChannel(DMA1, DMA_CHANNEL_WRITE);

    DMA_CLEAR_GI();
    xSemaphoreTake(TC_semphr, 0);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_WRITE,          //
                              LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_INCREMENT     //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_WRITE, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_WRITE, (uint32_t)buf);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_WRITE, size);
    } while (false);

    do
    {
        LL_DMA_ConfigTransfer(DMA1, DMA_CHANNEL_READ,           //
                              LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                                  | LL_DMA_MODE_NORMAL          //
                                  | LL_DMA_PERIPH_NOINCREMENT   //
                                  | LL_DMA_MEMORY_NOINCREMENT   //
                                  | LL_DMA_PDATAALIGN_BYTE      //
                                  | LL_DMA_MDATAALIGN_BYTE      //
                                  | DMA_PRIORITY                //
        );

        LL_DMA_SetPeriphAddress(DMA1, DMA_CHANNEL_READ, LL_SPI_DMA_GetRegAddr(SPIx));
        LL_DMA_SetMemoryAddress(DMA1, DMA_CHANNEL_READ, (uint32_t)&dummy);
        LL_DMA_SetDataLength(DMA1, DMA_CHANNEL_READ, size);
    } while (false);

    DMA_ENABLE_IT_TC();
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_WRITE);
    LL_DMA_EnableChannel(DMA1, DMA_CHANNEL_READ);

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

void DMA_CHANNEL_IRQHandler()
{
    if (DMA_CHECK_TC())
    {
        DMA_DISABLE_IT_TC();
        DMA_CLEAR_TC();
        xSemaphoreGiveFromISR(TC_semphr, NULL);
    }
}
