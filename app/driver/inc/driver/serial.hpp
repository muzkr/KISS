#ifndef _DRIVER_SERIAL_HPP
#define _DRIVER_SERIAL_HPP

#include "KISS_config.h"

#if KISS_ENABLE_SERIAL

#include "FreeRTOS.h"

static_assert(KISS_ENABLE_SERIAL_TX || KISS_ENABLE_SERIAL_RX, "One of KISS_ENABLE_SERIAL_TX and KISS_ENABLE_SERIAL_RX must be enabled");

namespace driver::serial
{
    void init();

#if KISS_ENABLE_SERIAL_TX
    bool lock_tx(TickType_t block = portMAX_DELAY);
    void unlock_tx();
    uint32_t send(const uint8_t *buf, uint32_t size, TickType_t block = portMAX_DELAY);
#endif // KISS_ENABLE_SERIAL_TX

#if KISS_ENABLE_SERIAL_RX

#endif // KISS_ENABLE_SERIAL_RX

} // namespace driver::serial

#endif // KISS_ENABLE_SERIAL

#endif // _DRIVER_SERIAL_HPP
