#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C"
{
#endif
    // Users of the library need to implement these
#if ICM42688_FEATURE_SPI
    extern void spiTransfer(uint8_t idx, const uint8_t *tx_buf, uint8_t *rx_buf, size_t count);
#endif
#if ICM42688_FEATURE_I2C
    extern void    i2cWrite(uint8_t idx, uint8_t data);
    extern uint8_t i2cRead(uint8_t idx);
#endif
#ifdef __cplusplus
}
#endif
