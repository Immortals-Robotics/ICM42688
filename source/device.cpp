#include "device.h"

#include <assert.h>
#include <cstring>
#include <thread>

#include "hal.h"

namespace ICM42688
{
#if ICM42688_FEATURE_I2C
/* Device object, input the I2C bus and address */
Device::Device(uint8_t idx, uint8_t address)
{
    device_idx = idx;
    _address   = address; // I2C address
    _useSPI    = false;   // set to use I2C
}
#endif

#if ICM42688_FEATURE_SPI
/* Device object, input the SPI bus and chip select pin */
Device::Device(uint8_t idx)
{
    device_idx = idx;
    _useSPI    = true; // set to use SPI
}
#endif

/* starts communication with the ICM42688 */
int Device::begin()
{
    // reset the ICM42688
    reset();

    // check the WHO AM I byte
    if (whoAmI() != WHO_AM_I)
    {
        return -3;
    }

    // turn on accel and gyro in Low Noise (LN) Mode
    if (writeRegister(Register::UB0_REG_PWR_MGMT0, 0x0F) < 0)
    {
        return -4;
    }

    // 16G is default -- do this to set up accel resolution scaling
    int ret = setAccelFS(gpm16);
    if (ret < 0)
        return ret;

    // 2000DPS is default -- do this to set up gyro resolution scaling
    ret = setGyroFS(dps2000);
    if (ret < 0)
        return ret;

    // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
    // if (setFilters(false, false) < 0) {
    //   return -7;
    // }

    // estimate gyro bias
    if (calibrateGyro() < 0)
    {
        return -8;
    }
    // successful init, return 1
    return 1;
}

/* sets the accelerometer full scale range to values other than default */
int Device::setAccelFS(AccelFS fssel)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change FS_SEL in reg
    reg = (fssel << 5) | (reg & 0x1F);

    if (writeRegister(Register::UB0_REG_ACCEL_CONFIG0, reg) < 0)
        return -2;

    _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
    _accelFS    = fssel;

    return 1;
}

/* sets the gyro full scale range to values other than default */
int Device::setGyroFS(GyroFS fssel)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change FS_SEL in reg
    reg = (fssel << 5) | (reg & 0x1F);

    if (writeRegister(Register::UB0_REG_GYRO_CONFIG0, reg) < 0)
        return -2;

    _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
    _gyroFS    = fssel;

    return 1;
}

int Device::setAccelODR(ODR odr)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change ODR in reg
    reg = odr | (reg & 0xF0);

    if (writeRegister(Register::UB0_REG_ACCEL_CONFIG0, reg) < 0)
        return -2;

    return 1;
}

int Device::setGyroODR(ODR odr)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change ODR in reg
    reg = odr | (reg & 0xF0);

    if (writeRegister(Register::UB0_REG_GYRO_CONFIG0, reg) < 0)
        return -2;

    return 1;
}

int Device::setFilters(bool gyroFilters, bool accFilters)
{
    if (setBank(1) < 0)
        return -1;

    if (gyroFilters == true)
    {
        if (writeRegister(Register::UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0)
        {
            return -2;
        }
    }
    else
    {
        if (writeRegister(Register::UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0)
        {
            return -3;
        }
    }

    if (setBank(2) < 0)
        return -4;

    if (accFilters == true)
    {
        if (writeRegister(Register::UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0)
        {
            return -5;
        }
    }
    else
    {
        if (writeRegister(Register::UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0)
        {
            return -6;
        }
    }
    if (setBank(0) < 0)
        return -7;
    return 1;
}

int Device::enableDataReadyInterrupt()
{
    // push-pull, pulsed, active HIGH interrupts
    if (writeRegister(Register::UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0)
        return -1;

    // need to clear bit 4 to allow proper INT1 and INT2 operation
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_INT_CONFIG1, 1, &reg) < 0)
        return -2;
    reg &= ~0x10;
    if (writeRegister(Register::UB0_REG_INT_CONFIG1, reg) < 0)
        return -3;

    // route UI data ready interrupt to INT1
    if (writeRegister(Register::UB0_REG_INT_SOURCE0, 0x18) < 0)
        return -4;

    return 1;
}

int Device::disableDataReadyInterrupt()
{
    // set pin 4 to return to reset value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_INT_CONFIG1, 1, &reg) < 0)
        return -1;
    reg |= 0x10;
    if (writeRegister(Register::UB0_REG_INT_CONFIG1, reg) < 0)
        return -2;

    // return reg to reset value
    if (writeRegister(Register::UB0_REG_INT_SOURCE0, 0x10) < 0)
        return -3;

    return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int Device::getAGT()
{
    // grab the data from the ICM42688
    if (readRegisters(Register::UB0_REG_TEMP_DATA1, 14, _buffer) < 0)
        return -1;

    // combine bytes into 16 bit values
    for (size_t i = 0; i < 7; i++)
    {
        _rawMeas[i] = ((int16_t) _buffer[i * 2] << 8) | _buffer[i * 2 + 1];
    }

    _t = (static_cast<float>(_rawMeas[0]) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

    _acc[0] = ((_rawMeas[1] * _accelScale) - _accB[0]) * _accS[0];
    _acc[1] = ((_rawMeas[2] * _accelScale) - _accB[1]) * _accS[1];
    _acc[2] = ((_rawMeas[3] * _accelScale) - _accB[2]) * _accS[2];

    _gyr[0] = (_rawMeas[4] * _gyroScale) - _gyrB[0];
    _gyr[1] = (_rawMeas[5] * _gyroScale) - _gyrB[1];
    _gyr[2] = (_rawMeas[6] * _gyroScale) - _gyrB[2];

    return 1;
}

/* estimates the gyro biases */
int Device::calibrateGyro()
{
    // set at a lower range (more resolution) since IMU not moving
    const GyroFS current_fssel = _gyroFS;
    if (setGyroFS(dps250) < 0)
        return -1;

    // take samples and find bias
    _gyroBD[0] = 0;
    _gyroBD[1] = 0;
    _gyroBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++)
    {
        getAGT();
        _gyroBD[0] += (gyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
        _gyroBD[1] += (gyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
        _gyroBD[2] += (gyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    _gyrB[0] = _gyroBD[0];
    _gyrB[1] = _gyroBD[1];
    _gyrB[2] = _gyroBD[2];

    // recover the full scale setting
    if (setGyroFS(current_fssel) < 0)
        return -4;
    return 1;
}

/* returns the gyro bias in the X direction, dps */
float Device::getGyroBiasX()
{
    return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float Device::getGyroBiasY()
{
    return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float Device::getGyroBiasZ()
{
    return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void Device::setGyroBiasX(float bias)
{
    _gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void Device::setGyroBiasY(float bias)
{
    _gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void Device::setGyroBiasZ(float bias)
{
    _gyrB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int Device::calibrateAccel()
{
    // set at a lower range (more resolution) since IMU not moving
    const AccelFS current_fssel = _accelFS;
    if (setAccelFS(gpm2) < 0)
        return -1;

    // take samples and find min / max
    _accBD[0] = 0;
    _accBD[1] = 0;
    _accBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++)
    {
        getAGT();
        _accBD[0] += (accX() / _accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
        _accBD[1] += (accY() / _accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
        _accBD[2] += (accZ() / _accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (_accBD[0] > 0.9f)
    {
        _accMax[0] = _accBD[0];
    }
    if (_accBD[1] > 0.9f)
    {
        _accMax[1] = _accBD[1];
    }
    if (_accBD[2] > 0.9f)
    {
        _accMax[2] = _accBD[2];
    }
    if (_accBD[0] < -0.9f)
    {
        _accMin[0] = _accBD[0];
    }
    if (_accBD[1] < -0.9f)
    {
        _accMin[1] = _accBD[1];
    }
    if (_accBD[2] < -0.9f)
    {
        _accMin[2] = _accBD[2];
    }

    // find bias and scale factor
    if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f))
    {
        _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
        _accS[0] = 1 / ((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
    }
    if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f))
    {
        _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
        _accS[1] = 1 / ((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
    }
    if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f))
    {
        _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
        _accS[2] = 1 / ((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
    }

    // recover the full scale setting
    if (setAccelFS(current_fssel) < 0)
        return -4;
    return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float Device::getAccelBiasX_mss()
{
    return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
float Device::getAccelScaleFactorX()
{
    return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float Device::getAccelBiasY_mss()
{
    return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
float Device::getAccelScaleFactorY()
{
    return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float Device::getAccelBiasZ_mss()
{
    return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
float Device::getAccelScaleFactorZ()
{
    return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void Device::setAccelCalX(float bias, float scaleFactor)
{
    _accB[0] = bias;
    _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void Device::setAccelCalY(float bias, float scaleFactor)
{
    _accB[1] = bias;
    _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void Device::setAccelCalZ(float bias, float scaleFactor)
{
    _accB[2] = bias;
    _accS[2] = scaleFactor;
}

/* returns the accelerometer measurement in the x direction, raw 16-bit integer */
int16_t Device::getAccelX_count()
{
    return _rawMeas[1];
}

/* returns the accelerometer measurement in the y direction, raw 16-bit integer */
int16_t Device::getAccelY_count()
{
    return _rawMeas[2];
}

/* returns the accelerometer measurement in the z direction, raw 16-bit integer */
int16_t Device::getAccelZ_count()
{
    return _rawMeas[3];
}

/* returns the gyroscople measurement in the x direction, raw 16-bit integer */
int16_t Device::getGyroX_count()
{
    return _rawMeas[4];
}

/* returns the gyroscople measurement in the y direction, raw 16-bit integer */
int16_t Device::getGyroY_count()
{
    return _rawMeas[5];
}

/* returns the gyroscople measurement in the z direction, raw 16-bit integer */
int16_t Device::getGyroZ_count()
{
    return _rawMeas[6];
}

/* writes a byte to ICM42688 register given a register address and data */
int Device::writeRegister(Register t_register, uint8_t data)
{
    /* write data to device */
#if ICM42688_FEATURE_SPI
    if (_useSPI)
    {
        const uint8_t buffer[2] = {(uint8_t) t_register, data};
        spiTransfer(device_idx, &buffer[0], nullptr, 2);
    }
#endif
#if ICM42688_FEATURE_I2C
    if (!_useSPI)
    {
        i2cWrite(device_idx, (uint8_t) t_register); // write the register address
        i2cWrite(device_idx, data);                 // write the data
    }
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    /* read back the register */
    readRegisters(t_register, 1, _buffer);
    /* check the read back register against the written register */
    if (_buffer[0] == data)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int Device::readRegisters(Register t_register, uint8_t count, uint8_t *dest)
{
#if ICM42688_FEATURE_SPI
    if (_useSPI)
    {
        assert(count < 16);

        uint8_t buffer[16] = {};
        buffer[0]          = (uint8_t) t_register | 0x80;
        spiTransfer(device_idx, &buffer[0], &buffer[0], count + 1);
        memcpy(dest, buffer, count);

        return 1;
    }
#endif

#if ICM42688_FEATURE_I2C
    if (!_useSPI)
    {
        i2cWrite(device_idx, (uint8_t) t_register); // specify the starting register address
        _numBytes = count;
        for (uint8_t i = 0; i < count; i++)
        {
            dest[i] = i2cRead(device_idx);
        }

        return 1;
    }
#endif

    return -1;
}

int Device::setBank(uint8_t bank)
{
    // if we are already on this bank, bail
    if (_bank == bank)
        return 1;

    _bank = bank;

    return writeRegister(Register::REG_BANK_SEL, bank);
}

void Device::reset()
{
    setBank(0);

    writeRegister(Register::UB0_REG_DEVICE_CONFIG, 0x01);

    // wait for ICM42688 to come back up
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t Device::whoAmI()
{
    setBank(0);

    // read the WHO AM I register
    if (readRegisters(Register::UB0_REG_WHO_AM_I, 1, _buffer) < 0)
    {
        return -1;
    }
    // return the register value
    return _buffer[0];
}
} // namespace ICM42688
