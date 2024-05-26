#include "fifo.h"

#include <assert.h>
#include <cstring>

#if ICM42688_FEATURE_FIFO
namespace ICM42688
{
/* configures and enables the FIFO buffer  */
int Fifo::enableFifo(bool accel, bool gyro, bool temp)
{
    if (writeRegister(Register::UB0_REG_FIFO_CONFIG1,
                      (accel * FIFO_ACCEL) | (gyro * FIFO_GYRO) | (temp * FIFO_TEMP_EN)) < 0)
    {
        return -2;
    }
    _enFifoAccel   = accel;
    _enFifoGyro    = gyro;
    _enFifoTemp    = temp;
    _fifoFrameSize = accel * 6 + gyro * 6 + temp * 2;
    return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
int Fifo::readFifo()
{
    // get the fifo size
    readRegisters(Register::UB0_REG_FIFO_COUNTH, 2, _buffer);
    _fifoSize = (((uint16_t) (_buffer[0] & 0x0F)) << 8) + (((uint16_t) _buffer[1]));
    // read and parse the buffer
    for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++)
    {
        // grab the data from the ICM42688
        if (readRegisters(Register::UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer) < 0)
        {
            return -1;
        }
        if (_enFifoAccel)
        {
            // combine into 16 bit values
            int16_t rawMeas[3];
            rawMeas[0] = (((int16_t) _buffer[0]) << 8) | _buffer[1];
            rawMeas[1] = (((int16_t) _buffer[2]) << 8) | _buffer[3];
            rawMeas[2] = (((int16_t) _buffer[4]) << 8) | _buffer[5];
            // transform and convert to float values
            _axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
            _ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
            _azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
            _aSize     = _fifoSize / _fifoFrameSize;
        }
        if (_enFifoTemp)
        {
            // combine into 16 bit values
            int16_t rawMeas = (((int16_t) _buffer[0 + _enFifoAccel * 6]) << 8) | _buffer[1 + _enFifoAccel * 6];
            // transform and convert to float values
            _tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
            _tSize    = _fifoSize / _fifoFrameSize;
        }
        if (_enFifoGyro)
        {
            // combine into 16 bit values
            int16_t rawMeas[3];
            rawMeas[0] = (((int16_t) _buffer[0 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) |
                         _buffer[1 + _enFifoAccel * 6 + _enFifoTemp * 2];
            rawMeas[1] = (((int16_t) _buffer[2 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) |
                         _buffer[3 + _enFifoAccel * 6 + _enFifoTemp * 2];
            rawMeas[2] = (((int16_t) _buffer[4 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) |
                         _buffer[5 + _enFifoAccel * 6 + _enFifoTemp * 2];
            // transform and convert to float values
            _gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
            _gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
            _gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
            _gSize     = _fifoSize / _fifoFrameSize;
        }
    }
    return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void Fifo::getFifoAccelX_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _axFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void Fifo::getFifoAccelY_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _ayFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void Fifo::getFifoAccelZ_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _azFifo, _aSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, dps */
void Fifo::getFifoGyroX(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gxFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, dps */
void Fifo::getFifoGyroY(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gyFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, dps */
void Fifo::getFifoGyroZ(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gzFifo, _gSize * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void Fifo::getFifoTemperature_C(size_t *size, float *data)
{
    *size = _tSize;
    memcpy(data, _tFifo, _tSize * sizeof(float));
}
} // namespace ICM42688
#endif
