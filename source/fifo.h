#pragma once
#include "device.h"

#if ICM42688_FEATURE_FIFO
namespace ICM42688
{
class Fifo : public Device
{
public:
    using Device::Device;
    int  enableFifo(bool accel, bool gyro, bool temp);
    int  readFifo();
    void getFifoAccelX_mss(size_t *size, float *data);
    void getFifoAccelY_mss(size_t *size, float *data);
    void getFifoAccelZ_mss(size_t *size, float *data);
    void getFifoGyroX(size_t *size, float *data);
    void getFifoGyroY(size_t *size, float *data);
    void getFifoGyroZ(size_t *size, float *data);
    void getFifoTemperature_C(size_t *size, float *data);

protected:
    // fifo
    bool   _enFifoAccel   = false;
    bool   _enFifoGyro    = false;
    bool   _enFifoTemp    = false;
    size_t _fifoSize      = 0;
    size_t _fifoFrameSize = 0;
    float  _axFifo[85]    = {};
    float  _ayFifo[85]    = {};
    float  _azFifo[85]    = {};
    size_t _aSize         = 0;
    float  _gxFifo[85]    = {};
    float  _gyFifo[85]    = {};
    float  _gzFifo[85]    = {};
    size_t _gSize         = 0;
    float  _tFifo[256]    = {};
    size_t _tSize         = 0;
};
} // namespace ICM42688
#endif
