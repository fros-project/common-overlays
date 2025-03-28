#ifndef GENERIC_IMU_HPP
#define GENERIC_IMU_HPP



// base class for imu Definitions
class genericImu {
public:

    struct imuData
    {
        
        float ax, ay, az;   // accelaration
        float gx, gy, gz;   // angular accelaration
        float mx, my, mz;   // magnetometer readings
        float x, y, z, w;   // orientation
    } data;


    virtual void initHwi() {};
    virtual void fetchData() {};

    // virtual ~genericImu() = 0;

};


#endif //  GENERIC_IMU_HPP