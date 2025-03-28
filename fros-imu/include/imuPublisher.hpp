#ifndef IMU_PUBLISHER_HPP
#define IMU_PUBLISHER_HPP

#include <fros/urosElement.hpp>
#include <genericImu.hpp>


#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>


#define IMU_DEF_FREQ 20

class imuPublisher : public urosElement, public genericImu {

public:

    // initialize hardware interface here
    imuPublisher();

    // initialize micro-ros publisher / subscribers here
    void init();
    void declareParameters();
    

    static void imu_publish_callback(rcl_timer_s* time, int64_t num);

    void publishMsg();


private:

    // float pubDelayMs = 1000 / IMU_DEF_FREQ;
    rcl_timer_t timer;
    rcl_publisher_t imu_publisher, mag_publisher;
    static imuPublisher* defaultPub;


    sensor_msgs__msg__Imu imu_msg;
    sensor_msgs__msg__MagneticField mag_msg;

};


#endif //  IMU_PUBLISHER_HPP
