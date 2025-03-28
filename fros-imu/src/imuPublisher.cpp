#include "imuPublisher.hpp"

#include <esp_log.h>
#include <rclc/publisher.h>


#include <pinmap.hpp>
#define LOG "imu_pub"



const rosidl_message_type_support_t * imu_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
const rosidl_message_type_support_t * mag_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField);

imuPublisher*  imuPublisher::defaultPub = 0;


imuPublisher::imuPublisher() {
    defaultPub = this;
}


void imuPublisher::init()
{
    
    memset(&imu_msg, 0, sizeof(imu_msg));
    memset(&mag_msg, 0, sizeof(mag_msg));

    imu_msg.header.frame_id.data = "base_link";
    imu_msg.header.frame_id.size = 9;
    imu_msg.header.frame_id.capacity = 4;

    memcpy(&mag_msg.header, &imu_msg.header, sizeof(imu_msg.header));

    rclc_publisher_init_default(&imu_publisher, node, imu_type_support, "imu/data_raw");
    rclc_publisher_init_default(&mag_publisher, node, mag_type_support, "imu/mag");

    rclc_timer_init_default(&timer, support, RCL_MS_TO_NS(50), imuPublisher::imu_publish_callback);

    rclc_executor_add_timer(exec, &timer);
}

void imuPublisher::declareParameters(){

};

void imuPublisher::imu_publish_callback(rcl_timer_s* time, int64_t num){

    if(defaultPub) defaultPub->publishMsg();
}

void imuPublisher::publishMsg() {

    fetchData();

    imu_msg.linear_acceleration = {.x = data.ax, .y = data.ay, .z = data.az};
    imu_msg.angular_velocity = {.x = data.gx, .y = data.gy, .z = data.gz};
    imu_msg.orientation = { .x = data.x, .y = data.y, .z = data.z, .w = data.w };
    mag_msg.magnetic_field = { .x = data.mx, .y = data.my, .z = data.mz };

    rcl_publish(&imu_publisher, &imu_msg, NULL);
    rcl_publish(&mag_publisher, &mag_msg, NULL);
};
