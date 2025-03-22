
// #include <stdio.h>

#include "urosElement.hpp"
#include "esp_log.h"

#define LOG "DRIVE_SYSTEM"

#include <rclc/timer.h>
#include <std_msgs/msg/int32.h>
#include <rcl/publisher.h>
#include <rclc/publisher.h>
#include <rclc/executor.h>

#include "velDriver.hpp"
#include <driver/gpio.h>
#include "pinmap.hpp"

int pwmPins[] = { PWM_FL, PWM_FR, PWM_BL, PWM_BR};// LIFT_PWM}; // LIFT_PWM, GRAB_PWM};
int dirPins[] = { DIR_FL, DIR_FR, DIR_BL, DIR_BR};;// LIFT_DIR}; // LIFT_DIR, GRAB_DIR};



extern "C" void start();

extern "C" void* overlay_main(){
    return new velDriver();
}