
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

int pwmPins[] = { GPIO_NUM_0, GPIO_NUM_0, GPIO_NUM_0, GPIO_NUM_0};// LIFT_PWM}; // LIFT_PWM, GRAB_PWM};
int dirPins[] = { GPIO_NUM_0, GPIO_NUM_0, GPIO_NUM_0, GPIO_NUM_0};// LIFT_DIR}; // LIFT_DIR, GRAB_DIR};


qmd* gen = new qmd(pwmPins, dirPins, 4);

extern "C" void start();

extern "C" void* overlay_main(){
    gen->setInvertingMode(false);
    gen->setRange(19900, 0);

    velDriver* drv = new velDriver(gen);
    return drv;
}