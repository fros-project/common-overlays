#ifndef PINMAP_HPP
#define PINMAP_HPP

#include <esp_log.h>
#include <driver/gpio.h>



#define TX0 GPIO_NUM_43
#define RX0 GPIO_NUM_44
#define TX1 GPIO_NUM_17
#define RX1 GPIO_NUM_18

// FL
#define DIR_FL GPIO_NUM_18
#define PWM_FL GPIO_NUM_8


// FR
#define DIR_FR GPIO_NUM_7
#define PWM_FR GPIO_NUM_15

// BL
#define DIR_BL GPIO_NUM_16
#define PWM_BL GPIO_NUM_17 

// BR
#define DIR_BR GPIO_NUM_6
#define PWM_BR GPIO_NUM_5


#define ENC_FL_A GPIO_NUM_41
#define ENC_FL_B GPIO_NUM_40

#define ENC_FR_A GPIO_NUM_42
#define ENC_FR_B GPIO_NUM_44

#define ENC_BL_A GPIO_NUM_35
#define ENC_BL_B GPIO_NUM_36

#define ENC_BR_A GPIO_NUM_37
#define ENC_BR_B GPIO_NUM_39


#define MPU_SDA GPIO_NUM_11
#define MPU_SCL GPIO_NUM_10
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

#define ICM_CS GPIO_NUM_10
#define ICM_CLK GPIO_NUM_12
#define ICM_MISO GPIO_NUM_11
#define ICM_MOSI GPIO_NUM_13



#endif // PINMAP_HPP
