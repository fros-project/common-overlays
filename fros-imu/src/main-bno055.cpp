#include "esp_log.h"
#include "genericImu.hpp"
#include "imuPublisher.hpp"
#include "fros/urosHandler.hpp"

extern "C"{
    #include "bno055.h"
}


#include "driver/i2c_master.h"

#define BNO_DEF_ADDR 0x28
#define BNO_DEF_DPIN GPIO_NUM_2
#define BNO_DEF_CPIN GPIO_NUM_1
#define BNO_CLK 400000
#define BNO_CHIP_ID 0xA0


#define TAG "bno055"

class bno055 : public imuPublisher {
// class bno055 {
public:

    struct config : public urosElement::config
    {
        config() : urosElement::config("bno055", sizeof(bno055::config)) { 
            ESP_LOGI(TAG, "configuration loaded");
            load(); 
        };

        int64_t clk_pin = BNO_DEF_CPIN, data_pin = BNO_DEF_DPIN, address = BNO_DEF_ADDR;
    } cfg;
    
    bno055(){
        active = this;

        i2c_master_bus_config_t i2c_mst_config = {
            .i2c_port = -1,  // auto select port
            .sda_io_num = gpio_num_t(cfg.data_pin),
            .scl_io_num = gpio_num_t(cfg.clk_pin),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {.enable_internal_pullup = true }
        };
        

        i2c_new_master_bus(&i2c_mst_config, &bus_handle);
        
        
        // ESP_LOGI(TAG, "checking if bno055 exists at pin d %ld c %ld ", gpio_num_t(cfg.data_pin), gpio_num_t(cfg.clk_pin));
        // check if device exists,
        if(i2c_master_probe(bus_handle, cfg.address, 100)){
            ESP_LOGE(TAG, "bno055 not found !!");
            return;
        }
        else ESP_LOGI(TAG, "probe successful ! bno055 found");
        
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = cfg.address,
            .scl_speed_hz = BNO_CLK,
            // .scl_wait_us = 10000
        };
        
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
        initHwi();
        fetchData();
    };

    
    void declareParameters(){
        urosHandler::addParameter_int("i2c clock pin", &cfg.clk_pin, &cfg, nullptr, 1, 64, 1);
        urosHandler::addParameter_int("i2c data pin", &cfg.data_pin, &cfg, nullptr, 1, 64, 1);
        urosHandler::addParameter_int("bno055 address", &cfg.address, &cfg);
    }

    
    void initHwi(){
        ESP_LOGI(TAG, "initializing ");
        
        memset(&bno_config, 0, sizeof(bno_config));

        bno_config.dev_addr = uint8_t(cfg.address);
        bno_config.bus_write = bno_hwi_write;
        bno_config.bus_read = bno_hwi_read;
        bno_config.delay_msec = bno_hwi_delay;
        
        if(bno055_init(&bno_config)){
            // if initialization fails
            ESP_LOGI(TAG, "initialization failed");
        };

        uint8_t chip_id = 0;
        
        bno055_read_chip_id(&chip_id);

        if(chip_id != BNO_CHIP_ID){
            ESP_LOGE(TAG, " chip id mismatch ");
            initSuccessful = false;
        };

        bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
        ESP_LOGI(TAG, "initialization complete ");
        initSuccessful = true;
    };
    
    

    static void bno_hwi_delay(BNO055_MDELAY_DATA_TYPE ms){
        rclc_sleep_ms(ms);
    };


    static int8_t bno_hwi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t r_len){
        // ESP_LOGI(TAG, "bno read called with i2c addr %d reg_addr %d len %d", dev_addr, reg_addr, r_len);
        // if(active && i2c_master_receive(active->dev_handle, reg_data, r_len, -1)){
        if(i2c_master_transmit_receive(active->dev_handle, &reg_addr, 1, reg_data, r_len, -1)){
            // on error
            ESP_LOGE(TAG, "read failed");
            return 1;
        };
        
        // ESP_LOG_BUFFER_HEX("BNO", reg_data, r_len);
        return 0;
    };
    
    static int8_t bno_hwi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t r_len){
        // ESP_LOGI(TAG, "bno write called with i2c addr %d reg_addr %d len %d", dev_addr, reg_addr, r_len);

        uint8_t buffer[r_len + 1] = {0};
        buffer[0] = reg_addr;
        memcpy(buffer+1, reg_data, r_len);

        // if(active && i2c_master_transmit(active->dev_handle, reg_data, r_len, -1)){
        if(i2c_master_transmit(active->dev_handle, buffer, r_len + 1, -1)){
            // on error
            ESP_LOGE(TAG, "bno055 write failed");
            return 1;
        };

        // ESP_LOG_BUFFER_HEX("BNO", reg_data, r_len);
        return 0;
    };


    void fetchData(){
        if(!initSuccessful) return;

        // ESP_LOGI(TAG, "fetching latest data from bno055");
        bno055_quaternion_t quat;
        bno055_accel_t acc;
        if(!bno055_read_quaternion_wxyz(&quat)){

            // bno055_read_euler_hrp(&euler);
            // bno055_read_accel_xyz(&acc);
            data.w = float(quat.w) / 0x3fff;
            data.x = float(quat.x) / 0x3fff;
            data.y = float(quat.y) / 0x3fff;
            data.z = float(quat.z) / 0x3fff;
        }

        ESP_LOGI(TAG, "orientation %3f %3f %3f %3f ", data.x, data.y, data.z, data.w);
    };

    ~bno055(){};

private:


    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    static bno055* active;

    bno055_t bno_config;
    bool initSuccessful = false;
};

bno055* bno055::active = 0;

extern "C" void* overlay_main(){
    return new bno055();
};  