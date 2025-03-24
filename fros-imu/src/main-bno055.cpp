

#include "esp_log.h"
#include "genericImu.hpp"
#include "imuPublisher.hpp"



class dummyImu : public genericImu {

public:

    void init(){};
    void update(){
        data.ax += 0.1;
    };

    ~dummyImu(){};
};


extern "C" void* overlay_main(){
    return new imuPublisher(new dummyImu());
};