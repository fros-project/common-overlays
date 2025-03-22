
#include "velDriver.hpp"
#include "urosHandler.hpp"


velDriver* velDriver::def = 0;


velDriver::velDriver() {
    def = this;
    int pwmPins[] = { cfg.pwms[0], cfg.pwms[1], cfg.pwms[2], cfg.pwms[3]};// LIFT_PWM}; // LIFT_PWM, GRAB_PWM};
    int dirPins[] = { cfg.dirs[0], cfg.dirs[1], cfg.dirs[2], cfg.dirs[3]};;
    handler = new qmd(pwmPins, dirPins, 4);
    handler->setInvertingMode(false);
    handler->setRange(19900, 0);
};

char* drive_additional_constrains = "\n"
                                    "0,diffrential \n"
                                    "1,three wheel omni \n"
                                    "2,mechanum \n";
                                    
void velDriver::declareParameters(){
    urosHandler::addParameter_int("drive type", &cfg.driveType, &cfg, drive_additional_constrains, 0, 2, 1);
    urosHandler::addParameter_int("A PWM", &cfg.pwms[0], &cfg);
    urosHandler::addParameter_int("A DIR", &cfg.dirs[0], &cfg);

    urosHandler::addParameter_int("B PWM", &cfg.pwms[1], &cfg);
    urosHandler::addParameter_int("B DIR", &cfg.dirs[1], &cfg);
    
    if(cfg.driveType >= 1){
        urosHandler::addParameter_int("C PWM", &cfg.pwms[2], &cfg);
        urosHandler::addParameter_int("C DIR", &cfg.dirs[2], &cfg);        
    };
    
    if(cfg.driveType == 2){
        urosHandler::addParameter_int("D PWM", &cfg.pwms[3], &cfg);
        urosHandler::addParameter_int("D DIR", &cfg.dirs[3], &cfg); 
    };

    
};

void velDriver::init()
{
    rclc_subscription_init_default(&sub, node, twistMsgType, "/cmd_vel");
    geometry_msgs__msg__Twist__init(&msgAlloc);
    rclc_executor_add_subscription(exec, &sub, &msgAlloc, cmdVelCallback, ON_NEW_DATA);
};

wheelSpeed velDriver::holonomicMap(float x, float y, float w){
    // invert w to correct for inverted axis
    w = -w;
    x = -x; 

    float si, co, max, theta = atan2(y, x), power = sqrt( x * x + y * y);

    si = sin(theta - M_PI/4);
    co = cos(theta - M_PI/4);
    max = abs(si) > abs(co) ? abs(si) : abs(co);

    wheelSpeed ret;
    // ret.fl = power * co / max + w;
    // ret.fr = power * si / max - w;
    // ret.bl = power * si / max + w;
    // ret.br = power * co / max - w;

    ret.fl = power * co + w;
    ret.fr = power * si - w;
    ret.bl = power * si + w;
    ret.br = power * co - w;


    float overShootMax = 1.0f;
    for(int i = 0; i < 4; i++){
        overShootMax = (abs(ret.rawSpeed[i]) > overShootMax ) ? abs(ret.rawSpeed[i]) : overShootMax;
    }

    for(int i = 0; i < 4; i++){
        ret.rawSpeed[i] /= overShootMax;
    }


    // if((power + abs(w)) > 1){
    //     ret.fl /= power + w;
    //     ret.fr /= power + w;
    //     ret.bl /= power + w;
    //     ret.br /= power + w;
    // };

    return ret;

}


wheelSpeed velDriver::diffrentialMap(float x, float y, float w)
{   
    wheelSpeed ret;

    ret.rawSpeed[0] = ret.rawSpeed[1] = y;
    ret.rawSpeed[0] += w;
    ret.rawSpeed[1] -= w;

    return ret;
}

float omni_a[] = { 1.0f, 0.0f};
float omni_b[] = { cos(M_PI / 6), sin(M_PI / 6)};
float omni_c[] = { sin(M_PI / 6), cos(M_PI / 6)};
wheelSpeed velDriver::omniMap(float x, float y, float w)
{
    wheelSpeed ret;

    ret.rawSpeed[0] = omni_a[0] * x + omni_a[1] * y + w;
    ret.rawSpeed[1] = omni_b[0] * x + omni_b[1] * y + w;
    ret.rawSpeed[2] = omni_c[0] * x + omni_c[1] * y + w;

    return ret;
}

wheelSpeed velDriver::map(float x, float y, float w)
{
    switch (def->cfg.driveType)
    {
        case 0: return diffrentialMap(x, y, w);   break;
        case 1: return omniMap(x, y, w); break;
        case 2: return holonomicMap(x, y, w); break;
    
    default:
        break;
    }
}

void velDriver::cmdVelCallback(const void* msgIn){
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgIn;

    wheelSpeed mapped = map(msg->linear.y, msg->linear.x, msg->angular.z / 2.0f);
    memcpy(def->handler->speeds, &mapped, 4 * sizeof(float));

    def->handler->update();

    ESP_LOGI("CMD_VEL_SUB", " x %f y %f w %f fl %f fr %f bl %f br %f ",
        msg->linear.x,
        msg->linear.y,
        msg->angular.z,
        mapped.fl,
        mapped.fr,
        mapped.bl,
        mapped.br
    );
};