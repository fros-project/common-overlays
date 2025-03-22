// embed-ros - ros2 utilities for embedded systems 
// Copyright (C) 2023  akshay bansod <akshayb@gmx.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef UROS_HANDLER_HPP
#define UROS_HANDLER_HPP

#include <string>
#include <vector>
#include <unordered_map>

#include <driver/gpio.h>


#include <rclc_parameter/rclc_parameter.h>


#include "urosElement.hpp"

#include <sdkconfig.h>


#define MAX_PARAMS 20
#define EMBED_MAX_STRING_LEN 20
#define DEFAULT_SSID "nm-450"
#define DEFAULT_PASSWORD "qwqwqw12"

#define uart_port UART_NUM_0
#define UART_BUFFER_SIZE (512)
#define EMBED_DEFAULT_UART_TXD  (GPIO_NUM_43)
#define EMBED_DEFAULT_UART_RXD  (GPIO_NUM_44)
#define EMBED_DEFAULT_UART_BAUDRATE (460800)

#define EMBED_ROS_DEFAULT_TRANSPORT (urosHandler::config::TRANSPORT_USB)
#define EMBED_ROS_DEFAULT_NODE_NAME "embed_ros"

/**
 * @brief urosHandler to manage micro ros initialization
 */
class urosHandler : public urosElement {

public:

    struct config{
        const char node_name[EMBED_MAX_STRING_LEN] = EMBED_ROS_DEFAULT_NODE_NAME;

        enum transport_mode : int {
            TRANSPORT_USB = 0,
            TRANSPORT_UART, 
            TRANSPORT_WIFI,
        } mode = (transport_mode) (EMBED_ROS_DEFAULT_TRANSPORT);


        // only used when transport is set to TRANSPORT_WIFI
        const char SSID[EMBED_MAX_STRING_LEN] = DEFAULT_SSID;
        const char password[EMBED_MAX_STRING_LEN] = DEFAULT_PASSWORD;

        // only used when transport is set to TRANSPORT_UART
        gpio_num_t uart_tx = EMBED_DEFAULT_UART_TXD, uart_rx = EMBED_DEFAULT_UART_RXD;
        int baud_rate = EMBED_DEFAULT_UART_BAUDRATE;

    } cfg;
    
    const static config defaultConfig ;

    /**
     * @brief Construct a new uros Handler object with given configuration
     * 
     * Transport mode can be changed at runtime by setting appropriate mode 
     */
    urosHandler(const config& cfg = defaultConfig);


    void addElement(urosElement* elem);

    /**
     * @brief create a rclc executor on a new thread   
     * 
     * @param elements vector of urosElement pointer to run on these threaded executor
     * @param CPUID  cpuid to run executor thread 
     */

    // static methods to register a parameter, must be call before starting executor thread
    static void addParameter_bool(char* name, bool* data, urosElement::config* context = 0);
    static void addParameter_int(char* name, int64_t* data, urosElement::config* context = 0, int64_t min = 0, int64_t max = 0, int64_t stepSize = 0);
    static void addParameter_double(char* name, double* data, urosElement::config* context = 0, double min = 0, double max = 0, double stepSize = 0);



    void init()  override ;
    void declareParameters() override ;

    static urosHandler* currentHandler;
    std::vector<urosElement*> wait_queue;

private:

    struct paramMapEntry{
        urosElement::config* context;
        void* data;
    };

    // parameter configuration
    static rclc_parameter_server_t* paramServer;
    static std::unordered_map<std::string, paramMapEntry>  paramMap;


    static void executorTask(void* param);
    static bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context);

    rcl_allocator_t allocator;
    rclc_support_t support;
    rmw_init_options_t* rmw_options;
    rcl_node_t node;
    rclc_executor_t executor;

};

    

#endif // UROS_HANDLER_HPP
