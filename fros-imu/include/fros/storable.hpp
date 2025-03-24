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


#ifndef STORABLE_HPP
#define STORABLE_HPP


#include <cstring>
#include <nvs_flash.h>


#define STORABLE_ID_SIZE 12
#define NVS_PARTITION_ID "storable"

typedef uint32_t nvs_handle_t;

/**
 * @brief any subclasses of the storable class could be stored / loaded
 * from nvs. A copy of the entire object memory is stored in nvs.
 * 
 * extend this class and pass sub-class name and sub-class type to following constructor 
 */
struct storable
{

public:

    /**
     * @brief Construct a new storable object
     * 
     * @param id string id, must be similar to class name
     * @param size pass sizeof(subclass) here
     */
    storable(const char* id, size_t size);

    // write current state of the object to nvs    
    bool write();

    // update object from state copy in nvs    
    bool load();

    // clears only storable data, other data persists
    static void clear();

    // clears complete nvs data, including any external data stored
    static void clearNvs();

    static bool initNvs();


private:
    
    size_t size = sizeof(storable);
    char id[STORABLE_ID_SIZE] = {0};

    static bool isNvsInit;
    static nvs_handle_t nvsHandle;

};

#endif //  STORABLE_HPP