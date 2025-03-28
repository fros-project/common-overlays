cmake_minimum_required(VERSION 3.8)
project(fros-imu)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces)
find_package(action_msgs)
find_package(unique_identifier_msgs)
find_package(rcl_interfaces)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# overlay for bno055
add_executable(fros-imu-bno055 
    src/main-bno055.cpp 
    src/imuPublisher.cpp
    src/storable.cpp
  
    deps/BNO055_SensorAPI/bno055.c

)

target_include_directories(fros-imu-bno055 PUBLIC include deps/BNO055_SensorAPI)
target_link_libraries(fros-imu-bno055 fros std_msgs sensor_msgs geometry_msgs builtin_interfaces action_msgs unique_identifier_msgs rcl_interfaces)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
