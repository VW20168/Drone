# mkdir build
# cd build
# cmake  -DCMAKE_BUILD_TYPE=Release ..
# make

cmake_minimum_required(VERSION 3.16)
project(drone C CXX)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
else()
  message(FATAL_ERROR "OpenCV not found")
endif()
pkg_check_modules(LIBCAMERA REQUIRED libcamera)


SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -isystem/usr/include/glib-2.0 -isystem/usr/lib/aarch64-linux-gnu/glib-2.0/include -isystem/usr/lib/arm-linux-gnueabihf/glib-2.0/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -isystem/usr/include/glib-2.0 -isystem/usr/lib/aarch64-linux-gnu/glib-2.0/include -isystem/usr/lib/arm-linux-gnueabihf/glib-2.0/include -isystem/usr/include/libcamera")

add_executable(drone main.cpp
                    altimeter.cpp
                    altpid.cpp 
                    battery.cpp
                    mpu.cpp 
                    imu.cpp 
                    motor.cpp 
                    pid.cpp 
                    autopilot.cpp 
                    configuration.cpp 
                    ina219.cpp 
                    mpu9250.cpp 
                    alt.cpp
                    flightcontroller.cpp 
                    ms5611.cpp
                    vl53l1x.cpp
                    neo6m.cpp 
                    utils.cpp
                    testrunner.cpp
                    pii2c.cpp
                    pipwm.cpp
                    pipin.cpp
                    bt/adapter.c
                    bt/agent.c
                    bt/characteristic.c	
                    bt/device.c
                    bt/parser.c
                    bt/utility.c
                    bt/advertisement.c
                    bt/application.c
                    bt/descriptor.c
                    bt/logger.c
                    bt/service.c
                    cv/vision.cpp
                    cv/yolov8_utils.cpp
                    cv/yolov8.cpp
                    lccv/lccv.cpp
                    lccv/libcamera_app_options.cpp
                    lccv/libcamera_app.cpp)

target_link_libraries(drone gpiod glib-2.0 gio-2.0 rt ${LIBCAMERA_LINK_LIBRARIES} ${OpenCV_LIBS})

