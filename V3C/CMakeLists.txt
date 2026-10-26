# mkdir build
# cd build
# cmake  -DCMAKE_BUILD_TYPE=Release ..
# make

cmake_minimum_required(VERSION 3.16)
project(drone C CXX)

find_package(Threads)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -isystem/usr/include/glib-2.0 -isystem/usr/lib/aarch64-linux-gnu/glib-2.0/include -isystem/usr/lib/arm-linux-gnueabihf/glib-2.0/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -isystem/usr/include/glib-2.0 -isystem/usr/lib/aarch64-linux-gnu/glib-2.0/include -isystem/usr/lib/arm-linux-gnueabihf/glib-2.0/include")

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
                    bt/service.c)

target_link_libraries(drone pigpio glib-2.0 gio-2.0 rt)

