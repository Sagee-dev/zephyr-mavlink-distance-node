cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
PROJECT(fake_sensor_node)

# include MAVLink headers
target_include_directories(app PRIVATE
    c_library_v2/
)

target_sources(app PRIVATE 
src/main.c
src/uartLayer.c
src/mavlinkLayer.c
)
