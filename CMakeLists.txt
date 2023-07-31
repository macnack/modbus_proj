
cmake_minimum_required(VERSION 3.0)

project(modbus_proj)

include(modbus.cmake)

file(GLOB SOURCES main.cpp posital_encoder_modbus.cpp posital_encoder_modbus.hpp)
add_executable(main ${SOURCES})
target_link_directories(main PUBLIC include)
target_link_libraries(main PUBLIC modbus)