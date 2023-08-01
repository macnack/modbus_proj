
cmake_minimum_required(VERSION 3.0)

project(modbus_proj)

include(modbus.cmake)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()
if(MSVC)
    # /W4 flag enables level 4 warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
endif()
file(GLOB SOURCES main.cpp posital_encoder_modbus.cpp posital_encoder_modbus.hpp)
add_executable(main ${SOURCES})
target_link_directories(main PUBLIC include)
target_link_libraries(main PUBLIC modbus)