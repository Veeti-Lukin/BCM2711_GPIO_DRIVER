cmake_minimum_required(VERSION 3.16)
project(BCM2711_GPIO_Driver CXX)

add_library(BCM2711_GPIO_Driver
        src/bcm2711_gpio_driver.cpp
        inc/BCM2711_GPIO_Driver/bcm2711_gpio_driver.h
)

target_include_directories(BCM2711_GPIO_Driver PUBLIC inc)



add_executable(test_driver main.cpp)
target_link_libraries(test_driver BCM2711_GPIO_Driver)