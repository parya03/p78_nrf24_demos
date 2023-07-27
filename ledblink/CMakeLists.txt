cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(p78_nrf C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(p78_nrf
main.c
../p78_nrf24/nrf24l01.c
)

pico_enable_stdio_usb(p78_nrf 1)
pico_enable_stdio_uart(p78_nrf 1)
pico_add_extra_outputs(p78_nrf)

target_link_libraries(p78_nrf pico_stdlib hardware_spi hardware_gpio)