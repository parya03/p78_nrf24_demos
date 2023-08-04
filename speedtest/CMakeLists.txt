cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(p78_nrf_speedtest C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror=implicit")

pico_sdk_init()

add_executable(p78_nrf_speedtest_rpipico
main.c
)

target_compile_definitions(p78_nrf_speedtest_rpipico PRIVATE RPI_PICO)
add_library(libp78_nrf24_rpipico STATIC IMPORTED)
set_property(TARGET libp78_nrf24_rpipico PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../p78_nrf24/build/libp78_nrf24_rpipico.a)
target_link_libraries(p78_nrf_speedtest_rpipico libp78_nrf24_rpipico pico_stdlib hardware_spi hardware_gpio)

# Change if needed
set(libopencm3_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../p78_nrf24/libopencm3")

# LibOpenCM3 Build
# LibOpenCM3 build
add_custom_target(libopencm3 make TARGETS=stm32/f4 WORKING_DIRECTORY ${libopencm3_SOURCE_DIR})
add_library(stm32f429 STATIC IMPORTED)
set_property(TARGET stm32f429 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${libopencm3_SOURCE_DIR}/include)
set_property(TARGET stm32f429 PROPERTY IMPORTED_LOCATION ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32f4.a)
add_dependencies(stm32f429 libopencm3)
target_link_directories(stm32f429 INTERFACE ${libopencm3_SOURCE_DIR}/lib)

target_compile_definitions(stm32f429 INTERFACE -DSTM32F4)

set(STM32F4_COMPILE_OPTIONS 
  --static
  -nostartfiles
  -fno-common
  --specs=nosys.specs
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=hard
  -mfpu=fpv4-sp-d16
)
target_compile_options(stm32f429 INTERFACE ${STM32F4_COMPILE_OPTIONS})
target_link_options(stm32f429 INTERFACE ${STM32F4_COMPILE_OPTIONS})

add_executable(p78_nrf_speedtest_stm32f4
main.c
)

set(LINKER_SCRIPT1 "${libopencm3_SOURCE_DIR}/lib/cortex-m-generic.ld")
set(LINKER_SCRIPT2 "${CMAKE_CURRENT_SOURCE_DIR}/../p78_nrf24/stm32f429i-discovery.ld")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT1} -T ${LINKER_SCRIPT2}")
# set_property(TARGET p78_nrf_speedtest_stm32f4 PROPERTY LINK_OPTIONS "${LINK_OPTIONS} -T ${LINKER_SCRIPT1}")
# set_property(TARGET p78_nrf_speedtest_stm32f4 PROPERTY LINK_OPTIONS "${LINK_OPTIONS} -T ${LINKER_SCRIPT2}")
target_link_options(p78_nrf_speedtest_stm32f4 PRIVATE -T "${LINKER_SCRIPT2}")

target_include_directories(p78_nrf_speedtest_stm32f4 PRIVATE ${libopencm3_SOURCE_DIR}/include)
target_compile_definitions(p78_nrf_speedtest_stm32f4 PRIVATE STM32F4)
add_library(libp78_nrf24_stm32f4 STATIC IMPORTED)
set_property(TARGET libp78_nrf24_stm32f4 PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../p78_nrf24/build/libp78_nrf24_stm32f4.a)
target_link_libraries(p78_nrf_speedtest_stm32f4 libp78_nrf24_stm32f4 stm32f429)

pico_enable_stdio_usb(p78_nrf_speedtest_rpipico 1)
pico_enable_stdio_uart(p78_nrf_speedtest_rpipico 1)
pico_add_extra_outputs(p78_nrf_speedtest_rpipico)