cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(PicoSpiI2cExample C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(PicoSpiI2cExample
		src/main.c
		src/octoquad.c
		)

target_include_directories(PicoSpiI2cExample PUBLIC src src/usb src/util)

target_link_libraries(
		PicoSpiI2cExample
		pico_stdlib
		hardware_i2c
		hardware_spi)

pico_enable_stdio_usb(PicoSpiI2cExample 1)
pico_enable_stdio_uart(PicoSpiI2cExample 0)
pico_add_extra_outputs(PicoSpiI2cExample)

