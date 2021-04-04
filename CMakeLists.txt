# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH /home/om/Projects/pico/pico-sdk)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(marvin_motion C CXX)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(marvin_motion marvin_motion.cpp wheel.cpp)

pico_set_program_name(marvin_motion "marvin_motion")
pico_set_program_version(marvin_motion "0.1")

pico_enable_stdio_usb(marvin_motion 1)
pico_enable_stdio_uart(marvin_motion 0)

# Add any user reuqested libraries
target_link_libraries(marvin_motion
        pico_stdlib
        hardware_dma
        hardware_clocks
        hardware_pwm
        )


pico_add_extra_outputs(marvin_motion)

# add url via pico_set_program_url
#example_auto_set_url(marvin_motion)

