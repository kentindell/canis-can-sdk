# Simple Hello World CAN program using the CAN SDK
#
# Runs on the Raspberry Pi Pico connected to a Canis Labs CANPico board
# (see https://kentindell.github.io/canpico for more about the CANPico)
#
# If the Pico W is used with the CANPico board then set the PICO_BOARD variable
# appropriately (see below) or invoke cmake with PICO_BOARD=pico_w
#
# Running the example requires a functional CAN bus with at least one other
# active CAN controller that will set the acknowledge bit of the CAN frame
# (the ideal companion is a CANPico with the MicroPython CAN API)
#
# The program sends a 5-byte CAN frame every half second, and displays any
# incoming CAN frames.
#
# Building this example requires that the Raspberry Pi Pico C SDK is installed and
# the environment variable PICO_SDK_PATH is set to point to where it is installed.
# The Pico C SDK can be cloned from:
#
# https://github.com/raspberrypi/pico-sdk
#
# And documentation can be downloaded from:
#
# https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
#
# The text output of the example can be displayed on a Linux host by connecting to the USB
# serial port using minicom:
#
# $ minicom -b115200 -o -D /dev/ttyACM0
#
# (Assuming the USB serial port is /dev/ttyACM0 - it might be on a different port, depending
# on what other serial devices are connected)

# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Defaults to the Pico board (a value of 'pico'). Uncomment the following line if using an Pico W.
# set(PICO_BOARD pico_w)
set(PICO_BOARD pico)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(hello_can C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# These are the definitions to select the MCP25xxFD on the CANPico and
# the binding to the RP2040 and CANPico pinouts.
add_definitions(-DMCP25xxFD -DHOST_CANPICO)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    hello_can.c
    mcp25xxfd/mcp25xxfd.c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# Additional library for Pico W to access the LED via the WiFi chip
# (See https://github.com/raspberrypi/pico-sdk/releases for a list of other libraries)
if (${PICO_BOARD} MATCHES "pico_w") 
    target_link_libraries(${PROJECT_NAME} 
        pico_cyw43_arch_none
)
endif()


# For standalone execution, set USB to 1 and UART to 0 so that the output can be
# displayed over the USB serial port. For use with a picoprobe SWD debugger, use
# UART to 1 and USB to 0: the text will appear over USB serial but from the picoprobe
# pico.
#
# See the blog post:
#
# https://kentindell.github.io/2022/07/26/canpico-c-debug/
#
# for how to set up a picoprobe debugging system for C development on the Pico.
#
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
