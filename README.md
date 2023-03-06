## CAN SDK for C

This is the Canis Labs CAN C SDK. It provides a uniform API for CAN that is portable to different CAN controller hardware, and also to different microcontroller targets.

The first CAN controller for this API is the Microchip MCP25xxFD. This is the CAN controller on the Canis Labs CANPico board. A binding for that board is included.

## Building

Application code should include `canapi.h` for the definitions and relevant driver.

Add to the build for the MCP25xxFD CAN controller:
* `mcp25xxfd/mcp25xxfd.c`

Compilation options:
* Define `MCP25xxFD` to bring in the appropriate driver defintions
* Define `HOST_CANPICO` to build for the Canis Labs CANPico

See the reference manual (`docs/2208-02-RM.pdf`) for more details.

## Hello World example for the CANPico

There is a simple 'hello world' example program for the CANPico, built using the Pico C SDK. To build it, install the pre-requisites (Pico C SDK, Arm cross-compiler) and set PICO_SDK_PATH to where the Pico SDK is installed. Then:

    $ cmake CMakeLists.txt
    $ make

Program the firmware in the file `can_hello.uf2` to the Pico in the normal way. For Pico W support, see CMakeLists.txt. 

More on the CANPico can be found at the Canis Labs web site:

https://canislabs.com/canpico

There is also a blog post on how to debug in C for the Pico using Microsoft Visual Studio Code as an embedded IDE:

https://kentindell.github.io/2022/07/26/canpico-c-debug/

## File structure

The file structure for the SDK is:

    hello_can.c             [Example program for CANPico]
    canapi.h                [API for CAN drivers]
    CMakeFile.txt           [cmake file for CANPico hello world demo]
    docs/                   [Documentation]
        api-cheat-sheet.png     [Cheat sheet for the API]
        2303-02-RM.pdf          [CAN SDK for C reference manual]
    mcp25xxfd/              [Drivers for MCP25xxFD chip]
        mcp25xxfd-types.h       [Definition of target-specific types]
        mcp25xxfd.c             [Implementation of drivers]
        mcp25xxfd-inline.h      [Implementation of some API functions using inline code]
        rp2/
            mcp25xxfd-rp2.h         [Target-specific definitions for RP2040/CANPico]

## MicroPython API

This CAN SDK for C is used by the Canis Labs MicroPython firmware build for the CANPico. See the Canis Labs CANHack repository: https://github.com/kentindell/canhack in `pico/micropython`

## Changes in 2023-03-05 update

- Fixes for silicon errata (now uses CRC-protected reads from controller)
- Bug fixes (CAN ID mapping was incorrect for extended ID frames)
- Added support for multiple instances of a CAN controller (including binding to SPI interface)
- Renamed files with 'mcp2517fd' to 'mcp25xxfd' to better reflect devices supported
- Added support for Pico W in example program
## Future

Roadmap includes:

- Support for the on-chip bxCAN of the STM32 family
- Sleep and wakeup support
- Better FIFO design
- Support for the Bullseye (bullseye.com) CCover code coverage tool
- Support for CAN FD on the MCP25xxFD controllers
