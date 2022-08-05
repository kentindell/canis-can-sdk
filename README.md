## CAN SDK for C

This is the Canis Labs CAN C SDK. It provides a uniform API for CAN that is portable to different CAN controller hardware, and also to different microcontroller targets.

The first CAN controller for this API is the Microchip MCP2517FD (and Microchip MCP2518FD). This is the CAN controller on the Canis Labs CANPico board. A binding for that board is included.

## Building

Application code should include `canapi.h` for the definitions and relevant driver.

Add to the build for the MCP2517FD CAN controller:
* `mcp2517fd/mcp2517fd.c`

Compilation options:
* Define `MCP2517FD` to bring in the appropriate driver defintions
* Define `HOST_CANPICO` to build for the Canis Labs CANPico

See the reference manual (`docs/2208-01-RM.pdf`) for more details.

## Hello World example for the CANPico

There is a simple 'hello world' example program for the CANPico, built using the Pico C SDK. To build it, install the pre-requisites (Pico C SDK, Arm cross-compiler) and set PICO_SDK_PATH to where the Pico SDK is installed. Then:

    $ cmake CMakeLists.txt
    $ make

Program the firmware in the file `can_hello.uf2` to the Pico in the normal way.

More on the CANPico can be found at the Canis Labs CTO blog:

https://kentindell.github.io/canpico

There is also a blog post on how to debug in C for the Pico using Microsoft Visual Studio Code as an embedded IDE:

https://kentindell.github.io/2022/07/26/canpico-c-debug/

## File structure

The file structure for the SDK is:

    hello_can.c             [Example program for CANPico]
    canapi.h                [API for CAN drivers]
    CMakeFile.txt           [cmake file for CANPico hello world demo]
    docs/                   [Documentation]
        api-cheat-sheet.png     [Cheat sheet for the API]
        2208-01-RM.pdf          [CAN SDK for C reference manual]
    mcp2517fd/              [Drivers for MCP2517FD chip]
        mcp2517fd-types.h       [Definition of target-specific types]
        mcp2517fd.c             [Implementation of drivers]
        mcp2517fd-inline.h      [Implementation of some API functions using inline code]
        rp2/
            mcp2517fd-rp2.h         [Target-specific definitions for RP2040/CANPico]

## MicroPython API

This CAN SDK for C is used by the Canis Labs MicroPython firmware build for the CANPico. See the Canis Labs CANHack repository: https://github.com/kentindell/canhack in `pico/micropython`

## Future

Roadmap includes:

- Support for the on-chip bxCAN of the STM32 family
- Sleep and wakeup support
- Support for multiple CAN controllers on the same host
- Better FIFO design
- Support for the Bullseye (bullseye.com) CCover code coverage tool
