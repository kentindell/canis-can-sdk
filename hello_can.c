/**
 * Copyright (c) 2022-2023 Canis Automotive Labs Ltd
 *
 * Simple example of using the Canis Labs CAN SDK to send and receive frames with the MCP25xxFD CAN
 * controller on the Canis Labs CANPico board with a Raspberry Pi Pico.
 * 
 * See the blog post:
 * 
 * https://kentindell.github.io/2022/07/26/canpico-c-debug/ for 
 * 
 * on setting up a picoprobe and VSCode on Ubuntu to perform SWD debugging on a Pico + CANPico
 * 
 * For more on the Canis Labs CANPico see:
 * 
 * https://kentindell.github.io/canpico
 * 
 * To build this for a Pico:
 * 
 * Install the C++/C SDK for the Pico:
 * 
 * https://github.com/raspberrypi/pico-sdk
 * 
 * and set the environment variable PICO_SDK_PATH to point to where it is cloned.
 * 
 * Then build this example with:
 * 
 * $ cmake CMakeFiles.txt
 * $ make
 * 
 * Copy the firmware file hello_can.uf2 to the Pico by powering up the board with boot button
 * held down. Build the firmware either for USB serial (if standalone) or UART serial (if
 * running via a picoprobe debugger). See CMakeLists.txt for details.
 * 
 * The text output of the example can be displayed on a Linux host by connecting to the USB
 * serial port using minicom:
 *
 * $ minicom -b115200 -o -D /dev/ttyACM0
 *
 * Assuming the USB serial port is /dev/ttyACM0 - it might be on a different port, depending
 * on what other serial devices are connected.
 * 
 * NB: to exit minicom, do CTRL-A, then X and select Yes.
 */

#include <stdio.h>
#include "pico/stdlib.h"

// Include the WiFi chip access functions for driving the on-board LED on the Pico W
#ifndef PICO_DEFAULT_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "canapi.h"

#ifndef HELLO_ID
#define HELLO_ID        (0x123U)
#endif

// Utility function to print a CAN frame to stdout
void print_frame(can_frame_t *f, uint32_t timestamp)
{
    uint8_t len = can_frame_get_data_len(f);
    printf(can_frame_is_extended(f) ? "0x%08x " : "0x%03x ", can_frame_get_arbitration_id(f));
    if (can_frame_is_remote(f)) {
        printf("R"); // Remote frame doesn't have any data, len will be zero
    }
    for (uint32_t i = 0; i < len; i++) {
        printf("%02x", can_frame_get_data(f)[i]);
    }
    printf(can_frame_is_fd(f) ? " FD" : " CC");
    if (can_frame_is_brs(f)) {
        printf(" BRS");
    }
    if (can_frame_is_esi(f)) {
        printf(" ESI");
    }   
    printf(" DLC=%d ", can_frame_get_dlc(f));
    printf(" (%d)\n", timestamp);
}

static can_controller_t controller; 

#include <stdio.h>
#include "pico/stdlib.h"

// Wrapper to parameterize the API's IRQ handler with the handle to the single CAN
// controller on the CANPico (the structure is defined above and configured by
// binding to the SPI port and pins used on the CANPico).
//
// NB: The RP2040 with execute-in-place (XIP) flash suffers from very long cache
// miss delays, and these can cause significant delays to the interrupt handler.
// Generally the entire chain of interrupt handling (from vector table to first-level
// handler to device-specific handlers) should be located in RAM to avoid these
// cache delays. The CAN drivers are allocated to RAM with the "TIME_CRITICAL"
// attribute that causes the compiler to place the function in RAM.
//
// This problem of delaying interrupts also extends to critical sections: code that
// disables an interrupt around some function also should execute from RAM while in
// that function to avoid delaying an urgent interrupt handler. This "priority
// inversion" is discussed further here:
//
// https://kentindell.github.io/2021/03/05/pico-priority-inversion/
void TIME_CRITICAL irq_handler(void)
{
    // Work out if this interrupt is from the the MCP25xxFD. The bound interface
    // defines the pin used for the interrupt line from the CAN controller.
    uint8_t spi_irq = controller.host_interface.spi_irq;
    uint32_t events = gpio_get_irq_event_mask(spi_irq); 

    if (events & GPIO_IRQ_LEVEL_LOW) {
        mcp25xxfd_irq_handler(&controller);
    }
}

void led_init(void)
{
#ifndef PICO_DEFAULT_LED_PIN
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
    }
#else
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

void led_on(void)
{
#ifndef PICO_DEFAULT_LED_PIN
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#else
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
}

void led_off(void)
{
#ifndef PICO_DEFAULT_LED_PIN
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
#else
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
}

int main() {
    stdio_init_all();
    led_init();

    // This Pico SDK call binds the GPIO vector to calling the CAN controller ISR. If there are
    // other devices connected to GPIO interrupts (e.g. the WiFi chip on the Pico W)
    // then the appropriate handler has to be called.
    irq_add_shared_handler(IO_IRQ_BANK0, irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    can_errorcode_t rc;

    // Example uses 500Kbit/sec, 2Mbit/sec FD higher speed rate
    can_bitrate_t bitrate = {.profile = CAN_BITRATE_FD_500K_2M}; 

    // Bind the Pico SPI interface to the CANPico's pin layout
    mcp25xxfd_spi_bind_canpico(&controller.host_interface);

    // Set up the CAN controller on the CANPico using the bound SPI interface
    while (true) {
        rc = can_setup_controller(&controller, &bitrate, CAN_NO_FILTERS, CAN_MODE_NORMAL_FD, CAN_OPTIONS_NONE);
        if (rc != CAN_ERC_NO_ERROR) {
            // This can fail if the CAN transceiver isn't powered up properly. That might happen
            // if the board had 3.3V but not 5V (the transceiver needs 5V to operate). 
            printf("CAN setup error: %d\n", rc);
            // Try again after 1 second
            sleep_ms(1000);
        }
        else {
            break;
        }
    }

    // Create a CAN frame with 11-bit ID of (default 0x123) and 64 byte payload
    uint8_t data[64] = {
        0x54, 0x68, 0x65, 0x79, 0x20, 0x6d, 0x61, 0x79,
        0x20, 0x74, 0x61, 0x6b, 0x65, 0x20, 0x6f, 0x75,
        0x72, 0x20, 0x6c, 0x69, 0x76, 0x65, 0x73, 0x2c,
        0x20, 0x62, 0x75, 0x74, 0x20, 0x74, 0x68, 0x65,
        0x79, 0x20, 0x77, 0x69, 0x6c, 0x6c, 0x20, 0x6e,
        0x65, 0x76, 0x65, 0x72, 0x20, 0x74, 0x61, 0x6b,
        0x65, 0x20, 0x6f, 0x75, 0x72, 0x20, 0x66, 0x72,
        0x65, 0x65, 0x64, 0x6f, 0x6d, 0x21, 0x21, 0x21,
    };

    can_frame_t my_tx_frame;
    // DLC 15 = 64 bytes for an FD frame, set FDF (=CAN FD) and BRS (=Switch baud rate) flags
    can_make_frame(&my_tx_frame, false, HELLO_ID, 15U, data, CAN_FRAME_FLAG_FDF | CAN_FRAME_FLAG_BRS);

    uint32_t queued_ok = 0;

    while (true) {
        // Light on
        led_on();

#ifndef LISTEN_ONLY
        // Send our frame
        rc = can_send_frame(&controller, &my_tx_frame, false);
        if (rc != CAN_ERC_NO_ERROR) {
            // This can happen if there is no room in the transmit queue, which can
            // happen if the CAN controller is connected to a CAN bus but there are no
            // other CAN controllers connected and able to ACK a CAN frame, so the
            // transmit queue fills up and then cannot accept any more frames.
            printf("CAN send error: %d, sent=%d\n", rc, queued_ok);
        }
        else {
            queued_ok++;
        }
        can_frame_get_data(&my_tx_frame)[63]++; // Update last byte of frame payload

        printf("Frames queued OK=%d\n", queued_ok);
#endif

        // Wait
        sleep_ms(250);
        // Light off
        led_off();
        // Wait
        sleep_ms(250);

        // Print up to 10 received frames
        can_rx_event_t rx_event;
        uint32_t n = 0;
        while (n < 10U) {
            can_rx_event_t *e = &rx_event;
            if (can_recv(&controller, e) && can_event_is_frame(e)) {
                can_frame_t *f = can_event_get_frame(e);
                print_frame(f, e->timestamp);
                n++;
            }
            else {
                break; // No events left to check for this loop
            }
        }
    }
}
