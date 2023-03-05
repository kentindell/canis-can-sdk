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
 * (Assuming the USB serial port is /dev/ttyACM0 - it might be on a different port, depending
 * on what other serial devices are connected)
 */

#include <stdio.h>
#include "pico/stdlib.h"

// Include the WiFi chip access functions for driving the on-board LED on the Pico W
#ifndef PICO_DEFAULT_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "canapi.h"

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
    printf(" (%d)\n", timestamp);
}

static can_controller_t controller; 

#include <stdio.h>
#include "pico/stdlib.h"

// Wrapper to parameterize the API's IRQ handler with the handle to the single CAN
// controller on the CANPico (the structure is defined above and configured by
// binding to the SPI port and pins used on the CANPico).
void irq_handler(void)
{
    mcp25xxfd_irq_handler(&controller);
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
    // then the code will have to work out which interrupts are pending and call the
    // appropriate handlers.
    irq_set_exclusive_handler(IO_IRQ_BANK0, irq_handler);

    can_errorcode_t rc;

    // Example uses 500Kbit/sec, 75% sample point
    can_bitrate_t bitrate = {.profile = CAN_BITRATE_500K_75}; 

    // Bind the Pico SPI interface to the CANPico's pin layout
    mcp25xxfd_spi_bind_canpico(&controller.host_interface);

    // Set up the CAN controller on the CANPico using the bound SPI interface
    while (true) {
        rc = can_setup_controller(&controller, &bitrate, CAN_NO_FILTERS, CAN_MODE_NORMAL, CAN_OPTIONS_NONE);
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

    // Create a CAN frame with 11-bit ID of 0x123 and 5 byte payload of deadbeef00
    uint8_t data[5] = {0xdeU, 0xadU, 0xbeU, 0xefU, 0x00U};

    can_frame_t my_tx_frame;
    can_make_frame(&my_tx_frame, false, 0x123, sizeof(data), data, false);

    uint32_t queued_ok = 0;

    while (true) {
        // Light on
        led_on();

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
        can_frame_get_data(&my_tx_frame)[4]++; // Update last byte of frame payload

        printf("Frames queued OK=%d\n", queued_ok);

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
