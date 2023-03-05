// Copyright 2020-2023 Canis Automotive Labs (canislabs.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MCP25xxFD_RP2_H
#define MCP25xxFD_RP2_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// Target-specific type defining the interface to the CAN controller
// This is SPI for the MCP25xxFD (and MCP251863) and on the RP2040 there are
// certain parameters for the SPI. These are included in this structure.

typedef struct {
    spi_inst_t *spi_device;                 // Set to SPI_DEVICE
    uint32_t crc_errors;                    // Count of CRC errors
    uint32_t magic;                         // Magic number to indicate the interface is set
    uint8_t spi_rx;                         // MISO pin
    uint8_t spi_tx;                         // MOSI pin
    uint8_t spi_sck;                        // SPI clock
    uint8_t spi_irq;                        // IRQ pin
    uint8_t spi_cs;                         // SPI device chip select
} can_interface_t;

// Binds the drivers to the CANPico hardware
#if defined(HOST_CANPICO)
// These are the physical pin mappings for the CANPico. Other boards using the RP2040
// and the MCP25xxFD could be on different pins.
#define XSTBY_GPIO                          (3U)            // Transceiver standby pin

// SPI controller and pins to use
#define SPI_DEVICE                          (spi1)
#define SPI_Rx                              (8U)
#define SPI_SCK                             (10U)
#define SPI_Tx                              (11U)
#define SPI_IRQ                             (5U)            // Interrupt request
#define SPI_CS                              (6U)            // Chip select

// This binds an SPI interface for a specific board. For other devices this will change.
// For multiple devices on the same SPI channel, spi_cs and spi_irq will be different
// but the other fields the same.
static inline void mcp25xxfd_spi_bind_canpico(can_interface_t *interface)
{
    interface->spi_device = SPI_DEVICE;
    interface->spi_rx = SPI_Rx;
    interface->spi_tx = SPI_Tx;
    interface->spi_sck = SPI_SCK;
    interface->spi_irq = SPI_IRQ;
    interface->spi_cs = SPI_CS;
    interface->magic = 0x1e5515f0U;
}

#else
// Other boards might have different SPI controllers, pins, etc.
#error "Unknown board"
#endif

// Code on the RP2040 goes into RAM if it's time critical: the XIP flash is way too slow
#ifndef TIME_CRITICAL
#define TIME_CRITICAL                       __attribute__((noinline, long_call, section(".time_critical")))
#endif

#ifndef CONST_STORAGE
#define CONST_STORAGE                       /* Tables in RAM */
#endif

#ifndef WEAK
#define WEAK                                __attribute__((weak))
#endif 

#ifndef INLINE
#define INLINE                              static inline
#endif

// The MCP25xxFD is configured to have level sensitive interrupts (so will re-raise
// when another interrupt source continues to assert)

#define LEVEL_SENSITIVE_LOW                 (1U)
#define EDGE_SENSITIVE_RISING               (1U << 3)
#define SPI_GPIO_IRQ_PRIORITY               (1U << 6)       // Default IRQ priority is 0x80 (i.e. 2, where 0 is the highest and 3 is the lowest).

// This is called to convert 4 bytes in memory to 32-bits where the lowest address byte is
// at bits 7:0 of the word, which will then be transmitted to the MCP25xxFD in little endian
// format. For a little-endian CPU, this is already in the right format.
static inline uint32_t mcp25xxfd_convert_bytes(uint32_t w)
{
    return w;
}

static inline void mcp25xxfd_spi_gpio_enable_irq(can_interface_t *interface)
{
    // If the GPIO interrupts are shared (i.e. another device is connected to GPIO interrupts) then
    // disable them by disabling interrupts on the pin, so that the other devices can continue to
    // handle interrupts through critical sections where the SPI is being accessed.
    gpio_set_irq_enabled(interface->spi_irq, LEVEL_SENSITIVE_LOW, true);
}

static inline bool mcp25xxfd_spi_gpio_irq_asserted(can_interface_t *interface)
{
    // Reads the value of the interrupt pin, returns true if the interrupt pin is asserted

    // If the GPIO interrupts are shared (i.e. another device is connected to GPIO interrupts) then
    // disable them by disabling interrupts on the pin, so that the other devices can continue to
    // handle interrupts through critical sections where the SPI is being accessed.
    if (gpio_get_irq_event_mask(interface->spi_irq) & GPIO_IRQ_LEVEL_LOW) {
        return true;
    }
    else {
        return false;
    }
}

static inline void mcp25xxfd_spi_gpio_disable_irq(can_interface_t *interface)
{
    gpio_set_irq_enabled(interface->spi_irq, LEVEL_SENSITIVE_LOW, false);
}

static inline void mcp25xxfd_spi_select(can_interface_t *interface)
{
    sio_hw->gpio_clr = (1U << interface->spi_cs);
}

static inline void mcp25xxfd_spi_deselect(can_interface_t *interface)
{
    sio_hw->gpio_set = (1U << interface->spi_cs);
}

static inline void mcp25xxfd_spi_write(can_interface_t *interface, const uint8_t *src, size_t len)
{
    spi_write_blocking(interface->spi_device, src, len);
}

static inline void mcp25xxfd_spi_read_write(can_interface_t *interface, const uint8_t *cmd, uint8_t *resp, size_t len)
{
    spi_write_read_blocking(interface->spi_device, cmd, resp, len);
}

static inline void mcp25xxfd_spi_read(can_interface_t *interface, uint8_t *dst, size_t len)
{
    spi_read_blocking(interface->spi_device, 0xaa, dst, len);
}

static inline void mcp25xxfd_spi_pins_init(can_interface_t *interface) {
    // These are the defaults anyway:
    //    spi_set_format(spi1, 8U, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    //
    // CANPico board is clocked at 40MHz, so SPI must be no more than 17000000: FSCK must
    // be less than or equal to 0.85 * (FSYSCLK/2). Other boards may use different clocks for
    // the CAN controller and this should be reflected here.
    spi_init(interface->spi_device, 17000000);
    gpio_set_function(interface->spi_rx, GPIO_FUNC_SPI);
    gpio_set_function(interface->spi_sck, GPIO_FUNC_SPI);
    gpio_set_function(interface->spi_tx, GPIO_FUNC_SPI);

#ifdef HOST_CANPICO
    // This isn't strictly part of the SPI driver but the pin needs to be set to
    // enable the CAN transceiver on the CANPico board. Other boards may require
    // different settings.
    // Set XSTBY pin to software controlled
    gpio_set_function(XSTBY_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(XSTBY_GPIO, GPIO_OUT);
    // Set the XSTBY pin to 0 to enable the transceiver
    sio_hw->gpio_clr = (1U << XSTBY_GPIO);
#endif

    // Set the chip select pin for the MCP25xxFD as a GPIO port
    gpio_set_function(interface->spi_cs, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(interface->spi_cs, GPIO_OUT);
    // Deselect the MCP25xxFD for now
    gpio_set_mask(1U << interface->spi_cs);

    // Now enable GPIO interrupts (there may be other interrupt sources set up in the GPIO interrupt channel
    // and this may already be enabled)
    irq_set_enabled(IO_IRQ_BANK0, true);
}

#endif // MCP25xxFD_RP2_H
