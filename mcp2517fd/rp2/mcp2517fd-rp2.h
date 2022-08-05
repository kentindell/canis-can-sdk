// Copyright 2020-2022 Canis Automotive Labs (canislabs.com)
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

#ifndef MCP2517FD_RP2_H
#define MCP2517FD_RP2_H

// Binds the drivers to the CANPico hardware

#if defined(HOST_CANPICO)

// These are the physical pin mappings for the CANPico. Other boards using the RP2040
// and the MCP2517FD could be on different pins.
#define XSTBY_GPIO                          (3U)            // Transceiver standby pin

// SPI controller and pins to use
#define SPI_DEVICE                          (spi1)
#define SPI_Rx                              (8U)
#define SPI_SCK                             (10U)
#define SPI_Tx                              (11U)
#define SPI_IRQ                             (5U)            // Interrupt request
#define SPI_CS                              (6U)            // Chip select
#else
#error "Unknown board"
#endif

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// Code on the RP2040 goes into RAM if it's time critical: the XIP flash is way too slow
#ifndef TIME_CRITICAL
#define TIME_CRITICAL                       __attribute__((noinline, long_call, section(".time_critical")))
#endif

#ifndef WEAK
#define WEAK                                __attribute__((weak))
#endif 

#ifndef INLINE
#define INLINE                              static inline
#endif

#define LEVEL_SENSITIVE_LOW                 (1U)
#define EDGE_SENSITIVE_RISING               (1U << 3)
#define SPI_GPIO_IRQ_PRIORITY               (1U << 6)       // Default IRQ priority is 0x80 (i.e. 2, where 0 is the highest and 3 is the lowest).

static inline void mcp2517fd_spi_gpio_enable_irq(void)
{
    // If the GPIO interrupts are shared (i.e. another device is connected to GPIO interrupts) then
    // disable them by disabling interrupts on the pin, so that the other devices can continue to
    // handle interrupts through critical sections where the SPI is being accessed.
    gpio_set_irq_enabled(SPI_IRQ, LEVEL_SENSITIVE_LOW, true);
}

static inline void mcp2517fd_spi_gpio_disable_irq(void)
{
    gpio_set_irq_enabled(SPI_IRQ, LEVEL_SENSITIVE_LOW, false);
}

static inline void mcp2517fd_spi_select(void)
{
    sio_hw->gpio_clr = (1U << SPI_CS);
}

static inline void mcp2517fd_spi_deselect(void)
{
    sio_hw->gpio_set = (1U << SPI_CS);
}

static inline void mcp2517fd_spi_write(const uint8_t *src, size_t len)
{
    spi_write_blocking(SPI_DEVICE, src, len);
}

static inline void mcp2517fd_spi_read_write(const uint8_t *cmd, uint8_t *resp, size_t len)
{
    spi_write_read_blocking(SPI_DEVICE, cmd, resp, len);
}

static inline void mcp2517fd_spi_read(uint8_t *dst, size_t len)
{
    spi_read_blocking(SPI_DEVICE, 0xaa, dst, len);
}

static inline void mcp2517fd_spi_pins_init(void) {
    // These are the defaults anyway:
    //    spi_set_format(spi1, 8U, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // CANPico board is clocked at 40MHz, so SPI must be no more than 18500000 (according to data sheet)
    spi_init(SPI_DEVICE, 18500000);
    gpio_set_function(SPI_Rx, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_Tx, GPIO_FUNC_SPI);

#ifdef HOST_CANPICO
    // This isn't strictly part of the SPI driver but the pin needs to be set to
    // enable the CAN transceiver on the CANPico board.
    // Set XSTBY pin to software controlled
    gpio_set_function(XSTBY_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(XSTBY_GPIO, GPIO_OUT);
    // Set the XSTBY pin to 0 to enable the transceiver
    sio_hw->gpio_clr = (1U << XSTBY_GPIO);
#endif

    // Set the chip select pin for the MCP2517FD as a GPIO port
    gpio_set_function(SPI_CS, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(SPI_CS, GPIO_OUT);
    // Deselect the MCP2517FD for now
    gpio_set_mask(1U << SPI_CS);

    // Now enable GPIO interrupts (there may be other interrupt sources set up in the GPIO interrupt channel
    // and this may already be enabled)
    irq_set_enabled(IO_IRQ_BANK0, true);
}

#endif // MCP2517FD_RP2_H
