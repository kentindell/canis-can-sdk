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

// Drivers for the MCP2517FD and MCP2518FD CAN controllers from Microchip.
//
// See: https://www.microchip.com/en-us/product/MCP2517FD
//
// The MCP2518FD is a newer version of the MCP2517FD but the drivers are compatible
// with both variants. The device is an external CAN controller connected via SPI.
// This file and its header file mcp2517fd.h is the generic C device driver that
// is portable to any microcontroller.
//
// This driver is bound to a specific platform via target-specific SPI driver functions,
// as defined in the API.
//
// TODO speed up ISRs: calculate buffer addr by shadowing TEF and RX FIFO rather than
//      use an SPI transaction to pick it up
// TODO parameterize the mode request call to allow a timeout for requesting the controller
//      go into a specific mode (the CAN state machine goes into bus integration and 
//      comes out in the requested mode only after that completes successfully)
// TODO faster FIFO implementation using power-of-two masks on index values
// TODO add low-power standby mode to API (put controller into standby, put transceiver into standby via XSTBY pin)
// TODO add support for multiple transmit FIFOs feeding into the priority queue

#include <stdbool.h>

void debug_printf( const char *format, ... );

// Brings in API, chip-specific and board-specific definitions
#include "../canapi.h"

// Number of bytes used to store a receive event (CAN frame, error, etc.)
#define NUM_RX_EVENT_BYTES                  (19U)
// Number of bytes used to store a transmission event (CAN frame sent, etc.)
#define NUM_TX_EVENT_BYTES                  (9U)

// Write a 32-bit word in big endian format to a buffer
#define WRITE_BIG_ENDIAN(buf, word)         ((buf)[0] = (uint8_t)(((word) >> 24) & 0xffU),  \
                                            (buf)[1] = (uint8_t)(((word) >> 16) & 0xffU),  \
                                            (buf)[2] = (uint8_t)(((word) >> 8) & 0xffU),  \
                                            (buf)[3] = (uint8_t)((word) & 0xffU))

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// MCP2517FD SPI commands /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Support functions to convert controller register read/write operations into the MCP2517FD's SPI commands

// RP2040 is little-endian, SPI reads the words in little endian format (first byte is lowest bits)
static void TIME_CRITICAL write_word(uint32_t addr, uint32_t word)
{
    // Must be called with interrupts locked

    uint8_t buf[6];
    // MCP2517FD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x20 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;
    buf[2] = word & 0xffU;
    buf[3] = (word >> 8) & 0xffU;
    buf[4] = (word >> 16) & 0xffU;
    buf[5] = (word >> 24) & 0xffU;

    // SPI transaction
    // The Pico is little-endian so the first byte sent is the lowest-address, which is the
    // same as the RP2040
    mcp2517fd_spi_select();
    mcp2517fd_spi_write(buf, sizeof(buf));
    mcp2517fd_spi_deselect();
}

static void TIME_CRITICAL write_4words(uint16_t addr, const uint32_t words[])
{
    // Must be called with interrupts locked

    // Prepare a contiguous buffer for the command because the SPI hardware is pipelined and do not want to stop
    // to switch buffers
    uint8_t cmd[18];
    // MCP2517/18FD SPI transaction = command/addr, 4 bytes
    cmd[0] = 0x20 | ((addr >> 8U) & 0xfU);
    cmd[1] = addr & 0xffU;

    uint32_t i = 2U;
    for (uint32_t j = 0; j < 4U; j++) {
        cmd[i++] = words[j] & 0xffU;
        cmd[i++] = (words[j] >> 8) & 0xffU;
        cmd[i++] = (words[j] >> 16) & 0xffU;
        cmd[i++] = (words[j] >> 24) & 0xffU;
    }

    // SPI transaction
    mcp2517fd_spi_select();
    mcp2517fd_spi_write(cmd, sizeof(cmd));
    mcp2517fd_spi_deselect();
}

static uint32_t TIME_CRITICAL read_word(uint16_t addr)
{
    // Must be called with interrupts locked

    uint8_t cmd[6];
    uint8_t resp[6];

    cmd[0] = 0x30 | ((addr >> 8U) & 0xfU);
    cmd[1] = addr & 0xffU;
    // TODO can remove the following because not strictly necessary (but useful for debugging with a logic analyzer)
    cmd[2] = 0xdeU;
    cmd[3] = 0xadU;
    cmd[4] = 0xbeU;
    cmd[5] = 0xefU;

    // SPI transaction
    mcp2517fd_spi_select();
    mcp2517fd_spi_read_write(cmd, resp, sizeof(cmd));
    mcp2517fd_spi_deselect();

    uint32_t word = ((uint32_t)resp[2]) | ((uint32_t)resp[3] << 8) | ((uint32_t)resp[4] << 16) | ((uint32_t)resp[5] << 24);
    return word;
}

static void TIME_CRITICAL read_words(uint16_t addr, uint32_t *words, uint32_t n)
{
    // Must be called with interrupts locked

    uint8_t buf[2];

    // MCP2517/18FD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x30 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;

    // SPI transaction
    mcp2517fd_spi_select();
    // Send command, which flushes the pipeline then resumes
    mcp2517fd_spi_write(buf, 2U);
    // Bulk data
    mcp2517fd_spi_read((uint8_t *)(words), 4U * n);
    mcp2517fd_spi_deselect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Weak default user hooks ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void WEAK TIME_CRITICAL WEAK can_isr_callback_frame_rx(can_frame_t *frame, uint32_t timestamp)
{
    // Null version as a default if the application doesn't provide a callback handler
}

void WEAK TIME_CRITICAL can_isr_callback_error(can_error_t error, uint32_t timestamp)
{
    // Null version as a default if the application doesn't provide a callback handler
}

void WEAK TIME_CRITICAL can_isr_callback_frame_tx(can_uref_t uref, uint32_t timestamp)
{
    // Null version as a default if the application doesn't provide a callback handler
}

// Defined to allow a default null user reference value
can_uref_t WEAK can_uref_null = {.ref = 0};

// This is only called if the as_bytes option is selected for receiving transmit events
uint32_t WEAK TIME_CRITICAL can_isr_callback_uref(can_uref_t uref)
{
    // Null version as a default if the application doesn't provide a callback handler
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Start of MCP2517FD drivers //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

#define         OSC             (0xe00U)
#define         IOCON           (0xe04U)
#define             INTOD           (1U << 30)
#define             SOF             (1U << 29)
#define             TXCANOD         (1U << 28)
#define             PM1             (1U << 25)
#define             PM0             (1U << 24)
#define             TRIS1           (1U << 1)
#define             TRIS0           (1U << 0)
#define             XSTBYEN         (1U << 6)
#define         CRC             (0xe08U)
#define         ECCCON          (0xe0cU)
#define         ECCSTAT         (0xe10U)
#define         DEVID           (0xe14U)
#define         C1CON           (0x000U)
#define             TXBWS(n)        (((n) & 0xfU) << 28)
#define             REQOP(n)        (((n) & 0x7U) << 24)
#define             OPMOD(n)        (((n) & 0xfU) << 21)
#define             TXQEN           (1U << 20)
#define             STEF            (1U << 19)
#define             PXEDIS          (1U << 6)
#define         C1NBTCFG        (0x004U)
#define             BRP(n)          (((n) & 0xffU) << 24)
#define             TSEG1(n)        (((n) & 0xffU) << 16)
#define             TSEG2(n)        (((n) & 0x7fU) << 8)
#define             SJW(n)          (((n) & 0x7fU) << 0)
#define         C2DBTCFG        (0x008U)
#define         C1TDC           (0x00cU)
#define         C1TBC           (0x010U)
#define         C1TSCON         (0x014U)
#define             TSRES           (1U << 18)
#define             TSEOF           (1U << 17)
#define             TBCEN           (1U << 16)
#define             TBCPRE(n)       (((n) & 0x3ffU) << 0)
#define         C1VEC           (0x018U)
#define         C1INT           (0x01cU)
#define             IVMIE           (1U << 31)
#define             CERRIE          (1U << 29)
#define             TEFIE           (1U << 20)
#define             RXIE            (1U << 17)
#define             IVMIF           (1U << 15)
#define             WAKIF           (1U << 14)
#define             CERRIF          (1U << 13)
#define             SERRIF          (1U << 12)
#define             RXOVIF          (1U << 11)
#define             TXATIF          (1U << 10)
#define             SPICRCIF        (1U << 9)
#define             ECCIF           (1U << 8)
#define             TEFIF           (1U << 4)
#define             MODIF           (1U << 3)
#define             TBCIF           (1U << 2)
#define             RXIF            (1U << 1)
#define             TXIF            (1U << 0)
#define         C1RXIF          (0x020U)
#define         C1TIF           (0x024U)
#define         C1RXOVIF        (0x028U)
#define         C1TXATIF        (0x02cU)
#define         C1TXREQ         (0x030U)
#define         C1TREC          (0x034U)
#define             TXBO            (1U << 21)
#define         C1BDIAG0        (0x038U)
#define         C1BDIAG1        (0x03cU)
#define         C1TEFCON        (0x040U)
#define             FSIZE(n)        (((n) & 0x1fU) << 24) 
#define             FRESET          (1U << 10)
#define             UINC            (1U << 8)
#define             TEFTSEN         (1U << 5)
#define             TEFOVIE         (1U << 3)
#define             TEFFIE          (1U << 2)
#define             TEFHIE          (1U << 1)
#define             TEFNEIE         (1U << 0)
#define         C1TEFSTA        (0x044U)
#define             TEFOVIF         (1U << 3)
#define             TEFFIF          (1U << 2)
#define             TEFHIF          (1U << 1)
#define             TEFNEIF         (1U << 0)
#define         C1TEFUA         (0x048U)
#define         C1TXQCON        (0x050U)
#define             PLSIZE(n)       (((n) & 0x7U) << 29)
#define             FSIZE(n)        (((n) & 0x1fU) << 24)   // Shared definition with C1TEFCON
#define             TXAT(n)         (((n) & 0x3U) << 21)
#define             TXPRI(n)        (((n) & 0x1fU) << 16)
// #define             FRESET          (1U << 10)              // Shared definition with C1TEFCON
#define             TXREQ           (1U << 9)
#define             UINC            (1U << 8)               // Shared definition with C1TEFCON
#define             TXEN            (1U << 7)               // Shared definition with C1TEFCON
// #define             TXATIE          (1U << 5)
#define         C1TXQSTA        (0x054U)
#define             TXQNIF          (1U << 0)
#define         C1TXQUA         (0x058U)
#define         C1FIFOCON1      (0x05cU)
#define             RXTSEN          (1U << 5)
#define             TFNRFNIE        (1U << 0)

#define         C1FIFOSTA1      (0x060U)
#define         C1FIFOUA1       (0x064U)
#define         C1FLTCON(n)     (((n) * 4U) + 0x1d0U)
#define         C1FLTOBJ(n)     (((n) * 8U) + 0x1f0U)
#define         C1MASK(n)       (((n) * 8U) + 0x1f4U)

// The drivers do not support more than one controller, so hang on to the controller
// structure pointer and use it in the calls.

static can_controller_t *cc = NULL;

// Hard reset of the MCP2517FD using a special SPI command
static void TIME_CRITICAL hard_reset(void)
{
    // Must be called with interrupts locked

    uint8_t buf[2] = {0, 0};

    mcp2517fd_spi_select();
    mcp2517fd_spi_write(buf, 2U);
    mcp2517fd_spi_deselect();
}

// The MCP2517FD has options for driving its transceiver TX pin in open drain mode.
// It also has two pins that could be used as GPIO.
static void TIME_CRITICAL set_controller_pins(bool tx_open_drain)
{
    // Must be called with interrupts locked

    // Set SYSCLK to 40MHz, the external crystal, and don't use the PLL
    write_word(OSC, 0);
    // Set up IOCON by setting:
    //
    // SOF=1 to select SOF on CLKO
    // TXCANOD=1 to select open collector transmit pin
    // PM1=1 to use pin as GPIO1
    // PM0=1 to use pin as GPIO0
    // TRIS1=1 to select GPIO1 as an input
    // TRIS0=1 to select GPIO0 as an input
    uint32_t word = SOF | PM1 | PM0 | TRIS1 | TRIS0;
    if (tx_open_drain) {
        word |= TXCANOD;
    }
    write_word(IOCON, word);
}

// This is called after the mode change to normal has occurred so that there won't be a interrupt
// coming from configuration to normal mode (which would be confused with a bus-off interrupt).
static void TIME_CRITICAL enable_controller_interrupts(uint16_t dismiss)
{
    // Enable interrupts
    // IVMIE -  CAN error
    // CERRIE - CAN error status (error passive, bus-off, etc.)
    // TEFIE - transmit event FIFO
    // RXIE - receive FIFO
    //
    // Dismiss relevant interrupt flags:
    //
    // The interrupts to be dismissed are:
    ///
    // IVMIF
    // CERRIF
    //
    // The RXIF and TEFIF flags are cleared by dismissing the source (i.e. removing items
    // from the queue)
    uint32_t enable = IVMIE | CERRIE | RXIE | TEFIE;
    uint16_t dismiss_mask = ~dismiss;
    write_word(C1INT, enable | dismiss_mask);
}

// After a hard reset this should always succeed
// Returns true if succeeded, false if the device won't go into config mode
static bool TIME_CRITICAL set_controller_mode_config(void)
{
    // NB: The MCP2517FD pins must have been initialized before calling this function
    // Must be called with interrupts locked

    // Try multiple times to go into config mode before reporting an error
    for (uint32_t i = 0; i < 3U; i++) {
        write_word(C1CON, REQOP(4U));
        uint32_t c1con = read_word(C1CON);
        uint32_t current_mode = (c1con >> 21) & 0x7U;
        if (current_mode == 4U) {
            return true;
        }
    }
    return false;
}

// From configuration mode, go into requested mode with the defined bit rate
static bool TIME_CRITICAL set_controller_mode(can_mode_t mode, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
    // NB: The MCP2517FD pins must have been initialized before calling this function
    // Must be called with interrupts locked
    uint32_t c1con;
    uint32_t current_mode;

    c1con = read_word(C1CON);
    current_mode = (c1con >> 21) & 0x7U;

    if (current_mode == 4U) {
        // Set bit rate values
        write_word(C1NBTCFG, BRP(brp) | TSEG1(tseg1) | TSEG2(tseg2) | SJW(sjw));

        // Set timestamping counter
        // Set prescaler to /40 to count microseconds

        write_word(C1TSCON, TBCEN | TBCPRE(39U));

        // Transmit event FIFO control register
        // FSIZE 32-deep
        // TEFTSEN Timestamp transmissions
        // TEFNEIIE not empty interrupt enable
        write_word(C1TEFCON, FSIZE(0x1fU) | TEFTSEN | TEFNEIE);

        // Transmit queue control register
        // FSIZE 32-deep
        // TXAT Unlimited retransmissions (this field isn't active but set it anyway)
        write_word(C1TXQCON, FSIZE(0x1fU) | TXAT(0x3U));

        // FIFO 1 is the receive FIFO
        // FSIZE 32-deep
        // RXTSEN Timestamp receptions
        // TFNRFNIE interrupts enabled
        write_word(C1FIFOCON1, FSIZE(0x1fU) | RXTSEN | TFNRFNIE);

        // Enable the interrupts, don't dismiss any pending ones
        enable_controller_interrupts(0);

        // Enable transmit queue, store in transmit event FIFO, CAN 2.0 mode
        // Select mode
        uint32_t reqop;
        switch (mode) {
            default:
            case CAN_MODE_NORMAL:
                reqop = 6U;
                break;
            case CAN_MODE_LISTEN_ONLY:
                reqop = 3U;
                break;
            case CAN_MODE_ACK_ONLY:
                reqop = 7U;
                break;
            case CAN_MODE_OFFLINE:
                reqop = 4U;
                break;
        }

        // Try multiple times to put the controller into the desired mode, then give up with
        // an error. This might take some time because it has to wait for bus idle, which could
        // take up to a frame time to happen (134us at 500kbit/sec, much longer at slow bit rates)
        for (uint32_t i = 0; i < 64U; i++) {
            write_word(C1CON, STEF | TXQEN | REQOP(reqop));
            c1con = read_word(C1CON);
            uint32_t current_mode = (c1con >> 21) & 0x7U;
            if (current_mode == reqop) {
                return true;
            }
        }
    }
    // If wasn't in config mode already then this is an error, or it couldn't
    // be put into the desired mode then this is also an error
    return false;
}

// This is the main function for transmitting a frame
// Returns false if no room (used for various calls)
static bool TIME_CRITICAL send_frame(can_controller_t *controller, const can_frame_t *frame, bool fifo)
{
    // Must be called with interrupts locked
    // This function may be called from an ISR

    if (!fifo || controller->tx_pri_queue.fifo_slot == CAN_TX_QUEUE_SIZE) {
        // Put the frame in the priority transmit queue
        if (controller->tx_pri_queue.num_free_slots == 0) {
            // No room in the transmit queue
            return false;
        }
        else {
            // Write frame to a transmit priority queue message slot
            // This must fit into 16 bits because the buffer space in total is only 2Kbytes, and starts from 0x400
            uint32_t c1txqsta = read_word(C1TXQSTA);

            if ((c1txqsta & TXQNIF) == 0) {
                // Queue full, can't write to it, should not have happened because now inconsistent with
                // software counters
                controller->target_specific.txqsta_bad++;
                return false;
            }

            uint16_t c1txqua = (uint16_t)read_word(C1TXQUA);
            uint16_t addr = c1txqua + 0x400U;
            // (Transmit event slots start at an offset of 0 (a total of 3 x 4 bytes x 32 slots = 384 bytes)
            // Transmit queue slots start at an offset 0x180, and each is 16 bytes (we allocated 16 bytes to the
            // data even though handling only CAN frames, meaning the whole buffer slot is 16 bytes)
            uint32_t free_slot = (c1txqua - 0x180U) >> 4;

            if (free_slot >= CAN_TX_QUEUE_SIZE) {
                controller->target_specific.txqua_bad++;
                return false;
            }
            // Copy the frame into the message slot in the controller
            // Layout of TXQ message object:
            uint32_t t[4];
            // ID in controller is already the same as arbitration ID + IDE flag
            t[0] = frame->canid.id & 0x1fffffffU;
            t[1] = (free_slot << 9) | frame->dlc;
            if (frame->canid.id & (1U << CAN_ID_EXT_BIT)) {
                t[1] |= (1U << 4);    // IDE
            }
            if (frame->remote) {
                t[1] |= (1U << 5);
            }
            // Data words are in little endian format: first byte is bits 0-7
            t[2] = frame->data[0];
            t[3] = frame->data[1];

            // Mark slot and update next free slot
            if (fifo) {
                controller->tx_pri_queue.fifo_slot = free_slot;
            }
            controller->tx_pri_queue.num_free_slots--;
            controller->tx_pri_queue.uref[free_slot] = frame->uref;
            controller->tx_pri_queue.uref_valid[free_slot] = true;

            // Timestamps for transmitted frames are filled in via the user reference after
            // transmit events are processed
            // frame->timestamp_valid = false;

            // TODO could use a DMA channel and chain these SPI transactions using DMA
            // Write this block over SPI
            write_4words(addr, t);

            // Now tell the controller to take the frame and move C1TXQUA
            // Set UINC=1, TXREQ=1
            // Transmit queue control register
            write_word(C1TXQCON, UINC | TXREQ);

            return true;
        }
    }
    else {
        if (controller->tx_fifo.num_free_slots == 0) {
            // No room in the FIFO
            return false;
        }
        else {
            // Head of FIFO is in the priority queue so put this frame into the TX FIFO
            controller->tx_fifo.num_free_slots--;
            controller->tx_fifo.frames[controller->tx_fifo.tail_idx++] = *frame;
            if(controller->tx_fifo.tail_idx == CAN_TX_FIFO_SIZE) {
                // Wrap the tail index
                controller->tx_fifo.tail_idx = 0;
            }
            return true;
        }
    }
}

// Erase all transmit buffers (called by initialization and also as a response
// to bus-off since those frames will have been erased from the controller by
// the automatic bus-off recovery)
static void TIME_CRITICAL init_tx_buffers(can_controller_t *controller)
{
    // Must be called with interrupts locked

    controller->tx_fifo.head_idx = 0;
    controller->tx_fifo.tail_idx = 0;
    controller->tx_fifo.num_free_slots = CAN_TX_FIFO_SIZE;

    // Ensure there are no references to any CANFrame instances
    for (uint32_t i = 0; i < CAN_TX_QUEUE_SIZE; i++) {
        controller->tx_pri_queue.uref[i] = can_uref_null;
        controller->tx_pri_queue.uref_valid[i] = false;
    }
    controller->tx_pri_queue.num_free_slots = CAN_TX_QUEUE_SIZE;
    controller->tx_pri_queue.fifo_slot = CAN_TX_FIFO_SIZE;
}

// At present there is a single CAN controller on the board so there is no need to work out which
// device is interrupting, etc. The outermost handler passes the controller through as a parameter
// to allow this to be more easily converted to a multiple controller driver in the future.

// Called to deal with a frame that has been transmitted (actually the TEF IRQ on the device)
static void TIME_CRITICAL tx_handler(can_controller_t *controller)
{
    // Find out which frame was sent (using the SEQ field), get the frame object handle,
    // fill in the timestamp, remove the frame from the software queue, adding a FIFO queue
    // frame if necessary
    uint16_t addr = (uint16_t)read_word(C1TEFUA) + 0x400U;

    // Don't care about the CAN frame ID (we know it already), just SEQ and timestamp
    uint32_t details[2];
    read_words(addr + 4U, details, 2U);
    uint8_t seq = (uint8_t)(details[0] >> 9);

    // The sequence number may have been corrupted over SPI by noise so we treat it with some
    // suspicion. If it doesn't refer to a valid slot then we dismiss the interrupt without
    // processing it.
    if (seq > CAN_TX_QUEUE_SIZE || !controller->tx_pri_queue.uref_valid[seq]) {
        // Bad SEQ value, keep a count of it and then dismiss the interrupt. This will result
        // in the transmit buffer slot not being cleared, so slowly the buffer will run out of
        // space.
        // TODO use the CRC feature of the MCP2517FD SPI commands to check for bad SEQ and re-request
        controller->target_specific.seq_bad++;
    }
    else {
        uint32_t timestamp = details[1];
        bool fifo = (seq == controller->tx_pri_queue.fifo_slot);

        // Remove frame from the transmit queue
        if (fifo) {
            controller->tx_pri_queue.fifo_slot = CAN_TX_FIFO_SIZE;
        }
        can_uref_t uref = controller->tx_pri_queue.uref[seq];
        // The user-reference as no longer valid
        controller->tx_pri_queue.uref_valid[seq] = false;

        // Remove frame from transmit queue (most of the management of the free space in the
        // transmit queue is done by the hardware, but we keep track of how many free slots
        // to save SPI transactions asking for them)
        controller->tx_pri_queue.num_free_slots++;

        // Overwrite user-defined reference to allow for garbage collection (MicroPython scans
        // all RAM for references to the heap address space when doing garbage collection so
        // if this is running in MicroPython firmware it's important to ensure there are no
        // references left).
        controller->tx_pri_queue.uref[seq] = can_uref_null;
        // If this is a FIFO frame and there are more FIFO frames, then queue that one
        if (fifo && controller->tx_fifo.num_free_slots < CAN_TX_FIFO_SIZE) {
            controller->tx_fifo.num_free_slots++;
            can_frame_t *fifo_frame = &controller->tx_fifo.frames[controller->tx_fifo.head_idx];
            // Pop head of transmit FIFO
            controller->tx_fifo.head_idx++;
            if (controller->tx_fifo.head_idx == CAN_TX_FIFO_SIZE) {
                // Wrap the head index
                controller->tx_fifo.head_idx = 0;
            }
            send_frame(controller, fifo_frame, true);
        }

        can_isr_callback_frame_tx(uref, timestamp);
        if (controller->options & CAN_OPTION_RECORD_TX_EVENTS) {
            ////// Keep track of the transmit event //////
            // TODO the FIFO code should really be made generic (can't use compiler to inline due to XIP issue
            // on RP2040)
            if (controller->tx_event_fifo.free <= 1U) {
                // No space for it - discard event, marking the last free slot as an "overflow"
                if (controller->tx_event_fifo.free == 0) {
                    // Do nothing: there must already an overflow event at the back of the FIFO
                    controller->tx_event_fifo.events[controller->tx_event_fifo.dropped_event_idx].info.overflow_cnt++;
                } else {
                    // Add an 'overflow' event to the back of the queue
                    controller->tx_event_fifo.free = 0;
                    uint8_t idx = controller->tx_event_fifo.tail_idx++;
                    controller->tx_event_fifo.dropped_event_idx = idx;
                    if (controller->tx_event_fifo.tail_idx == CAN_TX_EVENT_FIFO_SIZE) {
                        controller->tx_event_fifo.tail_idx = 0;
                    }
                    controller->tx_event_fifo.events[idx].event_type = CAN_EVENT_TYPE_OVERFLOW;
                    // The tag is used as a dropped event counter
                    controller->tx_event_fifo.events[idx].info.overflow_cnt = 0;
                    // The timestamp is the time of the first drop
                    controller->tx_event_fifo.events[idx].timestamp = timestamp;
                }
            } else {
                // Put the frame transmit event into the FIFO: copy in the uref
                controller->tx_event_fifo.free--;
                uint8_t idx = controller->tx_event_fifo.tail_idx++;
                if (controller->tx_event_fifo.tail_idx == CAN_TX_EVENT_FIFO_SIZE) {
                    controller->tx_event_fifo.tail_idx = 0;
                }
                controller->tx_event_fifo.events[idx].timestamp = timestamp;
                controller->tx_event_fifo.events[idx].info.uref = uref;
                controller->tx_event_fifo.events[idx].event_type = CAN_EVENT_TYPE_TRANSMITTED_FRAME;
            }
        }
    }

    // Pop event in controller, keep timestamps enabled, keep not-empty interrupts enabled
    // MCP2517FD interrupts are level-sensitive so GPIO must be set to level sensitive; interrupt
    // will be re-raised if still not empty when serviced.
    // Set FSIZE, UINC, TEFTSEN, TEFNEIE
    write_word(C1TEFCON, FSIZE(0x1fU) | UINC | TEFTSEN | TEFNEIE);
}

static void TIME_CRITICAL bus_off_handler(can_controller_t *controller)
{
    // Frames should not be accepted for transmission if not yet in normal mode.
    uint32_t c1trec = read_word(C1TREC);

    if (c1trec & TXBO) {
        // Bus-off will erase the transmit queues, so all the frames we queued are now going to be
        // thrown away, so discard them in the software queues too

        // Discard all the pending frames
        init_tx_buffers(controller);
    }
    // This could be warning about error passive and other states, but we don't care about those
}

static void TIME_CRITICAL error_handler(can_controller_t *controller)
{
    // Read the time (won't be very accurate because of the time taken to get here)
    uint32_t timestamp = read_word(C1TBC);        // Get information about the error
    uint32_t c1bdiag1 = read_word(C1BDIAG1);
    // Clear down the diagnostics word
    write_word(C1BDIAG1, 0);

    can_error_t error = {.details = c1bdiag1};
    // May be configured not to receive errors into the receive FIFO
    can_isr_callback_error(error, timestamp);

    if (controller->options & CAN_OPTION_RECV_ERRORS) {
        // Put the error in to the receive FIFO since it's kind of a received thing, even if not a frame
        if (controller->rx_fifo.free <= 1U) {
            // No space for it - mark the last slot as an overflow and then future error / received frames can
            // increment the counter
            if (controller->rx_fifo.free == 0) {
                // There must already an overflow frame at the back of the queue
                controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.error_cnt++;
                if (controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.error_cnt == 0) {
                    // Overflowed so roll it back to make it sticky
                    controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.error_cnt--;
                }
            } else {
                // Add an 'overflow' frame to the back of the queue
                controller->rx_fifo.free = 0;
                uint8_t idx = controller->rx_fifo.tail_idx++;
                controller->rx_fifo.dropped_event_idx = idx;
                if (controller->rx_fifo.tail_idx == CAN_RX_FIFO_SIZE) {
                    controller->rx_fifo.tail_idx = 0;
                }
                controller->rx_fifo.rx_events[idx].event_type = CAN_EVENT_TYPE_OVERFLOW;
                // Initialize the counters
                controller->rx_fifo.rx_events[idx].event.overflow.error_cnt = 1U;  // Did not record this error frame
                controller->rx_fifo.rx_events[idx].event.overflow.frame_cnt = 0;
                // The timestamp is the time of the first drop
                controller->rx_fifo.rx_events[idx].timestamp = timestamp;
            }
        } else {
            // Put the error into the FIFO
            controller->rx_fifo.free--;
            uint8_t idx = controller->rx_fifo.tail_idx++;
            if (controller->rx_fifo.tail_idx == CAN_RX_FIFO_SIZE) {
                controller->rx_fifo.tail_idx = 0;
            }
            controller->rx_fifo.rx_events[idx].event_type = CAN_EVENT_TYPE_CAN_ERROR;
            controller->rx_fifo.rx_events[idx].timestamp = timestamp;
            // Helper functions to decode the error details
            controller->rx_fifo.rx_events[idx].event.error.details = c1bdiag1;
        };
    }
}

// Called with a received frame
// TODO performance enhancement: calculate addr by shadowing RX FIFO rather than use an SPI transaction to pick it up
static void TIME_CRITICAL rx_handler(can_controller_t *controller)
{
    uint16_t addr = (uint16_t)read_word(C1FIFOUA1) + 0x400U;

    // Pick up the frame
    uint32_t r[5];
    read_words(addr, r, 5U);

    // Get to the receive callback as quickly as possible

    // Assemble CAN ID from ID A, ID B and IDE
    can_id_t canid;

    // CAN ID in the controller is already in 'arbitration ID' layout
    uint32_t arbitration_id = r[0] & 0x1fffffffU;
    bool ide = r[0] & (1U << 4);
    canid.id = arbitration_id | (!!ide << CAN_ID_EXT_BIT);
    // canid.id = ((r[0] >> 11) & 0x3ffffU) | ((r[0] & 0x7ff) << 18) | ((r[1] & (1U << 4)) << 25);

    uint8_t dlc = r[1] & 0xfU;
    bool remote = (r[1] & (1U << 5)) != 0;
    uint8_t id_filter = (r[1] >> 11) & 0x1fU;
    uint32_t timestamp = r[2];
    uint32_t data_0 = r[3];
    uint32_t data_1 = r[4];

    can_frame_t frame = {.canid = canid,
                         .dlc = dlc,
                         .remote = remote,
                         .data[0] = data_0,
                         .data[1] = data_1,
                         .id_filter = id_filter};

    // Callback is a good place to put triggering
    can_isr_callback_frame_rx(&frame, timestamp);

    // Mark the frame as taken, ensure that timestamping and the not-empty interrupt are still enabled
    // Set UINC, RXTSEN, TFNRFNIE. This will dismiss the interrupt level (if there are no other interrupts)
    write_word(C1FIFOCON1, UINC | RXTSEN | TFNRFNIE);

    bool ignore_overflow = controller->options & CAN_OPTION_REJECT_OVERFLOW;
    bool ignore_remote = controller->options & CAN_OPTION_REJECT_REMOTE;
    bool store_frame = false;
    bool create_overflow = false;
    bool update_overflow = false;

    if ((remote && ignore_remote)) {
        store_frame = false;
    }
    if (ignore_overflow) {
        // Space for the frame
        store_frame = controller->rx_fifo.free > 0;
    }
    else {
        if (controller->rx_fifo.free > 1U) {
            // Space for the frame
            store_frame = true;
        }
        else {
            // Either no space (in which case there is already an overflow at the back)
            // or there is space for 1 slot (in which case, put an overflow event there) 
            if (controller->rx_fifo.free == 0) {
                // Update overflow
                update_overflow = true;
            }
            else {
                // There is one free slot at the back so put an overflow event in there
                create_overflow = true;
            }
        }
    }

    if (create_overflow) {
        // Add an 'overflow' frame to the back of the queue
        controller->rx_fifo.free = 0;
        uint8_t idx = controller->rx_fifo.tail_idx++;
        controller->rx_fifo.dropped_event_idx = idx;
        if (controller->rx_fifo.tail_idx == CAN_RX_FIFO_SIZE) {
            controller->rx_fifo.tail_idx = 0;
        }
        controller->rx_fifo.rx_events[idx].event_type = CAN_EVENT_TYPE_OVERFLOW;
        controller->rx_fifo.rx_events[idx].event.overflow.frame_cnt = 1U;  // Did not record this received frame
        controller->rx_fifo.rx_events[idx].event.overflow.error_cnt = 0;
        // The timestamp is the time of the first drop
        controller->rx_fifo.rx_events[idx].timestamp = timestamp;
    }
    if (update_overflow) {    
        // This doesn't add anything to the FIFO, just updates the tail record
        // There must already an overflow frame at the back of the queue
        controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.frame_cnt++;
        if (controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.frame_cnt == 0) {
            // Overflowed so roll it back to make it sticky
            controller->rx_fifo.rx_events[controller->rx_fifo.dropped_event_idx].event.overflow.frame_cnt--;
        }
    }
    if (store_frame) {
        // Put the frame into the FIFO
        controller->rx_fifo.free--;
        uint8_t idx = controller->rx_fifo.tail_idx++;
        if (controller->rx_fifo.tail_idx == CAN_RX_FIFO_SIZE) {
            controller->rx_fifo.tail_idx = 0;
        }
        controller->rx_fifo.rx_events[idx].event_type = CAN_EVENT_TYPE_RECEIVED_FRAME;
        controller->rx_fifo.rx_events[idx].timestamp = timestamp;
        controller->rx_fifo.rx_events[idx].event.frame = frame;
    }
}

static void TIME_CRITICAL pop_rx_event(can_controller_t *controller, can_rx_event_t *dest)
{
    // This must be called with interrupts disabled

    // Pop the front of the receive FIFO
    controller->rx_fifo.free++;
    uint8_t idx = controller->rx_fifo.head_idx++;
    if (controller->rx_fifo.head_idx == CAN_RX_FIFO_SIZE) {
        controller->rx_fifo.head_idx = 0;
    }

    // Copy out the old head of the RX FIFO to the destination
    *dest = controller->rx_fifo.rx_events[idx];
}

// Pop an event from the receive event FIFO and convert it into bytes
static void TIME_CRITICAL pop_rx_event_as_bytes(can_controller_t *controller, uint8_t *buf)
{
    // Must be called with interrupts disabled
    // Must be called with a non-empty FIFO
    // Must be called with enough space to store the result

    // Pop the front of the receive event FIFO
    controller->rx_fifo.free++;
    uint8_t idx = controller->rx_fifo.head_idx++;
    if (controller->rx_fifo.head_idx == CAN_RX_FIFO_SIZE) {
        controller->rx_fifo.head_idx = 0;
    }

    buf[0] = controller->rx_fifo.rx_events[idx].event_type;
    WRITE_BIG_ENDIAN(buf + 1U, controller->rx_fifo.rx_events[idx].timestamp);

    can_event_type_t ev = controller->rx_fifo.rx_events[idx].event_type;
    if (ev == CAN_EVENT_TYPE_OVERFLOW) {
        // Pack out the rest of the bytes with the overflow counts
        WRITE_BIG_ENDIAN(buf + 7U, controller->rx_fifo.rx_events[idx].event.overflow.frame_cnt);
        WRITE_BIG_ENDIAN(buf + 11U, controller->rx_fifo.rx_events[idx].event.overflow.error_cnt);
    }
    else if (ev == CAN_EVENT_TYPE_CAN_ERROR) {
        // Pack out the rest of the bytes with the details of the error
        WRITE_BIG_ENDIAN(buf + 7U, controller->rx_fifo.rx_events[idx].event.error.details);
    }
    else if (ev == CAN_EVENT_TYPE_RECEIVED_FRAME) {
        // Pack out the rest of the bytes with the frame details
        // Add flag info to indicate a remote frame
        buf[0] |= controller->rx_fifo.rx_events[idx].event.frame.remote ? 0x80U : 0x00U;
        // DLC, ID filter hit, timestamp, CAN ID, data
        buf[5] = controller->rx_fifo.rx_events[idx].event.frame.dlc;
        buf[6] = controller->rx_fifo.rx_events[idx].event.frame.id_filter;
        WRITE_BIG_ENDIAN(buf + 7U, controller->rx_fifo.rx_events[idx].event.frame.canid.id);
        for (size_t i = 0; i < 8U; i++) {
            buf[11U + i] = *((uint8_t *) (controller->rx_fifo.rx_events[idx].event.frame.data) + i);
        }
    }
}

static void TIME_CRITICAL pop_tx_event(can_controller_t *controller, can_tx_event_t *event)
{
    // This must be called with interrupts disabled

    // Pop the front of the transmit event FIFO
    controller->tx_event_fifo.free++;
    uint8_t idx = controller->tx_event_fifo.head_idx++;
    if (controller->tx_event_fifo.head_idx == CAN_TX_EVENT_FIFO_SIZE) {
        controller->tx_event_fifo.head_idx = 0;
    }

    *event = controller->tx_event_fifo.events[idx];
}

// Pop a transmit event (if there's room) into a buffer that has n bytes spare
static uint32_t TIME_CRITICAL pop_tx_event_as_bytes(can_controller_t *controller, uint8_t *buf, uint32_t n)
{
    // This must be called with interrupts disabled

    // Transmit event is:
    // Byte 0: event type (bits 1:0, 7:2 reserved)
    // Bytes 1-4: tag (in little endian format)
    // Bytes 5-8: timestamp (in little endian format)

    // Transmit event is exactly 9 bytes
    if (n >= NUM_TX_EVENT_BYTES) {
        // Pop the front of the transmit event FIFO
        controller->tx_event_fifo.free++;
        uint8_t idx = controller->tx_event_fifo.head_idx++;
        if (controller->tx_event_fifo.head_idx == CAN_TX_EVENT_FIFO_SIZE) {
            controller->tx_event_fifo.head_idx = 0;
        }

        // Set flags for the type of event
        buf[0] = controller->tx_event_fifo.events[idx].event_type;
        if (controller->tx_event_fifo.events[idx].event_type == CAN_EVENT_TYPE_TRANSMITTED_FRAME) {
            // Access the relevant parameter via a user_ref user reference
            can_uref_t uref = controller->tx_event_fifo.events[idx].info.uref;
            // Convert the uref into a 32-bit bytes object
            WRITE_BIG_ENDIAN(buf + 1U, can_isr_callback_uref(uref));
        }
        else {
            // Overflow event, store the overflow count is in the same place as the tag
            uint32_t overflow_cnt = controller->tx_event_fifo.events[idx].info.overflow_cnt;
            WRITE_BIG_ENDIAN(buf + 1U, overflow_cnt);
        }
        uint32_t timestamp = controller->tx_event_fifo.events[idx].timestamp;
        WRITE_BIG_ENDIAN(buf + 5U, timestamp);
        return NUM_TX_EVENT_BYTES;
    }
    else {
        return 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// DRIVER API ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void TIME_CRITICAL mcp2517fd_irq_handler(void)
{
    // Gets an interrupt from the following sources:
    //
    // Sent frame
    // Received frame
    // Bus-off
    // Error
    //
    // Can call a single sub-handler and each sub-handler will handle one
    // event; if there are multiple events then the IRQ will be re-raised
    // (since it is level-sensitive). Interrupt will also be re-raised if
    // more events have occurred.

    if (cc == NULL) {
        // Spurious interupt from the controller so just return
        return;
    }
    can_controller_t *controller = cc;

    // Read C1INT and then handle interrupts
    uint32_t events = read_word(C1INT);
    if (events & IVMIF) {               // IVMIF to detect error frames
        // Dismisses IVMIF interrupt (but not others)
        enable_controller_interrupts(IVMIF);
        error_handler(controller);

    } else if (events & CERRIF) {       // CERRIF to detect bus off transition
        // Dismisses CERRIF interrupt (but not others)
        enable_controller_interrupts(CERRIF);
        bus_off_handler(controller);
    }
    else if (events & TEFIF) {           // TEFIF (i.e. TEF event)
        tx_handler(controller);
    }
    else if (events & RXIF) {            // RXIF (i.e. received frame into the FIFO)
        rx_handler(controller);
    }
}

uint32_t TIME_CRITICAL can_get_time(void)
{
    mcp2517fd_spi_gpio_disable_irq();
    uint32_t timebase = read_word(C1TBC);
    mcp2517fd_spi_gpio_enable_irq();

    return timebase;
}

can_status_t TIME_CRITICAL can_get_status(void)
{
    can_status_t status = {.status = 0};

    if (cc != NULL) {
        mcp2517fd_spi_gpio_disable_irq();
        uint32_t trec = read_word(C1TREC);
        mcp2517fd_spi_gpio_enable_irq();
        status.status = trec;
    }
    return status;
}

void can_status_request_recover(void)
{
    // Empty function on the MCP2517FD and MCP2518FD: hardware
    // automatically recovers from Bus Off
}

// Create the CAN instance and initialize the controller
can_errorcode_t TIME_CRITICAL can_setup_controller(can_controller_t *controller,
                                                   const can_bitrate_t *bitrate,
                                                   const can_id_filters_t *all_filters,
                                                   can_mode_t mode,
                                                   uint16_t options)
{    
    // Modes are:
    // 0: (default) CAN_NORMAL, start normally
    // 1: CAN_LISTEN_ONLY, does not ever set TX to 0
    // 2: CAN_ACK_ONLY, does not transmit but does set ACK=0
    // 3: CAN_OFFLINE, does not send or receive

    cc = NULL;

    uint8_t brp;
    uint8_t tseg1;
    uint8_t tseg2;
    uint8_t sjw;

    if (all_filters != CAN_NO_FILTERS && all_filters->n_filters > CAN_MAX_ID_FILTERS) {
        return CAN_ERC_RANGE;   // Only up to 32 filters possible
    }

    // Ensure no old pending interrupts
    mcp2517fd_spi_gpio_disable_irq();

    // Sets up SPI channel, ensures chip not selected
    mcp2517fd_spi_pins_init();

    // Do a hard reset if requested, once the SPI has been set up: this generates
    // a special SPI reset command that causes the MCP2517FD to be reset
    if (options & CAN_OPTION_HARD_RESET) {
        hard_reset();
    }

    // Set controller's clock and I/O pins (and set TX pin to open drain if requested)
    set_controller_pins(options & CAN_OPTION_OPEN_DRAIN);

    // Set the bit rate values according to the profile, default to 500K if an unknown profile
    switch (bitrate->profile) {
        default:
        case CAN_BITRATE_500K_75:
            brp = 4U;       // 40MHz / 5 = 8MHz, 16 time quanta per bit
            tseg1 = 10U;    // Sync seg is 1
            tseg2 = 3U;
            sjw = 2U;
            break;
        case CAN_BITRATE_250K_75: // 250bit/sec, 75%
            brp = 9U;       // 40MHz / 10 = 8MHz, 16 time quanta per bit
            tseg1 = 10U;
            tseg2 = 3U;
            sjw = 2U;
            break;
        case CAN_BITRATE_125K_75:
            brp = 19U;      // 40MHz / 20 = 8MHz, 16 time quanta per bit
            tseg1 = 10U;
            tseg2 = 3U;
            sjw = 2U;
            break;
        case CAN_BITRATE_1M_75:
            brp = 1U;       // 40MHz / 2 = 20MHz, 20 time quanta per bit
            tseg1 = 13U;
            tseg2 = 4U;
            sjw = 2U;
            break;
        case CAN_BITRATE_500K_50:
            brp = 4U;       // 40MHz / 5 = 8MHz, 16 time quanta per bit
            tseg1 = 6U;     // Sync seg is 1
            tseg2 = 7U;
            sjw = 2U;
            break;
        case CAN_BITRATE_250K_50:
            brp = 9U;       // 40MHz / 10 = 8MHz, 16 time quanta per bit
            tseg1 = 6U;     // Sync seg is 1
            tseg2 = 7U;
            sjw = 2U;
            break;
        case CAN_BITRATE_125K_50:
            brp = 19U;      // 40MHz / 20 = 8MHz, 16 time quanta per bit
            tseg1 = 6U;     // Sync seg is 1
            tseg2 = 7U;
            sjw = 2U;
            break;
        case CAN_BITRATE_1M_50:
            brp = 1U;       // 40MHz / 2 = 20MHz, 20 time quanta per bit
            tseg1 = 8U;     // Sync seg is 1
            tseg2 = 9U;
            sjw = 2U;
            break;
        case CAN_BITRATE_2M_50:
            brp = 0;
            tseg1 = 8U;
            tseg2 = 9U;
            sjw = 1U;
            break;
        case CAN_BITRATE_4M_90:
            brp = 0;
            tseg1 = 7U;
            tseg2 = 0;
            sjw = 1U;
            break;
        case CAN_BITRATE_2_5M_75:
            brp = 1;
            tseg1 = 4U;
            tseg2 = 1U;
            sjw = 1U;
            break;
        case CAN_BITRATE_2M_80:
            brp = 0U;
            tseg1 = 14U;
            tseg2 = 3U;
            sjw = 1U;
            break;
        case CAN_BITRATE_500K_875:
            brp = 4U;       // 40MHz / 5 = 8MHz, 16 time quanta per bit
            tseg1 = 12U;    // Sync seg is 1
            tseg2 = 1U;
            sjw = 1U;
            break;
        case CAN_BITRATE_250K_875: // 250bit/sec, 75%
            brp = 9U;       // 40MHz / 10 = 8MHz, 16 time quanta per bit
            tseg1 = 12U;
            tseg2 = 1U;
            sjw = 1U;
            break;
        case CAN_BITRATE_125K_875:
            brp = 19U;      // 40MHz / 20 = 8MHz, 16 time quanta per bit
            tseg1 = 12U;
            tseg2 = 1U;
            sjw = 1U;
            break;
        case CAN_BITRATE_1M_875:
            brp = 1U;       // 40MHz / 2 = 20MHz, 20 time quanta per bit
            tseg1 = 15U;
            tseg2 = 2U;
            sjw = 1U;
            break;
        case CAN_BITRATE_CUSTOM:
            brp = bitrate->brp;
            tseg1 = bitrate->tseg1;
            tseg2 = bitrate->tseg2;
            sjw = bitrate->sjw;
            break;
    }

    // Won't go into config mode then return an error
    if (!set_controller_mode_config()) {
        return CAN_ERC_BAD_INIT;
    }

    // Disable all the ID filters (which might not be disabled if we come into this with the controller
    // having already been running and not reset)
    uint32_t filter_control[8];
    for (uint32_t i = 0; i < 8U; i++) {
        filter_control[i] = 0;
        write_word(C1FLTCON(i), 0);
    }

    // Add in the filters
    if (all_filters != CAN_NO_FILTERS) {
        for (uint32_t idx = 0; idx < all_filters->n_filters; idx++) {
            can_id_filter_t *filter = &all_filters->filter_list[idx];
            if (filter->enabled) {
                // Enables the filter and sets it to direct frames to RX FIFO 1
                filter_control[idx >> 2] |= (0x81U << ((idx & 0x03U) << 3));
                // Sets the mask/match registers accordingly
                write_word(C1FLTOBJ(idx), all_filters->filter_list[idx].fltobj);
                write_word(C1MASK(idx), all_filters->filter_list[idx].mask);
            }
        }
    } else {
        // Set filter 0 to match-all and direct frames to the RX FIFO
        filter_control[0] = 0x81U;
        write_word(C1FLTOBJ(0), 0);
        write_word(C1MASK(0), 0);
    }

    // Enable the appropriate filters
    for (uint32_t i = 0; i < 8U; i++) {
        write_word(C1FLTCON(i), filter_control[i]);
    }

    // Init data structures
    // Receive frame FIFO is empty
    controller->rx_fifo.head_idx = 0;
    controller->rx_fifo.tail_idx = 0;
    controller->rx_fifo.free = CAN_RX_FIFO_SIZE;
    controller->rx_fifo.dropped_event_idx = 0;

    // Transmit event FIFO is empty
    controller->tx_event_fifo.head_idx = 0;
    controller->tx_event_fifo.tail_idx = 0;
    controller->tx_event_fifo.free = CAN_TX_EVENT_FIFO_SIZE;
    controller->tx_event_fifo.dropped_event_idx = 0;

    // Initialize transmit buffers
    init_tx_buffers(controller);

    // Record details of the controller
    controller->options = options;
    controller->mode = mode;
    controller->target_specific.seq_bad = 0;
    controller->target_specific.txqua_bad = 0;
    controller->target_specific.txqsta_bad = 0;

    if (!set_controller_mode(mode, brp, tseg1, tseg2, sjw)) {
        // Won't go into the requested mode, return an error
        return CAN_ERC_BAD_INIT;
    }

    cc = controller;

    // Enable SPI interrupts on controller IRQ pin. GPIO interrupts will already have been enabled
    // by the initialization of the pins in mcp2517fd_spi_pins_init()
    mcp2517fd_spi_gpio_enable_irq();

    return CAN_ERC_NO_ERROR;
}

can_errorcode_t TIME_CRITICAL can_send_frame(const can_frame_t *frame, bool fifo)
{
    if (cc == NULL) {
        return CAN_ERC_BAD_INIT;
    }
    can_controller_t *controller = cc;
    // Going to put a frame into the transmit queue or the transmit FIFO
    mcp2517fd_spi_gpio_disable_irq();
    bool queued = send_frame(controller, frame, fifo);
    mcp2517fd_spi_gpio_enable_irq();

    return queued ? CAN_ERC_NO_ERROR : CAN_ERC_NO_ROOM;
}

uint32_t TIME_CRITICAL can_recv_as_bytes(uint8_t *dest, size_t n_bytes)
{
    // There is a single controller set up
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then return no bytes
        return 0;
    }
    if (n_bytes < NUM_RX_EVENT_BYTES) {
        return 0;
    }

    uint32_t result;
    mcp2517fd_spi_gpio_disable_irq();
    if (CAN_RX_FIFO_SIZE - controller->rx_fifo.free) {
        pop_rx_event_as_bytes(controller, dest);
        result = NUM_RX_EVENT_BYTES;
    }
    else {
        result = 0;
    }
    mcp2517fd_spi_gpio_enable_irq();
    return result;
}

// Receives one event from the receive FIFO (events can be CAN frames, CAN errors, FIFO overflow)
bool TIME_CRITICAL can_recv(can_rx_event_t *event)
{
    // There is a single controller set up
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then return nothing received
        return false;
    }

    bool result;
    mcp2517fd_spi_gpio_disable_irq();
    uint32_t max_num_events = CAN_RX_FIFO_SIZE - controller->rx_fifo.free;
    if (max_num_events > 0) {
        pop_rx_event(controller, event);
        result = true;
    }
    else {
        result = false;
    }
    mcp2517fd_spi_gpio_enable_irq();

    return result;
}

// Return number of events waiting in the RX FIFO.`
uint32_t TIME_CRITICAL can_recv_pending(void)
{
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then return nothing
        return 0;
    }
    // It's probably OK to access this with interrupt concurrency because it's an atomic word, but
    // other calls extend the interrupt lock around this to guard against an empty queue being
    // accessed.
    return CAN_RX_FIFO_SIZE - controller->rx_fifo.free;
}

bool TIME_CRITICAL can_recv_tx_event(can_tx_event_t *event)
{
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then nothing has returned
        return false;
    }

    // tx_event_fifo.free is probably to access outside the ISR because it's likely to compile
    // to an atomic word and can only decrease so num_events can only increase
    mcp2517fd_spi_gpio_disable_irq();
    bool result;
    uint32_t max_num_events = CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
    if (max_num_events > 0) {
        pop_tx_event(controller, event);
        result = true;
    }
    else {
        result = false;
    }
    mcp2517fd_spi_gpio_enable_irq();

    return result;
}

uint32_t TIME_CRITICAL can_recv_tx_event_as_bytes(uint8_t *dest, size_t n_bytes)
{
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then return no bytes
        return 0;
    }

    // tx_event_fifo.free is probably OK to access outside the ISR because it's likely to compile
    // to an atomic word and can only decrease so num_events can only increase
    mcp2517fd_spi_gpio_disable_irq();
    uint32_t recvd;
    uint32_t max_num_events = CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
    if (max_num_events > 0) {
        pop_tx_event_as_bytes(controller, dest, n_bytes);
        recvd = NUM_TX_EVENT_BYTES;
    }
    else {
        recvd = 0;
    }
    mcp2517fd_spi_gpio_enable_irq();
    // Return the number of bytes actually received
    return recvd;
}

// Return number of events waiting in the TX event FIFO
uint32_t TIME_CRITICAL can_recv_tx_events_pending(void)
{
    can_controller_t *controller = cc;
    if (controller == NULL) {
        // If the controller has not been initialized then return no events pending
        return 0;
    }

    // It's OK to access this with interrupt concurrency because it's an atomic word
    return CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
}

uint32_t TIME_CRITICAL can_get_send_space(bool fifo)
{
    can_controller_t *controller = cc;
        if (controller == NULL) {
        // If the controller has not been initialized then return no space
        return 0;
    }

    if (fifo) {
        return controller->tx_fifo.num_free_slots;
    }
    else {
        return controller->tx_pri_queue.num_free_slots;
    }
}

bool TIME_CRITICAL can_is_space(uint32_t n_frames, bool fifo)
{
    can_controller_t *controller = cc;
        if (controller == NULL) {
        // If the controller has not been initialized then return no space
        return false;
    }

    // Check there is room for the total number of frames, in the queue or in the software FIFO
    // so that all the frames are queued or none are
    if (fifo) {
        if (controller->tx_pri_queue.fifo_slot == CAN_TX_QUEUE_SIZE) {
            // No existing FIFO frame in the transmit queue
            if (controller->tx_pri_queue.num_free_slots < 1U) {
                // No room for the first frame in the priority queue
                return false;
            }
            if (n_frames - 1U > controller->tx_fifo.num_free_slots) {
                // No room for the rest of the frames frames in the FIFO
                return false;
            }
        }
        else {
            // No room for all the frames in the FIFO
            if (n_frames > controller->tx_fifo.num_free_slots) {
                return false;
            }
        }
    }
    else {
        if (n_frames > controller->tx_pri_queue.num_free_slots) {
            return false;
        }
    }
    return true;
}

void can_stop_controller(void)
{
    if (cc == NULL) {
        // Already not started
        return;
    }

    // Disable SPI interrupts from the GPIO pin so this
    // runs without interference from the ISRs.
    mcp2517fd_spi_gpio_disable_irq();

    // Request config mode to stop the device transmitting if it was
    write_word(C1CON, REQOP(4U));

    // Remove all the items in the transmit queue (if enabled) so that
    // there are no user references
    init_tx_buffers(cc);

    // Stop all controller interrupts at source
    write_word(C1INT, 0);

    // Remove the reference to the controller
    cc = NULL;
}
