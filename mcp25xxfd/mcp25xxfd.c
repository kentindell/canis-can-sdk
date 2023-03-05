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

// Drivers for the MCP251FD and MCP2518FD CAN controllers from Microchip (including
// the MCP251863 combined CAN controller and CAN transceiver package)
//
// See: https://www.microchip.com/en-us/product/MCP2517FD
//
// The MCP2518FD is a newer version of the MCP2517FD but the drivers are compatible
// with both variants. The device is an external CAN controller connected via SPI.
// This file and its header file mcp25xxfd.h is the generic C device driver that
// is portable to any microcontroller.
//
// This driver is bound to a specific platform via target-specific SPI driver functions,
// as defined in the API.
//
// TODO speed up ISRs: calculate buffer addr by shadowing TEF and RX FIFO rather than
//      use an SPI transaction to pick up
// TODO parameterize the mode request call to allow a timeout for requesting the controller
//      go into a specific mode (the CAN state machine goes into bus integration and 
//      comes out in the requested mode only after that completes successfully)
// TODO faster FIFO implementation using power-of-two masks on index values
// TODO add low-power standby mode to API (put controller into standby, put transceiver into standby via XSTBY pin)
// TODO add support for multiple transmit FIFOs feeding into the priority queue
//
// Silicon Errata
// ==============
//
// Errata sheeet DS80000789D for the MCP2518FD shows there are some silicon bugs that need
// to be worked around:
//
// 1. Reading from these registers may result in corruption due to an SPI glitch:
//
// C1TXIF
// C1RXIF
// C1CON
// C1TBC
// C1INT
// C1RXOVIF
// C1TXATIF
// C1TXREQ
// C1TREC
// C1BDIAG0
// C1BDIAG1
// C1TXQSTA
// C1FIFOSTAm  
//
// Workaround is to use the READ_CRC function to read them (which adds 2 bytes to the
// transaction) and then re-issue a request if that fails. There is a gnomic phrase
// "The occurrence can be minimized by not using FIFOs 7/15/23/31" which doesn't help much.
//
// 2. Using word writes to IOCON can result in corruption of LAT0 and LAT1. These are
// the GPIO outputs and not used on the CANPico but to work around this, single byte SFR
// requests should be issued.

#include <stdbool.h>

// Brings in API, chip-specific and board-specific definitions
#include "../canapi.h"

// FIXME get rid of this after debugging done
typedef void (*mp_print_strn_t)(void *data, const char *str, size_t len);
typedef struct _mp_print_t {
    void *data;
    mp_print_strn_t print_strn;
} mp_print_t;
int mp_printf(const mp_print_t *print, const char *fmt, ...);
extern const mp_print_t mp_plat_print;
#define MP_PYTHON_PRINTER &mp_plat_print
void debug_printf( const char *format, ... );

// Number of times a CRC-failed transaction will be re-tried
#define CRC_RETRIES                         (5U)
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
///////////////////////////////////////// MCP251xFD SPI commands /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Support functions to convert controller register read/write operations into the MCP251xFD's SPI commands
static void TIME_CRITICAL write_word(can_interface_t *spi_interface, uint32_t addr, uint32_t word)
{
    // Must be called with interrupts locked
    uint8_t buf[6];
    // MCP251xFD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x20 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;
    buf[2] = word & 0xffU;
    buf[3] = (word >> 8) & 0xffU;
    buf[4] = (word >> 16) & 0xffU;
    buf[5] = (word >> 24) & 0xffU;

    // SPI transaction
    mcp25xxfd_spi_select(spi_interface);
    mcp25xxfd_spi_write(spi_interface, buf, sizeof(buf));
    mcp25xxfd_spi_deselect(spi_interface);
}

static void TIME_CRITICAL write_byte(can_interface_t *spi_interface, uint32_t addr, uint8_t byte)
{
    // Must be called with interrupts locked

    uint8_t buf[3];
    // MCP251xFD SPI transaction = command/addr, 1 byte
    buf[0] = 0x20 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;
    buf[2] = byte;

    // SPI transaction
    mcp25xxfd_spi_select(spi_interface);
    mcp25xxfd_spi_write(spi_interface, buf, sizeof(buf));
    mcp25xxfd_spi_deselect(spi_interface);
}

static void TIME_CRITICAL write_4words(can_interface_t *spi_interface, uint16_t addr, const uint32_t words[])
{
    // Must be called with interrupts locked

    // Prepare a contiguous buffer for the command because the SPI hardware is pipelined and do not want to stop
    // to switch buffers
    uint8_t cmd[18];
    // MCP251xFD SPI transaction = command/addr, 4 bytes
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
    mcp25xxfd_spi_select(spi_interface);
    mcp25xxfd_spi_write(spi_interface, cmd, sizeof(cmd));
    mcp25xxfd_spi_deselect(spi_interface);
}

// Due to errata workarounds, the non-CRC checked read is not used, but
// the code is left here for future use.
#ifdef NOTDEF
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
    mcp25xxfd_spi_select();
    mcp25xxfd_spi_read_write(cmd, resp, sizeof(cmd));
    mcp25xxfd_spi_deselect();

    uint32_t word = ((uint32_t)resp[2]) | ((uint32_t)resp[3] << 8) | ((uint32_t)resp[4] << 16) | ((uint32_t)resp[5] << 24);
    return word;
}
#endif

// Calculate the CRC-16 value for each byte, using a lookup table. Table must
// be in RAM on the RP2040 to avoid XIP flash cache misses. Don't declare
// this as static in case the compiler tries to put the table into flash.
// so apply global namespace rule to the table name.
CONST_STORAGE uint16_t mcp25xxfd_crc16_table[256] = {
    0x0000U, 0x8005U, 0x800fU, 0x000aU, 0x801bU, 0x001eU, 0x0014U, 0x8011U,
    0x8033U, 0x0036U, 0x003cU, 0x8039U, 0x0028U, 0x802dU, 0x8027U, 0x0022U,
    0x8063U, 0x0066U, 0x006cU, 0x8069U, 0x0078U, 0x807dU, 0x8077U, 0x0072U,
    0x0050U, 0x8055U, 0x805fU, 0x005aU, 0x804bU, 0x004eU, 0x0044U, 0x8041U,
    0x80c3U, 0x00c6U, 0x00ccU, 0x80c9U, 0x00d8U, 0x80ddU, 0x80d7U, 0x00d2U,
    0x00f0U, 0x80f5U, 0x80ffU, 0x00faU, 0x80ebU, 0x00eeU, 0x00e4U, 0x80e1U,
    0x00a0U, 0x80a5U, 0x80afU, 0x00aaU, 0x80bbU, 0x00beU, 0x00b4U, 0x80b1U,
    0x8093U, 0x0096U, 0x009cU, 0x8099U, 0x0088U, 0x808dU, 0x8087U, 0x0082U,
    0x8183U, 0x0186U, 0x018cU, 0x8189U, 0x0198U, 0x819dU, 0x8197U, 0x0192U,
    0x01b0U, 0x81b5U, 0x81bfU, 0x01baU, 0x81abU, 0x01aeU, 0x01a4U, 0x81a1U,
    0x01e0U, 0x81e5U, 0x81efU, 0x01eaU, 0x81fbU, 0x01feU, 0x01f4U, 0x81f1U,
    0x81d3U, 0x01d6U, 0x01dcU, 0x81d9U, 0x01c8U, 0x81cdU, 0x81c7U, 0x01c2U,
    0x0140U, 0x8145U, 0x814fU, 0x014aU, 0x815bU, 0x015eU, 0x0154U, 0x8151U,
    0x8173U, 0x0176U, 0x017cU, 0x8179U, 0x0168U, 0x816dU, 0x8167U, 0x0162U,
    0x8123U, 0x0126U, 0x012cU, 0x8129U, 0x0138U, 0x813dU, 0x8137U, 0x0132U,
    0x0110U, 0x8115U, 0x811fU, 0x011aU, 0x810bU, 0x010eU, 0x0104U, 0x8101U,
    0x8303U, 0x0306U, 0x030cU, 0x8309U, 0x0318U, 0x831dU, 0x8317U, 0x0312U,
    0x0330U, 0x8335U, 0x833fU, 0x033aU, 0x832bU, 0x032eU, 0x0324U, 0x8321U,
    0x0360U, 0x8365U, 0x836fU, 0x036aU, 0x837bU, 0x037eU, 0x0374U, 0x8371U,
    0x8353U, 0x0356U, 0x035cU, 0x8359U, 0x0348U, 0x834dU, 0x8347U, 0x0342U,
    0x03c0U, 0x83c5U, 0x83cfU, 0x03caU, 0x83dbU, 0x03deU, 0x03d4U, 0x83d1U,
    0x83f3U, 0x03f6U, 0x03fcU, 0x83f9U, 0x03e8U, 0x83edU, 0x83e7U, 0x03e2U,
    0x83a3U, 0x03a6U, 0x03acU, 0x83a9U, 0x03b8U, 0x83bdU, 0x83b7U, 0x03b2U,
    0x0390U, 0x8395U, 0x839fU, 0x039aU, 0x838bU, 0x038eU, 0x0384U, 0x8381U,
    0x0280U, 0x8285U, 0x828fU, 0x028aU, 0x829bU, 0x029eU, 0x0294U, 0x8291U,
    0x82b3U, 0x02b6U, 0x02bcU, 0x82b9U, 0x02a8U, 0x82adU, 0x82a7U, 0x02a2U,
    0x82e3U, 0x02e6U, 0x02ecU, 0x82e9U, 0x02f8U, 0x82fdU, 0x82f7U, 0x02f2U,
    0x02d0U, 0x82d5U, 0x82dfU, 0x02daU, 0x82cbU, 0x02ceU, 0x02c4U, 0x82c1U,
    0x8243U, 0x0246U, 0x024cU, 0x8249U, 0x0258U, 0x825dU, 0x8257U, 0x0252U,
    0x0270U, 0x8275U, 0x827fU, 0x027aU, 0x826bU, 0x026eU, 0x0264U, 0x8261U,
    0x0220U, 0x8225U, 0x822fU, 0x022aU, 0x823bU, 0x023eU, 0x0234U, 0x8231U,
    0x8213U, 0x0216U, 0x021cU, 0x8219U, 0x0208U, 0x820dU, 0x8207U, 0x0202U};

// Compute next CRC for a byte
static uint16_t TIME_CRITICAL crc16_byte(uint16_t crc, uint8_t byte)
{
    crc = (crc << 8) ^ mcp25xxfd_crc16_table[(((uint8_t)(crc >> 8)) & 0xffU) ^ byte];

    return crc;
}

// As for read_word() but asks for the hardware to add a 2-byte CRC to the end,
// and re-issues the read if the CRC fails. If the read fails multiple
// times then eventually a result of 0 is returned.
static uint32_t TIME_CRITICAL read_word_crc(can_interface_t *spi_interface, uint16_t addr)
{
    // Must be called with interrupts locked

    uint8_t retries = 0;
    uint16_t crc_calc, crc_rxd;

    // Keep trying but avoid an infinite loop that will lock the CPU if something
    // happens like the device goes completely missing
    while (retries < CRC_RETRIES) {
        uint8_t cmd[9];
        uint8_t resp[9];

        // 0xb0 is "read with CRC" (c.f. 0x30 for read without CRC)
        cmd[0] = 0xb0 | ((addr >> 8U) & 0xfU);
        cmd[1] = addr & 0xffU;
        cmd[2] = 0x04U; // Four bytes to read
        // TODO could remove the following because not strictly necessary (but useful for debugging
        // with a logic analyzer)
        cmd[3] = 0xdeU; // The four bytes of data
        cmd[4] = 0xadU;
        cmd[5] = 0xbeU;
        cmd[6] = 0xefU;
        cmd[7] = 0xc0U; // Two bytes of CRC
        cmd[8] = 0x0cU;

        // SPI transaction
        mcp25xxfd_spi_select(spi_interface);
        mcp25xxfd_spi_read_write(spi_interface, cmd, resp, sizeof(cmd));
        mcp25xxfd_spi_deselect(spi_interface);

        // Calculate CRC on the transaction. The documentation is wrong and the
        // CRC is computed on the command bytes sent to the device and then on the
        // data bytes sent back (excluding the CRC itself, of course).
        crc_calc = 0xffffU;
        crc_calc = crc16_byte(crc_calc, cmd[0]);
        crc_calc = crc16_byte(crc_calc, cmd[1]);
        crc_calc = crc16_byte(crc_calc, cmd[2]);
        crc_calc = crc16_byte(crc_calc, resp[3]);
        crc_calc = crc16_byte(crc_calc, resp[4]);
        crc_calc = crc16_byte(crc_calc, resp[5]);
        crc_calc = crc16_byte(crc_calc, resp[6]);

        // CRC is transmitted by the chip as a big endian 16-bit integer
        crc_rxd = (resp[7] << 8) | (resp[8]);
    
        if (crc_calc != crc_rxd) {
            spi_interface->crc_errors++;
            retries++;
        }
        else {
            uint32_t word;
            word = ((uint32_t)resp[3]) | ((uint32_t)resp[4] << 8) | ((uint32_t)resp[5] << 16) | ((uint32_t)resp[6] << 24);
            return word;
        }
    }
    return 0xffffffffU; // A non-zero value to ensure a bad SEQ read is out of range;
}

// Errata does not require buffer space words to be read via CRC as a workaround to
// silicon bugs. This of course might not be true.
static void TIME_CRITICAL read_words(can_interface_t *spi_interface, uint16_t addr, uint32_t *words, uint32_t n)
{
    // Must be called with interrupts locked

    uint8_t buf[2];

    // MCP251xFD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x30 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;

    // SPI transaction
    mcp25xxfd_spi_select(spi_interface);
    // Send command, which flushes the pipeline then resumes
    mcp25xxfd_spi_write(spi_interface, buf, 2U);
    // Bulk data
    mcp25xxfd_spi_read(spi_interface, (uint8_t *)(words), 4U * n);
    mcp25xxfd_spi_deselect(spi_interface);
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
////////////////////////////////////// Start of MCP251xFD drivers //////////////////////////////////////
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
#define             TXBP            (1U << 20)
#define             RXBP            (1U << 19)
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

// Hard reset of the MCP251xFD using a special SPI command
static void TIME_CRITICAL hard_reset(can_interface_t *spi_interface)
{
    // Must be called with interrupts locked

    uint8_t buf[2] = {0, 0};

    mcp25xxfd_spi_select(spi_interface);
    mcp25xxfd_spi_write(spi_interface, buf, 2U);
    mcp25xxfd_spi_deselect(spi_interface);
}

// The MCP251xFD has options for driving its transceiver TX pin in open drain mode.
// It also has two pins that could be used as GPIO.
static void TIME_CRITICAL set_controller_pins(can_interface_t *spi_interface, bool tx_open_drain)
{
    // Must be called with interrupts locked

    // Set SYSCLK to the external crystal (40MHz on the CANPico) and don't use the PLL
    write_word(spi_interface, OSC, 0);
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
    // Due to a silicon bug, IOCON must be written byte-by-byte
    // MCP251xFD is little-endian (least-signficant bits of a word are in low
    // memory address - see Figure 3-1 of datasheet). Compiler will spot these
    // shifts and turn them into byte accesses.
    write_byte(spi_interface, IOCON + 0U, (word >> 0) & 0xffU);
    write_byte(spi_interface, IOCON + 1U, (word >> 8) & 0xffU);
    write_byte(spi_interface, IOCON + 2U, (word >> 16) & 0xffU);
    write_byte(spi_interface, IOCON + 3U, (word >> 24) & 0xffU);
}

// This is called after the mode change to normal has occurred so that there won't be a interrupt
// coming from configuration to normal mode (which would be confused with a bus-off interrupt).
// The dismiss parameter indicates which interrupts should be dismissed.
static void TIME_CRITICAL enable_controller_interrupts(can_interface_t *spi_interface, uint16_t dismiss)
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
    write_word(spi_interface, C1INT, enable | dismiss_mask);
}

// After a hard reset this should always succeed
// Returns true if succeeded, false if the device won't go into config mode
// Errata: C1CON read can be corrupted
static bool TIME_CRITICAL set_controller_mode_config(can_interface_t *spi_interface)
{
    // NB: The MCP251xFD pins must have been initialized before calling this function
    // Must be called with interrupts locked

    // Try multiple times to go into config mode before reporting an error
    // Note that if no hardware is connected then the CRC will retry several times,
    // so this could result in several transactions.
    for (uint32_t i = 0; i < 3U; i++) {
        write_word(spi_interface, C1CON, REQOP(4U));
        uint32_t c1con = read_word_crc(spi_interface, C1CON);
        uint32_t current_mode = (c1con >> 21) & 0x7U;
        if (current_mode == 4U) {
            return true;
        }
    }
    return false;
}

// From configuration mode, go into requested mode with the defined bit rate
// Errata: C1CON read can be corrupted
static bool TIME_CRITICAL set_controller_mode(can_interface_t *spi_interface, can_mode_t mode, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
    // NB: The MCP251xFD pins must have been initialized before calling this function
    // Must be called with interrupts locked
    uint32_t c1con;
    uint32_t current_mode;

    c1con = read_word_crc(spi_interface, C1CON);
    current_mode = (c1con >> 21) & 0x7U;

    if (current_mode == 4U) {
        // Set bit rate values
        write_word(spi_interface, C1NBTCFG, BRP(brp) | TSEG1(tseg1) | TSEG2(tseg2) | SJW(sjw));

        // Set timestamping counter
        // Set prescaler to /40 to count microseconds

        write_word(spi_interface, C1TSCON, TBCEN | TBCPRE(39U));

        // Transmit event FIFO control register
        // FSIZE 32-deep
        // TEFTSEN Timestamp transmissions
        // TEFNEIIE not empty interrupt enable
        write_word(spi_interface, C1TEFCON, FSIZE(0x1fU) | TEFTSEN | TEFNEIE);

        // Transmit queue control register
        // FSIZE 32-deep
        // TXAT Unlimited retransmissions (this field isn't active but set it anyway)
        write_word(spi_interface, C1TXQCON, FSIZE(0x1fU) | TXAT(0x3U));

        // FIFO 1 is the receive FIFO
        // FSIZE 32-deep
        // RXTSEN Timestamp receptions
        // TFNRFNIE interrupts enabled
        write_word(spi_interface, C1FIFOCON1, FSIZE(0x1fU) | RXTSEN | TFNRFNIE);

        // Enable the interrupts, don't dismiss any pending ones
        enable_controller_interrupts(spi_interface, 0);

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
            write_word(spi_interface, C1CON, STEF | TXQEN | REQOP(reqop));
            c1con = read_word_crc(spi_interface, C1CON);
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
// Errata: C1TXQSTA read can be corrupted
static bool TIME_CRITICAL send_frame(can_controller_t *controller, const can_frame_t *frame, bool fifo)
{
    // Must be called with interrupts locked
    // This function may be called from an ISR

    can_interface_t *spi_interface = &controller->host_interface;

    if (!fifo || controller->tx_pri_queue.fifo_slot == CAN_TX_QUEUE_SIZE) {
        // Put the frame in the priority transmit queue
        if (controller->tx_pri_queue.num_free_slots == 0) {
            // No room in the transmit queue
            return false;
        }
        else {
            // Write frame to a transmit priority queue message slot
            // This must fit into 16 bits because the buffer space in total is only 2Kbytes, and starts from 0x400
            uint32_t c1txqsta = read_word_crc(spi_interface, C1TXQSTA);

            ///// Error checking code: should not happen if hardware is behaving correctly
            if ((c1txqsta & TXQNIF) == 0) {
                // Queue full, can't write to it, should not have happened because now inconsistent with
                // software counters
                controller->target_specific.txqsta_bad++;
                return false;
            }

            // Although the official errata does not list this register as being corrupted,
            // such corruption has been seen and so we use a CRC-protected read.
            uint16_t c1txqua = (uint16_t)read_word_crc(spi_interface, C1TXQUA);
            uint16_t addr = c1txqua + 0x400U;
            // (Transmit event slots start at an offset of 0 (a total of 3 x 4 bytes x 32 slots = 384 bytes)
            // Transmit queue slots start at an offset 0x180, and each is 16 bytes (we allocated 16 bytes to the
            // data even though handling only CAN frames, meaning the whole buffer slot is 16 bytes)
            uint32_t free_slot = (c1txqua - 0x180U) >> 4;

            ////// Error checking: should not happen if the hardware is behaving correctly
            if (free_slot >= CAN_TX_QUEUE_SIZE) {
                controller->target_specific.txqua_bad++;
                return false;
            }
            // Copy the frame into the message slot in the controller
            // Layout of TXQ message object:
            uint32_t t[4];
            
            // CAN ID in the controller is in the following format:
            //          31       23       15       7
            //          V        V        V        V
            //          ++++++++ ++++++++ ++++++++ ++++++++
            //          ---BBBBB BBBBBBBB BBBBBAAA AAAAAAAA
            //
            //          A       = 11-bit ID A
            //          B       = 18-bit ID B
            t[1] = (free_slot << 9) | frame->dlc;
            // The ID format for CAN IDs already matches the native CAN ID register layout
            t[0] = frame->canid.id & CAN_ID_ARBITRATION_ID;
            if (can_id_is_extended(frame->canid)) {
                t[1] |= (1U << 4); // Also set the IDE bit
            }
            if (frame->remote) {
                t[1] |= (1U << 5);
            }
            // These words together represent a block of 8 bytes, data byte 0 at the lowest address,
            // data in the controller is in little endian format that matches this. If the host is
            // a big endian CPU then the word must be flipped to little endian first.

            t[2] = mcp25xxfd_convert_bytes(frame->data[0]);
            t[3] = mcp25xxfd_convert_bytes(frame->data[1]);

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
            write_4words(spi_interface, addr, t);

            // Now tell the controller to take the frame and move C1TXQUA
            // Set UINC=1, TXREQ=1
            // Transmit queue control register
            write_word(spi_interface, C1TXQCON, UINC | TXREQ);

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
    // frame if necessary. Even though the errata does not list this register as being corrupted,
    // a read protected by CRC is used.
    can_interface_t *spi_interface = &controller->host_interface;

    uint16_t addr = (uint16_t)read_word_crc(spi_interface, C1TEFUA) + 0x400U;

    // Don't care about the CAN frame ID (we know it already), just SEQ and timestamp are needed
    uint32_t details[2];
    read_words(spi_interface, addr + 4U, details, 2U);
    // The MCP2518FD has a larger SEQ field than the MPC2517FD but we only use a part of it.
    uint8_t seq = (uint8_t)(details[0] >> 9);

    ////// Error checking: should not fail if the hardware is behaving correctly
    // The sequence number may have been corrupted over SPI by noise so we treat it with some
    // suspicion. If it doesn't refer to a valid slot then we dismiss the interrupt without
    // processing it.
    if (seq > CAN_TX_QUEUE_SIZE || !controller->tx_pri_queue.uref_valid[seq]) {
        // Bad SEQ value, keep a count of it and then dismiss the interrupt. This will result
        // in the transmit buffer slot not being cleared, so slowly the buffer will run out of
        // space. But the CRC-protected read should not permit this to fail (it returns 0xffffffffU
        // if the retries all failed, and this will cause the SEQ range check to fail)
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
    // MCP251xFD interrupts are level-sensitive so GPIO must be set to level sensitive; interrupt
    // will be re-raised if still not empty when serviced.
    // Set FSIZE, UINC, TEFTSEN, TEFNEIE
    write_word(spi_interface, C1TEFCON, FSIZE(0x1fU) | UINC | TEFTSEN | TEFNEIE);
}

// Errata: C1TREC read can be corrupted
static void TIME_CRITICAL bus_off_handler(can_controller_t *controller)
{
    can_interface_t *spi_interface = &controller->host_interface;

    // Frames should not be accepted for transmission if not yet in normal mode.
    uint32_t c1trec = read_word_crc(spi_interface, C1TREC);

    if (c1trec & TXBO) {
        // Bus-off will erase the transmit queues, so all the frames we queued are now going to be
        // thrown away, so discard them in the software queues too
        // Record that this happened
        controller->target_specific.bus_off++;

        // Discard all the pending frames
        init_tx_buffers(controller);
    }
    // This could be warning about error passive and other states, but we don't care about those
}

// Errata: C1TBC read can be corrupted
// Errata: C1BDIAG1 read can be corrupted
static void TIME_CRITICAL error_handler(can_controller_t *controller)
{
    can_interface_t *spi_interface = &controller->host_interface;

    // Read the time (won't be very accurate because of the time taken to get here)
    uint32_t timestamp = read_word_crc(spi_interface, C1TBC);        // Get information about the error
    uint32_t c1bdiag1 = read_word_crc(spi_interface, C1BDIAG1);
    // Clear down the diagnostics word
    write_word(spi_interface, C1BDIAG1, 0);

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
    can_interface_t *spi_interface = &controller->host_interface;

    uint16_t addr = (uint16_t)read_word_crc(spi_interface, C1FIFOUA1) + 0x400U;

    // Pick up the frame
    uint32_t r[5];
    read_words(spi_interface, addr, r, 5U);

    // Get to the receive callback as quickly as possible

    // Assemble CAN ID from ID A, ID B and IDE
    can_id_t canid;

    // CAN ID in the controller is in the following format:
    //          31       23       15       7
    //          V        V        V        V
    //          ++++++++ ++++++++ ++++++++ ++++++++
    //          ---BBBBB BBBBBBBB BBBBBAAA AAAAAAAA
    //
    //          A       = 11-bit ID A
    //          B       = 18-bit ID B
    uint32_t arbitration_id = r[0] & CAN_ID_ARBITRATION_ID;
    bool ide = r[1] & (1U << 4);
    // The CAN ID format already matches the native controller format
    canid.id = arbitration_id | (!!ide << CAN_ID_EXT_BIT);

    uint8_t dlc = r[1] & 0xfU;
    bool remote = (r[1] & (1U << 5)) != 0;
    uint8_t id_filter = (r[1] >> 11) & 0x1fU;
    uint32_t timestamp = r[2];
    // The data is pulled in little endian format into a word, and must be written to
    // memory in little endian format with the lowest address byte set to bits 7:0 of
    // the word.
    uint32_t data_0 = mcp25xxfd_convert_bytes(r[3]);
    uint32_t data_1 = mcp25xxfd_convert_bytes(r[4]);

    can_frame_t frame = {.canid = canid,
                         .dlc = dlc,
                         .remote = remote,
                         .data[0] = data_0,
                         .data[1] = data_1,
                         .id_filter = id_filter};

    // Callback is a good place to put any CAN ID or payload triggering function
    can_isr_callback_frame_rx(&frame, timestamp);

    // Mark the frame as taken, ensure that timestamping and the not-empty interrupt are still enabled
    // Set UINC, RXTSEN, TFNRFNIE. This will dismiss the interrupt level (if there are no other interrupts)
    write_word(spi_interface, C1FIFOCON1, UINC | RXTSEN | TFNRFNIE);

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

// TODO Could move to new vectorized interrupt handler using C1VEC
// C1VEC is used to work out which interrupt should be handled. The ICODE field
// is a 7-bit field that indicates the following enabled:
// 
// * 100 1001 C1TXATIF (TEF event)
// * 100 1000 IMVIF (error event)
//  [100 0111 MODIF (mode change)]
//  [100 0110 TBC overflow (timestamping counter rolled over)]
//  [100 0101 RX/TX MAB overflow/underflow (should not be possible)]
//  [100 0011 C1RXOVIF (overflow of receive FIFO, add this to the diagnostics count)]
// * 100 0001 CERRIF (CAN error interrupt: happens on warn/passive/bus off)
// * 000 0001 FIFO transmit or receive event
//  [000 0000 TXQ transmit event]

// Errata: C1INT read can be corrupted
void TIME_CRITICAL mcp25xxfd_irq_handler(can_controller_t *controller)
{
    // Gets an interrupt from the following sources:
    //
    // Sent frame
    // Received frame
    // Bus-off
    // Error
    //

    // The MCP25xxFD uses level-sensitive interrupts but in some cases
    // a GPIO pin on a host microcontroller cannot be set to level-sensitive. Therefore
    // this handler is written to work with either type of GPIO interrupt.
    //
    // It works by servicing all interrupts seen, in a loop, until there are no pending
    // interrupts (and the INT pin from the controller will have gone low) and then
    // the handler quits. An interrupt occuring after this test will latch the internal
    // interrupt flag if the interrupt is raised after this test. It is possible that
    // the CPU latches a falling edge of INT while already in this handler, such that when
    // the handler quits the CPU is re-interrupted by the latched edge even though
    // the original event that raised the interrupt has already been dealt with. In this case the
    // interrupt is spurious and should just be ignored. There is also the possibility of
    // a latched interrupt from a previous session still pending. Again, this will be a
    // spurious interrupt.

    if (controller == NULL) {
        // Spurious interupt from a now-deactivated controller hardware so just return
        return;
    }
    can_interface_t *spi_interface = &controller->host_interface;

    // If the level-sensitive interrupt pin is not being asserted then this is spurious
    if (!mcp25xxfd_spi_gpio_irq_asserted(spi_interface)) {
        controller->target_specific.spurious++;
    }

    //////// INTERRUPT EVENT HANDLING LOOP ////////
    while (mcp25xxfd_spi_gpio_irq_asserted(spi_interface)) {
        // Read C1INT and then handle interrupts
        uint32_t events = read_word_crc(spi_interface, C1INT);
        // While the interrupt line is asserted, handle pending events
        uint16_t dismiss = 0;

        if (events & IVMIF) {               // IVMIF to detect error frames
            // Dismisses IVMIF interrupt (but not others)
            dismiss |= IVMIF;
            error_handler(controller);
        }
        if (events & CERRIF) {              // CERRIF to detect bus off transition
            // Dismisses CERRIF interrupt (but not others)
            dismiss |= CERRIF;
            bus_off_handler(controller);
        }
        if (events & TEFIF) {               // TEFIF (i.e. TEF event)
            // Dismissal of this event is implicit by emptying the transmit FIFO
            tx_handler(controller);
        }
        if (events & RXIF) {                // RXIF (i.e. received frame into the FIFO)
            // Dismissal of this event is implicit by emptying the receive FIFO
            rx_handler(controller);
        }
        // Dismiss all the explicit handled events
        enable_controller_interrupts(spi_interface, dismiss);
    }
}

// Errata: C1TBC read can be corrupted
uint32_t TIME_CRITICAL can_get_time(can_controller_t *controller)
{
    can_interface_t *spi_interface = &controller->host_interface;

    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    uint32_t timebase = read_word_crc(spi_interface, C1TBC);
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);

    return timebase;
}

// Errata: C1TREC read can be corrupted
can_status_t TIME_CRITICAL can_get_status(can_controller_t *controller)
{
    can_interface_t *spi_interface = &controller->host_interface;
 
    can_status_t status = {.status = 0};

    if (controller != NULL) {
        mcp25xxfd_spi_gpio_disable_irq(spi_interface);
        uint32_t trec = read_word_crc(spi_interface, C1TREC);
        mcp25xxfd_spi_gpio_enable_irq(spi_interface);
        status.status = trec;
    }
    return status;
}

void can_status_request_recover(can_controller_t *controller)
{
    // Empty function on the MCP251xFD: hardware
    // automatically recovers from Bus Off
}

// Create the CAN instance and initialize the controller
// The caller should already have bound the controller to the interface via
// the host-specific API call for binding.
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

    can_interface_t *spi_interface = &controller->host_interface;

    if (spi_interface->magic != 0x1e5515f0U) {
        return CAN_ERC_NO_INTERFACE;
    }

    uint8_t brp;
    uint8_t tseg1;
    uint8_t tseg2;
    uint8_t sjw;

    if (all_filters != CAN_NO_FILTERS && all_filters->n_filters > CAN_MAX_ID_FILTERS) {
        return CAN_ERC_RANGE;   // Only up to 32 filters possible
    }

    // Ensure no old pending interrupts
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);

    // Sets up SPI channel, ensures chip not selected
    mcp25xxfd_spi_pins_init(spi_interface);

    // Do a hard reset if requested, once the SPI has been set up: this generates
    // a special SPI reset command that causes the MCP251xFD to be reset
    if (options & CAN_OPTION_HARD_RESET) {
        hard_reset(spi_interface);
    }

    // Set controller's clock and I/O pins (and set TX pin to open drain if requested)
    set_controller_pins(spi_interface, options & CAN_OPTION_OPEN_DRAIN);

    // Check that IOCON was set correctly for TX open drain and return an error if it
    // is not set.
    uint32_t iocon = read_word_crc(spi_interface, IOCON);
    if ((options & CAN_OPTION_OPEN_DRAIN) && !(iocon & TXCANOD)) {
        return CAN_ERC_BAD_WRITE;
    }

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
    if (!set_controller_mode_config(spi_interface)) {
        return CAN_ERC_BAD_INIT;
    }

    // Disable all the ID filters (which might not be disabled if we come into this with the controller
    // having already been running and not reset)
    uint32_t filter_control[8];
    for (uint32_t i = 0; i < 8U; i++) {
        filter_control[i] = 0;
        write_word(spi_interface, C1FLTCON(i), 0);
    }

    // Add in the filters
    if (all_filters != CAN_NO_FILTERS) {
        for (uint32_t idx = 0; idx < all_filters->n_filters; idx++) {
            can_id_filter_t *filter = &all_filters->filter_list[idx];
            if (filter->enabled) {
                // Enables the filter and sets it to direct frames to RX FIFO 1
                filter_control[idx >> 2] |= (0x81U << ((idx & 0x03U) << 3));
                // Sets the mask/match registers accordingly
                write_word(spi_interface, C1FLTOBJ(idx), all_filters->filter_list[idx].fltobj);
                write_word(spi_interface, C1MASK(idx), all_filters->filter_list[idx].mask);
            }
        }
    } else {
        // Set filter 0 to match-all and direct frames to the RX FIFO
        filter_control[0] = 0x81U;
        write_word(spi_interface, C1FLTOBJ(0), 0);
        write_word(spi_interface, C1MASK(0), 0);
    }

    // Enable the appropriate filters
    for (uint32_t i = 0; i < 8U; i++) {
        write_word(spi_interface, C1FLTCON(i), filter_control[i]);
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
    controller->target_specific.bus_off = 0;
    controller->target_specific.spurious = 0;
    controller->target_specific.crc_bad = 0;

    if (!set_controller_mode(spi_interface, mode, brp, tseg1, tseg2, sjw)) {
        // Won't go into the requested mode, return an error
        return CAN_ERC_BAD_INIT;
    }

    // Enable SPI interrupts on controller IRQ pin. GPIO interrupts will already have been enabled
    // by the initialization of the pins in mcp25xxfd_spi_pins_init()
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);

    return CAN_ERC_NO_ERROR;
}

can_errorcode_t TIME_CRITICAL can_send_frame(can_controller_t *controller, const can_frame_t *frame, bool fifo)
{
    if (controller == NULL) {
        return CAN_ERC_BAD_INIT;
    }
    can_interface_t *spi_interface = &controller->host_interface;

    // Going to put a frame into the transmit queue or the transmit FIFO
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    bool queued = send_frame(controller, frame, fifo);
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);

    return queued ? CAN_ERC_NO_ERROR : CAN_ERC_NO_ROOM;
}

uint32_t TIME_CRITICAL can_recv_as_bytes(can_controller_t *controller, uint8_t *dest, size_t n_bytes)
{
    // There is a single controller set up
    if (controller == NULL) {
        // If the controller has not been initialized then return no bytes
        return 0;
    }
    if (n_bytes < NUM_RX_EVENT_BYTES) {
        return 0;
    }

    can_interface_t *spi_interface = &controller->host_interface;

    uint32_t result;
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    if (CAN_RX_FIFO_SIZE - controller->rx_fifo.free) {
        pop_rx_event_as_bytes(controller, dest);
        result = NUM_RX_EVENT_BYTES;
    }
    else {
        result = 0;
    }
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);
    return result;
}

// Receives one event from the receive FIFO (events can be CAN frames, CAN errors, FIFO overflow)
bool TIME_CRITICAL can_recv(can_controller_t *controller, can_rx_event_t *event)
{
    if (controller == NULL) {
        // If the controller has not been initialized then return nothing received
        return false;
    }

    can_interface_t *spi_interface = &controller->host_interface;
    bool result;
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    uint32_t max_num_events = CAN_RX_FIFO_SIZE - controller->rx_fifo.free;
    if (max_num_events > 0) {
        pop_rx_event(controller, event);
        result = true;
    }
    else {
        result = false;
    }
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);

    return result;
}

// Return number of events waiting in the RX FIFO.`
uint32_t TIME_CRITICAL can_recv_pending(can_controller_t *controller)
{
    if (controller == NULL) {
        // If the controller has not been initialized then return nothing
        return 0;
    }
    // It's probably OK to access this with interrupt concurrency because it's an atomic word, but
    // other calls extend the interrupt lock around this to guard against an empty queue being
    // accessed.
    return CAN_RX_FIFO_SIZE - controller->rx_fifo.free;
}

bool TIME_CRITICAL can_recv_tx_event(can_controller_t *controller, can_tx_event_t *event)
{
    if (controller == NULL) {
        // If the controller has not been initialized then nothing has returned
        return false;
    }
    can_interface_t *spi_interface = &controller->host_interface;

    // tx_event_fifo.free is probably to access outside the ISR because it's likely to compile
    // to an atomic word and can only decrease so num_events can only increase
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    bool result;
    uint32_t max_num_events = CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
    if (max_num_events > 0) {
        pop_tx_event(controller, event);
        result = true;
    }
    else {
        result = false;
    }
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);

    return result;
}

uint32_t TIME_CRITICAL can_recv_tx_event_as_bytes(can_controller_t *controller, uint8_t *dest, size_t n_bytes)
{
    if (controller == NULL) {
        // If the controller has not been initialized then return no bytes
        return 0;
    }
    can_interface_t *spi_interface = &controller->host_interface;

    // tx_event_fifo.free is probably OK to access outside the ISR because it's likely to compile
    // to an atomic word and can only decrease so num_events can only increase
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);
    uint32_t recvd;
    uint32_t max_num_events = CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
    if (max_num_events > 0) {
        pop_tx_event_as_bytes(controller, dest, n_bytes);
        recvd = NUM_TX_EVENT_BYTES;
    }
    else {
        recvd = 0;
    }
    mcp25xxfd_spi_gpio_enable_irq(spi_interface);
    // Return the number of bytes actually received
    return recvd;
}

// Return number of events waiting in the TX event FIFO
uint32_t TIME_CRITICAL can_recv_tx_events_pending(can_controller_t *controller)
{
    if (controller == NULL) {
        // If the controller has not been initialized then return no events pending
        return 0;
    }

    // It's OK to access this with interrupt concurrency because it's an atomic word
    return CAN_TX_EVENT_FIFO_SIZE - controller->tx_event_fifo.free;
}

uint32_t TIME_CRITICAL can_get_send_space(can_controller_t *controller, bool fifo)
{
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

bool TIME_CRITICAL can_is_space(can_controller_t *controller, uint32_t n_frames, bool fifo)
{
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

void can_stop_controller(can_controller_t *controller)
{
    if (controller == NULL) {
        // Already not started
        return;
    }
    can_interface_t *spi_interface = &controller->host_interface;

    // Disable SPI interrupts from the GPIO pin so this
    // runs without interference from the ISRs.
    mcp25xxfd_spi_gpio_disable_irq(spi_interface);

    // Request config mode to stop the device transmitting if it was
    // (will ensure that queues are emptied and interrupts will not be
    // generated)
    write_word(spi_interface, C1CON, REQOP(4U));

    // Stop all controller interrupts at source, dismiss all pending interrupts
    write_word(spi_interface, C1INT, 0);

    // Remove all the items in the transmit queue (if enabled) so that
    // there are no user references active (will permit them to be garbage collected
    // if they are defined within MicroPython)
    init_tx_buffers(controller);

    // Note that for edge-sensitive devices there is a possibility that the
    // CPU still has an edge interrupt pending (because it will not be cleared
    // by the above, since the flag is a latch within the CPU). In this case,
    // there may be a spurious interrupt generated if/when GPIO interrupt is
    // enabled in the future.
}
