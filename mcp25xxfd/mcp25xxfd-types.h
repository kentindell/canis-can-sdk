// Copyright 2020-2023 Canis Automotive Labs (canislabs.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
// to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of
// the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MCP25xxFD_TYPES_H
#define MCP25xxFD_TYPES_H

#if defined(HOST_CANPICO)
#include "rp2/mcp25xxfd-rp2.h"
#elif defined(HOST_GENERIC)
#define TIME_CRITICAL /* */
typedef struct {
    uint32_t _generic;
} can_interface_t;
#else
#error "Unknown host"
#endif
// Other host platforms with the MCP25xxFD will be included here

// Size of data structures use for the CAN driver
#ifndef CAN_HW_TX_QUEUE_SIZE
#define CAN_HW_TX_QUEUE_SIZE            (20U)           // For the MCP25xxFD in FD mode
#if (CAN_HW_TX_QUEUE_SIZE > 20)
#error "CAN FD on the MCP25xxFD limits TX queue to 20 slots"
#endif
#endif // CAN_HW_TX_QUEUE_SIZE
#ifndef UREF_HASH_TABLE_SIZE
#define UREF_HASH_TABLE_SIZE            (22U)
#endif            

#ifndef CAN_HW_TEF_SIZE
#define CAN_HW_TEF_SIZE                 (8U)
#endif

#ifndef CAN_HW_RX_FIFO_SIZE
#define CAN_HW_RX_FIFO_SIZE             (6U)
#endif

// Software TX FIFO behind the hardware TX queue
#ifndef CAN_TX_FIFO_SIZE
#define CAN_TX_FIFO_SIZE                (32U)
#endif

// Number of frames in the software receive FIFO
#ifndef CAN_RX_FIFO_SIZE
#define CAN_RX_FIFO_SIZE                (128U)          // Must be less than 256
#endif

// Number of elements in the software TX event FIFO
#ifndef CAN_TX_EVENT_FIFO_SIZE
#define CAN_TX_EVENT_FIFO_SIZE          (128U)          // Must be less than 256
#endif

#define CAN_MAX_ID_FILTERS              (32U)

// Target-specific data types for MCP25xxFD CAN controller
typedef struct {
    uint32_t seq_bad;                               // Number of times the SEQ field was bad
    uint32_t txqua_bad;                             // C1TXQUA read back out of range or otherwise bad
    uint32_t txqsta_bad;                            // C1TXSTA read back showing queue full
    uint32_t bus_off;                               // Count of number of times device has been bus off
    uint32_t spurious;                              // Count of spurious interrupts
    uint32_t crc_bad;                               // Count of read CRC failure    
} can_controller_target_t;

typedef struct {
    uint32_t mask;
    uint32_t fltobj;
    bool enabled;
} can_id_filter_t;

#endif // MCP25xxFD_TYPES_H
