// Copyright 2020-2022 Canis Automotive Labs (canislabs.com)
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

#ifndef MCP2517FD_TYPES_H
#define MCP2517FD_TYPES_H

#include "../canapi.h"

#if defined(HOST_CANPICO)
#include "rp2/mcp2517fd-rp2.h"
#else
#error "Unknown host"
#endif
// Other host platforms with the MCP2517FD will be included here

// Target-specific data types for MCP2517FD CAN controller

typedef struct {
    uint32_t seq_bad;                               // Number of times the SEQ field was bad
    uint32_t txqua_bad;                             // C1TXQUA read back out of range or otherwise bad
    uint32_t txqsta_bad;                            // C1TXSTA read back showing queue full
} can_controller_target_t;

typedef struct {
    uint32_t mask;
    uint32_t fltobj;
    bool enabled;
} can_id_filter_t;

#define CAN_MAX_ID_FILTERS                          (32U)

#endif // MCP2517FD_TYPES_H
