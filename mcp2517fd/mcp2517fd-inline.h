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

#ifndef MCP2517FD_INLINE_H
#define MCP2517FD_INLINE_H

#include "../canapi.h"

// Status return
INLINE bool can_status_is_bus_off(can_status_t status)
{
    return status.status & (1U << 21);
}

INLINE bool can_status_is_error_passive(can_status_t status)
{
    return (status.status & (1U << 20)) || (status.status & (1U << 19));
}

INLINE bool can_status_is_error_warn(can_status_t status)
{
    return (status.status & (1U << 18)) || (status.status & (1U << 17));
}

INLINE uint8_t can_status_get_tec(can_status_t status)
{
    return (status.status >> 8) & 0xffU;
}

INLINE uint8_t can_status_get_rec(can_status_t status)
{
    return status.status & 0xffU;
}

// CAN error types
INLINE bool can_error_is_crc(can_error_t *error)
{
    return error->details & (1U << 21);
}

INLINE bool can_error_is_stuff(can_error_t *error)
{
    return error->details & (1U << 20);
}

INLINE bool can_error_is_form(can_error_t *error)
{
    return error->details & (1U << 19);
}

INLINE bool can_error_is_ack(can_error_t *error)
{
    return error->details & (1U << 18);
}

INLINE bool can_error_is_bit1(can_error_t *error)
{
    return error->details & (1U << 17);
}

INLINE bool can_error_is_bit0(can_error_t *error)
{
    return error->details & (1U << 16);
}

INLINE bool can_error_is_bus_off(can_error_t *error)
{
    return error->details & (1U << 23);
}

INLINE uint32_t can_error_get_frame_cnt(can_error_t *error)
{
    return error->details & 0xffffU;
}

// Filter construction
INLINE void can_make_id_filter(can_id_filter_t *filter, can_id_t canid)
{
    uint32_t arbitration_id = canid.id & CAN_ID_ARBITRATION_ID;

    if (canid.id & (1U << CAN_ID_EXT_BIT)) {
        filter->fltobj = (1U << 30) | arbitration_id;
        filter->mask = 0x5fffffffU;
    }
    else {
        filter->fltobj = arbitration_id;
        filter->mask = 0x400007ffU;
    }
    filter->enabled = true;
}

INLINE void can_make_id_filter_all(can_id_filter_t *filter)
{
    filter->fltobj = 0;
    filter->mask = 0;
    filter->enabled = true;
}

INLINE void can_make_id_filter_disabled(can_id_filter_t *filter)
{
    filter->fltobj = 0;
    filter->mask = 0;
    filter->enabled = false;
}

INLINE bool can_id_filter_is_all(can_id_filter_t *filter)
{
    return filter->fltobj == 0 && filter->mask == 0;
}

INLINE bool can_id_filter_is_extended(can_id_filter_t *filter)
{
    return filter->fltobj & (1U << 30);
}

INLINE uint32_t can_id_filter_get_mask(can_id_filter_t *filter)
{
    bool ide = can_id_filter_is_extended(filter);
    if (ide) {
        return filter->mask & CAN_ID_ARBITRATION_ID;
    }
    else {
        return filter->mask & 0x7ffU;
    }
}

INLINE uint32_t can_id_filter_get_match(can_id_filter_t *filter)
{
    bool ide = can_id_filter_is_extended(filter);
    if (ide) {
        return filter->fltobj & CAN_ID_ARBITRATION_ID;
    }
    else {
        return filter->fltobj & 0x7ffU;
    }
}

INLINE void can_make_id_filter_masked(can_id_filter_t *filter, bool ide, uint32_t arbitration_id_match, uint32_t arbitration_id_mask)
{
    if (ide) {
        filter->fltobj = (1U << 30) | (arbitration_id_match & CAN_ID_ARBITRATION_ID);
        filter->mask = 0x40000000U | (arbitration_id_mask & CAN_ID_ARBITRATION_ID);
    }
    else {
        filter->fltobj = arbitration_id_match & 0x7ffU;
        filter->mask = 0x40000000U | (arbitration_id_mask & 0x7ffU);
    }
    filter->enabled = true;
}

#endif // MCP2517FD_INLINE_H
