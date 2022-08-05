// Copyright 2020-2022 Canis Automotive Labs (https://canislabs.com)
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

// TODO add low-power standby mode to API (put controller into standby, put transceiver into standby)
// TODO add multiple FIFOs feeding in to the priority queue
// TODO parameterize API to allow multiple CAN controller instances

#ifndef CANAPI_H
#define CANAPI_H

#include <stdint.h>

#if defined(MCP2517FD)
#include "mcp2517fd/mcp2517fd-types.h"
#else
#error "Unknown CAN controller"
#endif

// Size of data structures use for the CAN driver
#ifndef CAN_TX_QUEUE_SIZE
#define CAN_TX_QUEUE_SIZE               (32U)           // Must be less <= 32
#endif

#ifndef CAN_TX_FIFO_SIZE
#define CAN_TX_FIFO_SIZE                (32U)           // Must be less <= 32
#endif

#ifndef CAN_RX_FIFO_SIZE
#define CAN_RX_FIFO_SIZE                (128U)          // Must be less than 256
#endif

#ifndef CAN_TX_EVENT_FIFO_SIZE
#define CAN_TX_EVENT_FIFO_SIZE          (128U)          // Must be less than 256
#endif

#if (CAN_TX_QUEUE_SIZE > 32)
#error "CAN_TX_QUEUE_SIZE must be <= 32"
#endif

#if (CAN_TX_FIFO_SIZE > 32)
#error "CAN_TX_FIFO_SIZE must be <= 32"
#endif

#if (CAN_RX_FIFO_SIZE > 255)
#error "CAN_RX_FIFO_SIZE must be < 256"
#endif

#if (CAN_TX_EVENT_FIFO_SIZE > 255)
#error "CAN_TX_EVENT_FIFO_SIZE must be < 256"
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// DATA STRUCTURES /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is a user-defined type that is included in CAN frames and passed back to the
// application when the frame is transmitted, both in the transmission event queue and also
// in the transmission interrupt.

/// @brief A user-defined reference associated with a frame that the API returns back
typedef struct {
    void *ref;
} can_uref_t;

/// @brief The value to be used for a null user reference
extern can_uref_t can_uref_null;

/// @brief API return codes
typedef enum {
    CAN_ERC_NO_ERROR = 0,                               // OK
    CAN_ERC_BAD_BITRATE,                                // Baud rate settings are not legal
    CAN_ERC_RANGE,                                      // Range error on parameters
    CAN_ERC_BAD_INIT,                                   // Can't get the controller to initialize
    CAN_ERC_NO_ROOM,                                    // No room
    CAN_ERC_NO_ROOM_PRIORITY,                           // No room in the transmit priority queue
    CAN_ERC_NO_ROOM_FIFO,                               // No room in the transmit FIFO queue
} can_errorcode_t;

/// @brief CAN error frame details
typedef struct {
    uint32_t details;                                   // Target-specifc details of a CAN error
} can_error_t;

/// @brief CAN ID in a target-independent definition
typedef struct {
    uint32_t id;
} can_id_t;


////////////////////////////////////////////// CAN ID //////////////////////////////////////////////


// IDs stored in a 32-bit integer, with IDE flag at bit 29, and
// the arbitration ID at bits 28:18 (standard ID) or 28:0 (extended ID)
#define CAN_ID_EXT_BIT                                  (29U)
#define CAN_ID_ARBITRATION_ID                           (0x1fffffffU)

/// @brief Return a CAN frame instance
/// @param extended is true if the arbitration ID is 29-bit, otherwise 11-bit
/// @param arbitration_id arbitration ID
INLINE can_id_t can_make_id(bool extended, uint32_t arbitration_id)
{
    can_id_t ret;
    if (extended) {
        ret.id = (arbitration_id & CAN_ID_ARBITRATION_ID) | (1U << CAN_ID_EXT_BIT);
    }
    else {
        ret.id = arbitration_id & 0x7ffU;
    }
    return ret;
}

/// @brief Returns true if the ID is an extended 29-bit ID
/// @param canid The CAN ID in standard API format
INLINE bool can_id_is_extended(can_id_t canid)
{
    return canid.id & (1U << CAN_ID_EXT_BIT);
}

/// @brief Returns the arbitration ID, a 29-bit value if the CAN is extended, 11-bit otherwise
INLINE uint32_t can_id_get_arbitration_id(can_id_t canid)
{
    return canid.id & CAN_ID_ARBITRATION_ID;
}

/// @brief Options when setting up the CAN controller
typedef enum {
    CAN_OPTIONS_NONE = 0,
    CAN_OPTION_REJECT_REMOTE        = (1U << 0),    // Don't store remote frames in the receive queue
    CAN_OPTION_HARD_RESET           = (1U << 1),    // Hard reset the controller hardware when setting it up
    CAN_OPTION_RECV_ERRORS          = (1U << 2),    // Store CAN error frames in the receive queue
    CAN_OPTION_OPEN_DRAIN           = (1U << 3),    // Set the TX pin to open drain mode (target-specific)
    CAN_OPTION_RECORD_TX_EVENTS     = (1U << 4),    // Record frame transmit events
    CAN_OPTION_REJECT_OVERFLOW      = (1U << 5),    // Don't store overflow events in the receive FIFO
} can_options_t;

/// @brief Standard CAN bit rate profiles
typedef enum {
    CAN_BITRATE_500K_75 = 0,    // 500kbit/sec 75% sample (default)    
    CAN_BITRATE_250K_75,        // 250kbit/sec 75% sample point 
    CAN_BITRATE_125K_75,        // 125kbit/sec 75% sample point
    CAN_BITRATE_1M_75,          // 1Mbit/sec 75% sample point
    CAN_BITRATE_500K_50,        // 500kbit/sec 50% sample (default)
    CAN_BITRATE_250K_50,        // 250kbit/sec 50% sample point 
    CAN_BITRATE_125K_50,        // 125kbit/sec 50% sample point
    CAN_BITRATE_1M_50,          // 1Mbit/sec 50% sample point
    CAN_BITRATE_2M_50,          // 2Mbit/sec 50% sample point NON STANDARD
    CAN_BITRATE_4M_90,          // 4Mbit/sec 50% sample point NON STANDARD
    CAN_BITRATE_2_5M_75 ,       // 2.5Mbit/sec 75% sample point NON STANDARD
    CAN_BITRATE_2M_80,          // 2Mbit/sec 80% sample point NON STANDARD
    CAN_BITRATE_500K_875,       // 500kbit/sec 87.5% sample
    CAN_BITRATE_250K_875,       // 250kbit/sec 87.5% sample point (J1939, CANOpen) 
    CAN_BITRATE_125K_875,       // 125kbit/sec 87.5% sample point
    CAN_BITRATE_1M_875,         // 1Mbit/sec 85.5% sample point
    CAN_BITRATE_CUSTOM,         // A custom profile (other parameters must be defined)
} can_profile_t;

/// @brief Structure holding the profile and other parameters
typedef struct {
    can_profile_t profile;
    uint8_t brp;                // Baud rate prescaler (0 = /1)
    uint8_t tseg1;              // CAN TSEG1 - 1
    uint8_t tseg2;              // CAN TSGE2 - 1
    uint8_t sjw;                // CAN SJW - 1
} can_bitrate_t;

/// @brief Event types
typedef enum {
    CAN_EVENT_TYPE_TRANSMITTED_FRAME = 0,   // Frame transmitted
    CAN_EVENT_TYPE_RECEIVED_FRAME,          // Frame received
    CAN_EVENT_TYPE_OVERFLOW,                // FIFO overflow happened
    CAN_EVENT_TYPE_CAN_ERROR                // CAN error frame received
} can_event_type_t;

/// @brief Controller modes
typedef enum {
    CAN_MODE_NORMAL,                        // Can send and receive CAN frames
    CAN_MODE_LISTEN_ONLY,                   // Listen only (don't acknowledge CAN frames)
    CAN_MODE_ACK_ONLY,                      // Listen but do acknowledge CAN frames
    CAN_MODE_OFFLINE                        // Do not listen or transmit
} can_mode_t;

//////////////////////////////////////////// CAN status ////////////////////////////////////////////

/// @brief Status of CAN controller
typedef struct {
    uint32_t status;                        // Target-specific status information
} can_status_t;

/// @brief Ask the CAN controller for its status
/// @exception This call is non-reentrant
can_status_t can_get_status(void);

/// @brief Ask the CAN controller to recover from Bus Off
/// @exception This call is non-reentrant
void can_status_request_recover(void);

// These are target-specific but inlined for performance

/// @brief Returns true if the CAN controller is in Bus Off state
/// @param status Status returned by can_get_status()
static bool can_status_is_bus_off(can_status_t status);

/// @brief Returns true if the CAN controller is in ErrorPassive state
/// @param status Status returned by can_get_status()
static bool can_status_is_error_passive(can_status_t status);

/// @brief Returns true if the CAN controller is in Warn state
/// @param status Status returned by can_get_status()
static bool can_status_is_error_warn(can_status_t status);

/// @brief Returns the Transmit Error Counter of the controller
/// @param status Status returned by can_get_status()
/// @return 8-bit value of TEC (if state is Bus Off then TEC is > 255)
static uint8_t can_status_get_tec(can_status_t status);

/// @brief Gets the Receive Error Counter of the controller
/// @param status Status returned by can_get_status()
/// @return 8-bit value of REC (if state is Bus Off then REC is > 255)
static uint8_t can_status_get_rec(can_status_t status);

/// @brief Structure holding the details of a CAN frame
typedef struct  {
    can_uref_t uref;    // User-defined data for callbacks to use
    can_id_t canid;     // CAN ID
    uint32_t data[2];   // Payload stored as two words (but in memory treated as bytes)
    uint8_t dlc;        // DLC (0-15)
    uint8_t id_filter;  // Filter hit: index of ID filter that accepted the frame (received only)
    bool remote;        // Frame is remote
} can_frame_t;

/// @brief stores the details of an overflow in the receive FIFO
typedef struct {
    uint32_t frame_cnt;     // How many frames have been discarded
    uint32_t error_cnt;     // How many error frames have been discarded
} can_rx_overflow_event_t;

/// @brief A receive event (can be a CAN frame, a CAN error or an overflow)
typedef struct {
    union {
        can_frame_t frame;                              // Received CAN frame
        can_rx_overflow_event_t overflow;               // Overflow event (indicates how frames/errors dropped)
        can_error_t error;                              // Error frame
    } event;
    uint32_t timestamp;                                 // SOF, microseconds (always valid)
    can_event_type_t event_type;                        // Which of the three events this is
} can_rx_event_t;

/// @brief A transmit event (can be a CAN frame or an overflow)
typedef struct {
    union {
        can_uref_t uref;                                // Supplied via API by user application
        uint32_t overflow_cnt;                          // Counts transmit event queue overflow events
    } info;
    uint32_t timestamp;                                 // SOF, microseconds
    can_event_type_t event_type;                        // Indicates if a frame transmit, error or overflow
} can_tx_event_t;

/// @brief A list of ID acceptance filters
typedef struct {
    can_id_filter_t *filter_list;   // A list of filters (with target-specific definition)
    uint8_t n_filters;              // Number of filters in the list (must be <= CAN_MAX_ID_FILTERS)
} can_id_filters_t;

/// @brief CAN controller structure holding the queues
typedef struct {
    // Receive FIFO, containing details of CAN frames, errors and overflow events
    struct {
        can_rx_event_t rx_events[CAN_RX_FIFO_SIZE];
        uint8_t head_idx;                               // Index into first used frame in the FIFO
        uint8_t tail_idx;                               // Index into first free frame at the back of the FIFO
        uint8_t free;                                   // Number of free slots in the queue
        uint8_t dropped_event_idx;                      // Used when the FIFO has overrun
    } rx_fifo;       

    // Frame transmit FIFO (feeds into priority queue)
    struct {
        can_frame_t frames[CAN_TX_FIFO_SIZE];           // Queued frames in transmit FIFO
        uint8_t head_idx;                               // Index into first used frame in the FIFO
        uint8_t tail_idx;                               // Index into first free frame at the back of the FIFO
        uint8_t num_free_slots;                         // Number of free slots in the queue
    } tx_fifo;

    // Software shadow structure for the transmit queue
    //
    // When a frame is queued, a user reference to it is put into this array and the sequence number assigned to
    // the index. That index is used later on transmission to take it out and create a transmit event,
    // and also a callback to a user handler to deal with the transmission of the frame.
    struct {
        can_uref_t uref[CAN_TX_QUEUE_SIZE];             // User reference of frame in transmit priority queue
        bool uref_valid[CAN_TX_QUEUE_SIZE];             // Slot is used (for error tolerance)
        uint32_t num_free_slots;                        // Number of free slots in the queue
        uint8_t fifo_slot;                              // If fifo_slot is < MCP2517FD_TX_QUEUE_SIZE true then indicates where the FIFO frame is
    } tx_pri_queue;

    // Transmit event FIFO that records the timestamp and the user reference of the transmitted frame
    struct {
        can_tx_event_t events[CAN_TX_EVENT_FIFO_SIZE];  // Transmit events
        uint8_t head_idx;                               // Index into first used frame in the FIFO
        uint8_t tail_idx;                               // Index into first free frame at the back of the FIFO
        uint8_t free;                                   // Number of free slots in the queue
        uint8_t dropped_event_idx;
    } tx_event_fifo;

    can_mode_t mode;
    uint16_t options;

    // Target-specific data
    can_controller_target_t target_specific;
} can_controller_t;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// FUNCTIONS ////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(MCP2517FD)
#include "mcp2517fd/mcp2517fd-inline.h"
#else
#error "Unknown CAN controller"
#endif

////////////////////////////////////////// CAN controller //////////////////////////////////////////

/// @brief Initialize the CAN controller
/// @param controller A pointer to the CAN controller structure (application must allocate this)
/// @param bitrate A decription of the CAN controller bit rate
/// @param all_filters The list of ID acceptance filters or CAN_NO_FILTERS if none (i.e. accept everything)
/// @param mode The controller mode (send/receive, listen-only, etc.)
/// @param options a bit mask of option values
/// @exception This call is non-reentrant
can_errorcode_t can_setup_controller(can_controller_t *controller,
                                     const can_bitrate_t *bitrate,
                                     const can_id_filters_t *all_filters, 
                                     can_mode_t mode,
                                     uint16_t options);

/// @brief Stop the CAN controller operating
void can_stop_controller(void);

/// @brief Get the bitmask of options used to initialize the CAN controller
INLINE uint16_t can_controller_get_options(const can_controller_t *controller)
{
    return controller->options;
}

//////////////////////////////////////////// CAN frame /////////////////////////////////////////////
/// @brief Put the CAN frame into the queue (priority or FIFO)
/// @param fifo If true, puts the frame into the FIFO queue that feeds into the priority queue
/// @exception This call is non-reentrant
can_errorcode_t can_send_frame(const can_frame_t *frame, bool fifo);

/// @brief Returns true if there is space to send a number of frames
/// @param n_frames The number of frames to send
/// @param fifo If the frames are to go into the FIFO queue
/// @exception This call is non-reentrant
bool can_is_space(uint32_t n_frames, bool fifo);

/// @brief Returns true if the frame has an extended ID
INLINE bool can_frame_is_extended(const can_frame_t *frame)
{
    return (frame->canid.id & (1U << CAN_ID_EXT_BIT));
}

/// @brief Returns true if the frame is a CAN remote frame
INLINE bool can_frame_is_remote(const can_frame_t *frame)
{
    return frame->remote;
}

/// @brief Returns the arbitration ID of the frame
INLINE uint32_t can_frame_get_arbitration_id(const can_frame_t *frame)
{
    return frame->canid.id & CAN_ID_ARBITRATION_ID;
}

/// @brief Returns a pointer to the payload of the frame
INLINE uint8_t *can_frame_get_data(const can_frame_t *frame)
{
    return (uint8_t *)frame->data;
}

/// @brief Returns the number of bytes in the payload
INLINE size_t can_frame_get_data_len(const can_frame_t *frame)
{
    uint8_t len = (frame->dlc & 0x8U) ? 8U : frame->dlc;
    if (can_frame_is_remote(frame)) {
        return 0;
    }
    else {
        return len;
    }
}

/// @brief Returns the 4-bit DLC field of the CAN frame
INLINE uint8_t can_frame_get_dlc(const can_frame_t *frame)
{
    return frame->dlc;
}

/// @brief Returns the index of the ID acceptance filter that matched the frame (default 0)
INLINE uint8_t can_frame_get_id_filter(const can_frame_t *frame)
{
    return frame->id_filter;
}

/// @brief Get a pointer to a block of words that contains the payload
INLINE uint32_t *can_frame_get_data_words(can_frame_t *frame)
{
    return frame->data;
}

/// @brief Get the user reference for a CAN frame
/// @param frame A pointer to a CAN frame
INLINE can_uref_t can_frame_get_uref(can_frame_t *frame)
{
    return frame->uref;
}

/// @brief Create a CAN frame
/// @param frame A pointer to the application-allocated structure
/// @param arbitration_id The 29-bit or 11-bit CAN ID
/// @param ide True if the arbitration ID is 29-bit
/// @param dlc The 4-bit DLC value for the frame
/// @param data A pointer to the bytes containing the payload
/// @param remote The frame is a remote frame
INLINE void can_make_frame(can_frame_t *frame, bool ide, uint32_t arbitration_id, uint8_t dlc, const uint8_t *data, bool remote)
{
    // Limit the DLC to 4 bits in case the caller has made an error
    dlc &= 0xfU;

    // Fill out the frame details
    frame->canid = can_make_id(ide, arbitration_id);
    frame->dlc = dlc;
    frame->remote = remote;
    frame->id_filter = 0;
    frame->uref = can_uref_null;    // User can fill this in later if necessary
    // Copy the correct number of data bytes in
    uint8_t *dst = (uint8_t *)frame->data;
    switch(dlc) {
        // Fallthrough case statement to copy a CAN data from a block of bytes
        // The source for this may not be word-aligned and memory beyond the specified
        // number of bytes may not be accessible, so copy exactly the number of bytes.
        // Because this is inline, the compiler may make a much better job of copying
        // in cases where a word copy will do.
        case 15U:
        case 14U:
        case 13U:
        case 12U:
        case 11U:
        case 10U:
        case 9U:
        case 8U:
            dst[7] = data[7];
        case 7U:
            dst[6] = data[6];
        case 6U:
            dst[5] = data[5];
        case 5U:
            dst[4] = data[4];
        case 4U:
            dst[3] = data[3];
        case 3U:
            dst[2] = data[2];
        case 2U:
            dst[1] = data[1];
        case 1U:
            dst[0] = data[0];
        default:
            break;
    }
}

/// @brief Set the user reference for a frame
INLINE void can_frame_set_uref(can_frame_t *frame, void *ref)
{
    frame->uref.ref = ref;
}

/// @brief Read a 32-bit endian word into a uint32_t
#define CAN_READ_BIG_ENDIAN_WORD(buf)        ((((uint32_t)(buf)[0]) << 24) |  \
                                             (((uint32_t)(buf)[1]) << 16) |  \
                                             (((uint32_t)(buf)[2]) << 8) |  \
                                             ((uint32_t)(buf)[3]))

/// @brief Creates a frame from a block of bytes
/// @param frame Pointer to a frame structure allocated by the application
/// @param src A pointer to bytes from where the frame will be created
INLINE void can_make_frame_from_bytes(can_frame_t *frame, const uint8_t *src)
{
    bool remote = (src[0] & 0x01U) != 0;
    uint8_t dlc = src[1] & 0x0fU;
    uint32_t tag = CAN_READ_BIG_ENDIAN_WORD(src + 3U);
    uint32_t can_id_word = CAN_READ_BIG_ENDIAN_WORD(src + 7U);
    bool ide = can_id_word & (1U << CAN_ID_EXT_BIT); // This is the bit in the binary format
    uint32_t arbitration_id = ide ? (can_id_word & 0x1fffffffU) : (can_id_word & 0x7ffU);
    const uint8_t *data = &src[11];

    can_make_frame(frame, ide, arbitration_id, dlc, data, remote);
    // The reference in the frame is the 32-bit tag stored in the frame
    can_frame_set_uref(frame, (void *)tag); 
}

/////////////////////////////////////// CAN receive overflow ///////////////////////////////////////

/// @brief Get the frame drop count for a receive overflow
/// @param overflow the overflow details returned by can_event_get_overflow()
INLINE uint32_t can_rx_overflow_get_frame_cnt(can_rx_overflow_event_t *overflow)
{
    return overflow->frame_cnt;
}

/// @brief Get the error frame drop count for a receive overflow
/// @param overflow the overflow details returned by can_event_get_overflow()
INLINE uint32_t can_rx_overflow_get_error_cnt(can_rx_overflow_event_t *overflow)
{
    return overflow->error_cnt;
}

//////////////////////////////////////// CAN receive event /////////////////////////////////////////

/// @brief Return the number of events in the receive FIFO.
/// @exception Is a minimum figure because more may have been added since the call returned.
/// @exception This call is non-reentrant
uint32_t can_recv_pending(void);

/// @brief Receive an event as a block of bytes
/// @param dest Pointer to where the bytes should be written
/// @param n_bytes Size of area to write the bytes
/// @returns number of bytes in the block
/// @exception This call is non-reentrant
/// @exception If the block isn't big enough then will return 0 even if there are pending events
uint32_t can_recv_as_bytes(uint8_t *dest, size_t n_bytes);

/// @brief Receive an event
/// @param event The application-allocated place to write an event to
/// @returns true if an event was received
/// @exception This call is non-reentrant
bool can_recv(can_rx_event_t *event);

/// @brief Get the event timestamp
/// @param event The event returned by can_recv()
/// @returns 32-bit timestamp (in microseconds) of the event
INLINE uint32_t can_event_get_timestamp(can_rx_event_t *event)
{
    return event->timestamp;
}

/// @brief Returns true if the event is an overflow event
/// @param event The event returned by can_recv()
INLINE bool can_event_is_overflow(can_rx_event_t *event)
{
    return event->event_type == CAN_EVENT_TYPE_OVERFLOW;
}

/// @brief Returns true if the event is a CAN error frame event
/// @param event The event returned by can_recv()
INLINE bool can_event_is_error(can_rx_event_t *event)
{
    return event->event_type == CAN_EVENT_TYPE_CAN_ERROR;
}

/// @brief Returns true if the event is a CAN frame received event
/// @param event The event returned by can_recv()
INLINE bool can_event_is_frame(can_rx_event_t *event)
{
    return event->event_type == CAN_EVENT_TYPE_RECEIVED_FRAME;
}

/// @brief Returns a pointer to the CAN frame of the event
/// @param event The event returned by can_recv()
/// @exception The event must be a frame received event
/// @exception The frame is contained within the storage of the event
INLINE can_frame_t *can_event_get_frame(can_rx_event_t *event)
{
    if (can_event_is_frame(event)) {
        return &event->event.frame;
    }
    else {
        return NULL;
    }
}

/// @brief Returns a pointer to the CAN error event
/// @param event The event returned by can_recv()
/// @exception The event must be a CAN error received event
INLINE can_error_t *can_event_get_error(can_rx_event_t *event)
{
    return &event->event.error;
}

/// @brief Returns a pointer to the overflow event
/// @param event The event returned by can_recv()
/// @exception The event must be an overflow event
INLINE can_rx_overflow_event_t *can_event_get_overflow(can_rx_event_t *event)
{
    return &event->event.overflow;
}

//////////////////////////////////////// CAN transmit event /////////////////////////////////////////

/// @brief Receive a frame transmission FIFO event
/// @param event The application-allocated place to write the event to
/// @returns true if an event was received
/// @exception This call is non-reentrant
bool can_recv_tx_event(can_tx_event_t *event);

/// @brief Receive an event as a block of bytes
/// @param dest Pointer to where the bytes should be written
/// @param n_bytes Size of area to write the bytes
/// @returns number of bytes in the block
/// @exception This call is non-reentrant
/// @exception If the block isn't big enough then will return 0 even if there are pending events
uint32_t can_recv_tx_event_as_bytes(uint8_t *dest, size_t n_bytes);

/// @brief Return the number of events in the transmit event FIFO.
/// @exception This call is non-reentrant
/// @exception Is a minimum figure because more may have been added since the call returned.
uint32_t can_recv_tx_events_pending(void);

/// @brief Returns true if the transmit event is a frame transmission
/// @param event The event returned by can_recv_tx_event()
INLINE bool can_tx_event_is_frame(can_tx_event_t *event)
{
    return event->event_type == CAN_EVENT_TYPE_TRANSMITTED_FRAME;
}

/// @brief Returns true if the transmit event is an overflow event
/// @param event The event returned by can_recv_tx_event()
INLINE bool can_tx_event_is_overflow(can_tx_event_t *event)
{
    return event->event_type == CAN_EVENT_TYPE_OVERFLOW;
}

/// @brief Returns the user reference associated with the transmitted frame
/// @param event The event returned by can_recv_tx_event()
INLINE can_uref_t can_tx_event_get_uref(can_tx_event_t *event)
{
    return event->info.uref;
}

/// @brief Returns the overflow count
/// @param event The event returned by can_recv_tx_event()
/// @exception The event must be an overflow event
INLINE uint32_t can_tx_event_get_overflow_cnt(can_tx_event_t *event)
{
    return event->info.overflow_cnt;
}

/// @brief Returns the 32-bit microsecond timestamp
/// @param event The event returned by can_recv_tx_event()
INLINE uint32_t can_tx_event_get_timestamp(can_tx_event_t *event)
{
    return event->timestamp;
}

///////////////////////////////////////// CAN ID filter ////////////////////////////////////////////

/// @brief Create an ID acceptance filter that exactly matches a CAN ID
/// @param filter A pointer to an application-allocated filter structure
/// @param canid The CAN ID to match exactly
void can_make_id_filter(can_id_filter_t *filter, can_id_t canid);

/// @brief Create an ID acceptance filter that matches all frames
/// @param filter A pointer to an application-allocated filter structure
void can_make_id_filter_all(can_id_filter_t *filter);

/// @brief Create an ID acceptance filter that matches no frames
/// @param filter A pointer to an application-allocated filter structure
void can_make_id_filter_disabled(can_id_filter_t *filter);

/// @brief Returns true if the ID acceptance filter will accept all frames
/// @param filter A pointer to an application-allocated filter structure
bool can_id_filter_is_all(can_id_filter_t *filter);

/// @brief Returns true if the ID acceptance filter will accept extended ID frames
/// @param filter A pointer to an application-allocated filter structure
bool can_id_filter_is_extended(can_id_filter_t *filter);

/// @brief Returns the arbitration ID mask of the ID acceptance filter
/// @param filter A pointer to an application-allocated filter structure
uint32_t can_id_filter_get_mask(can_id_filter_t *filter);

/// @brief Returns the arbitration ID match value of the ID acceptance filter
/// @param filter A pointer to an application-allocated filter structure
uint32_t can_id_filter_get_match(can_id_filter_t *filter);

/// @brief Create an ID acceptance filter that matches frames with a mask/match pair
/// @param filter A pointer to an application-allocated filter structure
/// @param ide If true, match frames with IDE=1 (i.e. extended ID frames) else match standard ID frames
/// @param arbitration_id_match The arbitration ID match value
/// @param arbitration_id_mask The arbitration ID mask value
void can_make_id_filter_masked(can_id_filter_t *filter,
                               bool ide,
                               uint32_t arbitration_id_match,
                               uint32_t arbitration_id_mask);

/// @brief A pre-defined empty list of ID acceptance filters
#define CAN_NO_FILTERS          ((can_id_filters_t *)0)

/////////////////////////////////////////// CAN error //////////////////////////////////////////////

/// @brief Return true if the CAN error frame is a CAN CRC error
/// @param error Error returned by a call to can_event_get_error()
bool can_error_is_crc(can_error_t *error);

/// @brief Return true if the CAN error frame is a CAN Stuff error
/// @param error Error returned by a call to can_event_get_error()
bool can_error_is_stuff(can_error_t *error);

/// @brief Return true if the CAN error frame is a CAN Form error
/// @param error Error returned by a call to can_event_get_error()
bool can_error_is_form(can_error_t *error);

/// @brief Return true if the CAN error frame is a CAN ACK error
/// @param error Error returned by a call to can_event_get_error()
bool can_error_is_ack(can_error_t *error);

/// @brief Return true if the CAN error frame is a CAN bit error (transmited 1, received 0)
/// @param error Error returned by a call to can_event_get_error()
bool can_error_is_bit1(can_error_t *error);

/// @brief Return true if the CAN error frame is a CRC error (transmited 0, received 1)
/// @param error Error returned by a call to can_event_get_error()
/// @exception This is hardware-specific: some controllers may report this as can_error_is_bit1()
bool can_error_is_bit0(can_error_t *error);

/// @brief Return true if this error caused a Bus Off
/// @param error Error returned by a call to can_event_get_error()
/// @exception This is target-specific and not part of the standard API
bool can_error_is_bus_off(can_error_t *error);

/// @brief Return a count of the number of frames received without error
/// @param error Error returned by a call to can_event_get_error()
/// @exception This is target-specific and not part of the standard API
uint32_t can_error_get_frame_cnt(can_error_t *error);

/// @brief Return the current timestamp timer.
/// @exception This call is non-reentrant
/// @exception The returned value will always lag the actual value due to accessing delays
/// @exception An external CAN controller connected by SPI will have a significant lag
uint32_t can_get_time(void);

/// @brief Return the space in the priority or FIFO transmit queues
/// @exception This call is non-reentrant
/// @exception Use can_is_space() to determine if there is space to queue the frame
uint32_t can_get_send_space(bool fifo);

/// @brief Handler for ISR; application is responsible for dismissing the GPIO interrupt
/// @exception This call is non-reentrant
/// @exception Target-specific to the MCP2517FD and MCP2518FD CAN controllers
void mcp2517fd_irq_handler(void);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// SPI BINDING ////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// These functions must be provided by the environment to bind the drivers to specific hardware
// and is an API for the driver binding code only.

// Initialize the SPI pins (set up the on-chip SPI controller and set its GPIO pins for SPI functions)
inline void mcp2517fd_spi_pins_init(void);

// Enable interrupts on the GPIO pin connected to the controller's IRQ line
inline void mcp2517fd_spi_gpio_enable_irq(void);

// Disable interrupts on the GPIO pin connected to the controller's IRQ line
inline void mcp2517fd_spi_gpio_disable_irq(void);

// Select the controller via its SPI chip select I/O pin (active low) 
inline void mcp2517fd_spi_select(void);

// Deselect the controller via its SPI chip select I/O pin
inline void mcp2517fd_spi_deselect(void);

// Write len bytes of data from src to the SPI
inline void mcp2517fd_spi_write(const uint8_t *src, size_t len);

// Write len bytes of data from cmd to the SPI port, storing the response in resp
inline void mcp2517fd_spi_read_write(const uint8_t *cmd, uint8_t *resp, size_t len);

// Read len bytes of data to dst from the SPI port
inline void mcp2517fd_spi_read(uint8_t *dst, size_t len);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// CALLBACKS /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Callback from ISR to indicat eframe has transmitted
/// @param uref The application provided user reference associated with the frame
/// @param timestamp The time the frame was transmitted (taken at frame SOF)
/// @exception Called at ISR level so interrupts are locked
/// @exception Code must be execute quickly (so should run from RAM in the RP2040)
void TIME_CRITICAL can_isr_callback_frame_tx(can_uref_t uref, uint32_t timestamp);

/// @brief Callback to convert a user reference into a 32-bit tag
/// @param uref The user reference associated with a frame
/// @exception Called at ISR level so interrupts are locked
/// @exception Code must be execute quickly (so should run from RAM in the RP2040)
uint32_t TIME_CRITICAL can_isr_callback_uref(can_uref_t uref);

/// @brief Callback from ISR to indicate frame has been received
/// @param frame A pointer to the frame
/// @param timestamp The time the frame was received (taken at frame SOF)
/// @exception Frame scope is for the call only: do not store reference to any part of the frame
/// @exception Called at ISR level so interrupts are locked
/// @exception Code must be execute quickly (so should run from RAM in the RP2040)
void TIME_CRITICAL can_isr_callback_frame_rx(can_frame_t *frame, uint32_t timestamp);

/// @brief Callback from ISR to indicate frame has been received
/// @param error A pointer to the error
/// @param timestamp The time the error was received
/// @exception Error scope is for the call only: do not store reference to any part of the error
/// @exception Called at ISR level so interrupts are locked
/// @exception Code must be execute quickly (so should run from RAM in the RP2040)
/// @exception The accuracy of the timestamp is target-specific
void TIME_CRITICAL can_isr_callback_error(can_error_t error, uint32_t timestamp);

#endif // CANAPI_H
