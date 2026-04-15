/*
 * lon_usb_single_link.c
 *
 * Copyright (c) 2017-2026 EnOcean
 * SPDX-License-Identifier: MIT
 * See LICENSE file for details.
 * 
 * Title:   LON USB Network Driver Link
 * Purpose: Provides a LON link-layer interface with a U10, U20,
 * 			U60, or U70 LON USB network interface device.
 * Notes:   Concurrency model (per-interface):
 * 			- Each LonUsbLinkState has a mutex lock guarding two shared structures:
 * 			  (1) lon_usb_uplink_ring_buffer (staging ring for raw bytes)
 * 			  (2) uplink_queue (singly-linked list of parsed messages)
 * 			- Producers:
 * 			  • LonUsbFeedRx() may run in an ISR/deferred thread to push bytes into
 * 			    lon_usb_uplink_ring_buffer
 * 			  • ProcessUplinkBytes() (invoked by ReadLonUsbMsg) produces messages
 * 			    into uplink_queue
 * 			- Consumers:
 * 			  • ReadLonUsbMsg() drains lon_usb_uplink_ring_buffer and pops from uplink_queue
 * 			    to return a message to the caller
 * 			- Locking rules:
 * 			  • Always OsalLockMutex(&state->queue_lock) before reading/writing
 * 			    lon_usb_uplink_ring_buffer or uplink_queue, including related stats updates
 * 			  • Keep the critical section minimal; perform parsing and other CPU
 * 			    work outside the lock
 * 			  • For uplink_queue pop, copy the node’s payload inside the lock,
 * 			    then free the node after unlocking to shorten the critical section
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "izot/IzotApi.h"
#include "lcs/lcs_platform.h"
#include "lon_usb/lon_usb_single_link.h"

#if LINK_IS(SINGLE_USB)

// LON frame sync byte value
#define FRAME_SYNC					0x7E	// Start of frame indicator; escaped when in data

// LON frame code masks
#define FRAME_CODE_SEQ_NUM_MASK		0xE0	// Sequence number mask (bits 5-7)
#define FRAME_CODE_SEQ_NUM_INC		0x20	// Sequence number increment (add to current value; wrap at 7)
#define FRAME_CODE_ACK_MASK			0x10	// Acknowledgment flag mask (bit 4)
#define FRAME_CODE_CMD_MASK			0x0F	// Frame command mask (bits 0-3)

// Sequence number mask
#define SEQ_NUM_MASK				0x07	// Sequence number mask (bits 0-2)

// USB driver parameters and defaults
#define DEFAULT_IN_TRANSFER_SIZE	4096
#define DEFAULT_READ_TIMEOUT		500
#define DEFAULT_WRITE_TIMEOUT		500
#define DEFAULT_UPLINK_LIMIT		200
#define DEFAULT_LLP_TIMEOUT			1000

// LON network diagnostic message used by the driver
const LonDataFrame MsgReportStatus = {LonNiLocalNetMgmtCmd,	// Network interface command
		UNICAST_MSG_CODE_OFFSET+(1),	// 15 PDU length
	  { 0x6F,					// 0 msg_type, 3 st (request), 0 auth, 15 tag (private)
		0x08,					// 0 priority, 0 path, 0 2-bit cmpl_code, 1 addr_mode, 0 alt_path, 0 pool, 0 response
		(1),					// 1 message length
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Network address
		IzotNdQueryStatus
	  }
};

// LON network management messages used by the driver
//static const LonDataFrame MsgClear 	  = {LonNiClearCmd, 0};
//static const LonDataFrame MsgIdentify = {LonNiIdentifyCmd, 0};
const LonDataFrame MsgModeL2	= {LonNiLayerModeCmd, 1, {1}};
const LonDataFrame MsgModeL5	  = {LonNiLayerModeCmd, 1, {0}};
const LonDataFrame MsgStatus	  = {LonNiStatusCmd, 0, {0}};
const LonDataFrame MsgUniqueId	= {LonNiLocalNetMgmtCmd,	// Network interface command (0x22)
		UNICAST_MSG_CODE_OFFSET+(5),	// 19 PDU length
	  { 0x70,					// 0 msg_type, 3 2-bit st (request), 1 auth, 0 4-bit tag
		0x00,					// 0 priority, 0 path, 0 2-bit cmpl_code, 0 addr_mode, 0 alt_path, 0 pool, 0 response
		(5),					// 5 message length
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Network address
		0x6d,					// Message code: read memory network management command
		0x01,					// Read-only memory relative address space
		0x00, 0x00,				// Starting address offset: 0
		0x06 					// Number of bytes to read: 6 (48-bits)
	  }
};

// Downlink state machine state names
const char* downlink_state_names[] = {
	"DOWNLINK_START",
	"DOWNLINK_WAIT_STARTUP_RESET",
	"DOWNLINK_WAIT_STARTUP_RESYNC_ACK",	"DOWNLINK_WAIT_RESYNC_RESET",
	"DOWNLINK_LAYER_5_REQUEST",
	"DOWNLINK_WAIT_LAYER_5_ACK",
	"DOWNLINK_WAIT_L5_RESYNC_ACK",
	"DOWNLINK_WAIT_L5_RESET",
	"DOWNLINK_NI_UNIQUE_ID_REQUEST",
	"DOWNLINK_WAIT_NI_UNIQUE_ID",
	"DOWNLINK_LAYER_2_REQUEST",
	"DOWNLINK_WAIT_LAYER_2_ACK",
	"DOWNLINK_WAIT_L2_RESYNC_ACK",
	"DOWNLINK_WAIT_L2_RESET",
	"DOWNLINK_MIP_STATUS_REQUEST",
	"DOWNLINK_WAIT_MIP_STATUS",
	"DOWNLINK_IDLE_START",
	"DOWNLINK_IDLE",
	"DOWNLINK_WAIT_CP_ACK",
	"DOWNLINK_WAIT_MSG_ACK",
	"DOWNLINK_WAIT_CP_MSG_REQ_ACK",
	"DOWNLINK_WAIT_CP_RESPONSE",
	"DOWNLINK_INVALID"
};

// Uplink state machine state names
const char* uplink_state_names[] = {
	"UPLINK_IDLE",
	"UPLINK_FRAME_CODE",
	"UPLINK_FRAME_PARAMETER",
	"UPLINK_CODE_PACKET_CHECKSUM",
	"UPLINK_MESSAGE",
	"UPLINK_ESCAPED_DATA",
	"UPLINK_INVALID"
};

/*****************************************************************
 * Section: Globals
 *****************************************************************/
bool linkInitialized = false; 	// Flag to indicate if LON USB
								// link is initialized
LonUsbLinkState iface_state;
								// Interface state array

/*****************************************************************
 * Section: Function Declarations
 *****************************************************************/
// LON USB link management
LonStatusCode StartLonUsbLink(LonUsbInterfaceMode configured_iface_mode);
void ResetUplinkState();
LonStatusCode RequestUsbNiUid();
LonStatusCode SetNiLayerMode(LonUsbInterfaceMode iface_mode);

// LON downlink processing
LonStatusCode ProcessNextDownlinkMessage();
LonStatusCode WriteDownlinkMessage();
//LonStatusCode WriteByteDownlink(uint8_t msg_byte);
LonStatusCode WriteDownlinkLocalNiCmd();
LonNiCommand GetExpectedResponse(uint8_t cmd);
LonStatusCode WriteDownlinkReportStatusCmd();
LonStatusCode WriteDownlinkCodePacket(LonUsbFrameCommand frame_cmd, uint8_t parameter);
LonStatusCode CreateFrameHeader(LonFrameHeader* frame_header,
				uint8_t frame_command, uint8_t parameter,
				uint8_t sequence_num, bool ack);
uint8_t ComputeChecksum(uint8_t *pkt, size_t len);
LonStatusCode IncrementDownlinkSequence();
LonStatusCode StartAckTimer(DownlinkState next_state);

LonStatusCode ResetDownlinkState();
LonStatusCode StopAckTimer();


// LON uplink processing
LonStatusCode ProcessUplinkBytes(uint8_t *chunk, size_t chunk_size);
LonStatusCode ProcessUplinkCodePacket();
LonStatusCode ClearUplinkTransaction();
LonStatusCode CheckUplinkCompleted(bool *completed);
LonStatusCode ProcessUplinkMessage();
size_t TotalMessageLength(const LonNiFrame *msg);

// Interface state array management
LonStatusCode InitIfaceStates(void);

// Downlink queue management
//static LonStatusCode PeekDownlinkBuffer(LonUsbQueueBuffer *dst);
LonStatusCode ReadDownlinkBuffer(LonUsbQueueBuffer *dst);
LonStatusCode FlushDownlinkQueue(MessagePriorityLevel priority);
LonStatusCode QueueDownlinkBuffer(LonUsbQueueBuffer *src);
size_t GetDownlinkBufferCount();
bool IsDownlinkQueueEmpty();

// Uplink queue management
//static LonStatusCode PeekUplinkBuffer(, LonUsbQueueBuffer *dst);
LonStatusCode ReadUplinkBuffer(LonUsbQueueBuffer *dst);
LonStatusCode QueueUplinkBuffer();
size_t GetUplinkBufferCount();
bool IsUplinkQueueEmpty();

// Debugging support functions
void PrintUplinkCodePacket(char *prefix);

//static void PrintMessage(char *prefix);

// Implementation-specific LON NI functions
void IdentifyLonNi();

/*****************************************************************
 * Section: Initialization Function Definitions
 *****************************************************************/
/*
 * Opens a LON USB network interface.
 * Parameters:
 *	 configured_iface_mode: LON interface configured_iface_mode, set to LON_IFACE_MODE_LAYER5
 *			   for layer 5 (host apps) or LON_IFACE_MODE_LAYER2 for
 *			   layer 2 (LON stacks)
 *   line_discipline: USB line discipline number, set to LDISCS_50 for
 *			   the U10 FT Rev C and U60 FT, and LDISCS_61 for the U10 FT
 *			   Rev A and B, U20 PL, U60 TP-1250, and U70 PL
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode OpenLonUsbLink(LonUsbInterfaceMode configured_iface_mode)
{
	LonStatusCode status = LonStatusNoError;
	if (configured_iface_mode != LON_IFACE_MODE_LAYER2 && configured_iface_mode != LON_IFACE_MODE_LAYER5) {
		status = LonStatusInvalidParameter;
		OsalPrintLog(ERROR_LOG, status, "OpenLonUsbLink: Invalid parameter(s)");
		return status;
	}
	
	// Initialize link interface states if this is the first LON USB open
	if (!linkInitialized) {
		if (!LON_SUCCESS(status = InitIfaceStates())) {
			OsalPrintLog(ERROR_LOG, status, "OpenLonUsbLink: Failed to initialize LON USB link interface states");
			return status;
		}
		linkInitialized = TRUE;
	}

	// Start LON USB link
	if (!LON_SUCCESS(status = StartLonUsbLink(configured_iface_mode))) {
		ResetUplinkState();
		OsalPrintLog(ERROR_LOG, status, "OpenLonUsbLink: Failed to start");
		return status;
	}
	iface_state.start_time = iface_state.last_timeout = OsalGetTickCount();
	OsalPrintLog(INFO_LOG, status, "OpenLonUsbLink: Opened LON USB interface ");
	return status;
}

/*
 * Starts the link for a specified LON USB network interface.
 * Parameters:
 *   configured_iface_mode: LON interface configured_iface_mode, set to
 *             LON_IFACE_MODE_LAYER5 for layer 5 (host apps) or
 *             LON_IFACE_MODE_LAYER2 for layer 2 (LON stacks)
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called at end of OpenLonUsbLink() and ProcessDownlinkRequests().
 */
LonStatusCode StartLonUsbLink(LonUsbInterfaceMode configured_iface_mode)
{
	LonStatusCode status = LonStatusNoError;
    LonUsbLinkState *state = &iface_state;
	// LonDataFrame *mode_msg; N/U also not used in lon_usb_link.c
	OsalPrintLog(INFO_LOG, status, "StartLonUsbLink: layer %d mode requested", configured_iface_mode == LON_IFACE_MODE_LAYER5 ? 5 : 2);
	// Reset LON USB link state
	if (!LON_SUCCESS(status = FlushDownlinkQueue(NORMAL_PRIORITY))) {
		OsalPrintLog(ERROR_LOG, status, "StartLonUsbLink: Failed to flush downlink queue");
		return status;
	}
	if (!LON_SUCCESS(status = ResetDownlinkState())) {
		OsalPrintLog(ERROR_LOG, status, "StartLonUsbLink: Failed to reset downlink state");
		return status;
	}
	ResetUplinkState();
    state->configured_iface_mode = configured_iface_mode;
	state->shutdown = false; // State initialization complete, cancel shutdown
	// Set start time and next downlink state
	state->start_time = state->last_timeout = OsalGetTickCount();
	state->downlink_state = DOWNLINK_START;
	return status;
}

/*
 * Resets uplink state for the start of a new packet.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called by:
 *	   - InitIfaceStates() which is called by OpenLonUsbLink()
 *	   - StartLonUsbLink() which is called by OpenLonUsbLink()
 *	     called by OpenLonUsbLink()
 *	   - ProcessUplinkBytes() after a framing error which is called
 *	     by ReadLonUsbMsg()
 *	   - CheckUplinkCompleted() on success which is called by 
 *		 ProcessUplinkBytes()
 */
void ResetUplinkState()
{
    LonUsbLinkState *state = &iface_state;
	state->uplink_ack_timer = 0;
	state->uplink_ack_timeout = ACK_WAIT_TIME;
	state->uplink_ack_timeouts = 0;
	state->uplink_ack_timeout_phase = 0;
	state->uplink_duplicate = false;
	state->uplink_frame_error = false;
	state->uplink_msg_index = 0;
	state->uplink_msg_length = 0;
	state->uplink_msg_timer = 0;
	state->uplink_state = UPLINK_IDLE;
	state->uplink_tail_escaped = false;
}

/*
 * Sends a resynchronization request to the network interface.
 * Parameters: None
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
static LonStatusCode SendResync()
{
	LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	if (state->lon_usb_iface_type == LON_USB_INTERFACE_U50) {
		// For MIP/U50, request the network interface to resynchronize
		OsalPrintLog(INFO_LOG, status, "SendResync: Send resync command");
		if (!LON_SUCCESS(status = WriteDownlinkCodePacket(NI_RESYNC_FRAME_CMD, 0))) {
		    OsalPrintLog(ERROR_LOG, status, "SendResync: Failed to send resync command");
			return status;
		}
		// Wait for null response from the interface to confirm synchronization
		state->null_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_STARTUP_RESYNC_ACK;
	}
	return status;
}

/*
 * Sends a read memory request for the LON USB interface unique ID.
 * Parameters:
 *   iface_index: LON interface index returned by OpenLonUsbLink()
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode RequestUsbNiUid()
{
    LonUsbLinkState *state = &iface_state;
	LonStatusCode status = LonStatusNoError;
	if (state->have_uid) {
		// Already have the network interface unique ID
		OsalPrintLog(INFO_LOG, status, 
				"RequestUsbNiUid: Already have unique ID, no need to request");
		return status;
	} else if (state->uid_retries <= 0) {
		// NI unique ID read failed after max retries; stop waiting for UID
		state->uid_retries = 0;
		state->wait_for_uid = false;
		OsalPrintLog(ERROR_LOG, LonStatusTimeout, 
				"RequestUsbNiUid: Failed to read unique ID after %d attempts",
				MAX_UID_RETRIES);
		return status;
	} else {
		// Request the network interface unique ID with a read memory command
		OsalPrintLog(INFO_LOG, status,
				"RequestUsbNiUid: Request LON unique ID (%d retries left), %s state, %s state", 
				state->uid_retries, downlink_state_names[state->downlink_state], uplink_state_names[state->uplink_state]);
		status = WriteLonUsbMsg(&MsgUniqueId);
	}
	if (!LON_SUCCESS(status)) {
		return status;
	}
	state->uid_retries--;
	state->wait_for_uid = true;
	// Start a timer to wait for the response to the unique ID request
	state->uid_wait_timer = OsalGetTickCount();
	state->downlink_state = DOWNLINK_WAIT_NI_UNIQUE_ID;
	return status;
}

/*
 * Reads the LON USB interface unique ID from the state.
 * Parameters:
 *   uid_buffer: Buffer to receive the unique ID (must be at least 6 bytes)
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode ReadUsbNiUid(IzotUniqueId *uid_buffer)
{
    LonUsbLinkState *state = &iface_state;
	if (!uid_buffer) {
		return LonStatusInvalidParameter;
	}
	if (!state->have_uid) {
		return LonStatusLniUniqueIdNotAvailable;
	}
	memcpy(uid_buffer, state->uid, IZOT_UNIQUE_ID_LENGTH);
	return LonStatusNoError;
}

/*
 * Sends a status request for the LON USB interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
static LonStatusCode RequestUsbNiStatus()
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	if (state->have_status) {
		// Already have the LON interface status
		OsalPrintLog(INFO_LOG, status, 
				"RequestUsbNiStatus: Already have LON status, no need to request");
		return status;
	} else if (state->status_retries <= 0) {
		// NI status read failed after max retries; stop waiting for status
		state->status_retries = 0;
		state->wait_for_status = false;
		OsalPrintLog(ERROR_LOG, LonStatusTimeout, 
				"RequestUsbNiStatus: Failed to read LON interface unique ID after %d attempts",
				MAX_STATUS_RETRIES);
		return status;
	} else {
		// Request the network interface status with a read memory command
		OsalPrintLog(INFO_LOG, status,
		        "RequestUsbNiStatus: Request LON status (%d retries left), %s state, %s state", 
				state->status_retries, downlink_state_names[state->downlink_state], uplink_state_names[state->uplink_state]);
		if (!LON_SUCCESS(status = WriteLonUsbMsg(&MsgStatus))) {
			return status;
		}
	}
	state->status_retries--;
	state->wait_for_status = true;
	// Start a timer to wait for the response to the status request
	state->status_wait_timer = OsalGetTickCount();
	state->downlink_state = DOWNLINK_WAIT_MIP_STATUS;
	return status;
}

/*
 * Sets the LON interface layer mode to layer 2 or layer 5.
 * Parameters:
 *   iface_mode: LON interface mode to set, either LON_IFACE_MODE_LAYER2 or
 *               LON_IFACE_MODE_LAYER5
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode SetNiLayerMode(LonUsbInterfaceMode iface_mode)
{
    LonUsbLinkState *state = &iface_state;
    
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	OsalPrintLog(INFO_LOG, LonStatusNoError, 
			"SetNiLayerMode: Set LON interface layer %d mode request",
			iface_mode == LON_IFACE_MODE_LAYER5 ? 5 : 2);
	// Update the current desired interface mode in the state
	state->current_iface_mode = iface_mode;
	// Mark the operating layer mode as unknown until confirmation is received from the interface
	state->lon_stats.l2_l5_mode = LON_IFACE_MODE_UNKNOWN;
	// Send the requested layer mode command to the interface
	if (iface_mode == LON_IFACE_MODE_LAYER5) {
		status = WriteLonUsbMsg(&MsgModeL5);
	} else {
		status = WriteLonUsbMsg(&MsgModeL2);
	}
	return status;
}

/*
 * Checks if the LON USB link is ready.
 * Parameters:
 * Returns:
 *   true if the interface is ready; false otherwise
 */
bool LonUsbLinkReady()
{
	LonUsbLinkState *state = &iface_state;;
	if (state->shutdown) {
		return false;
	}
	return state->ready;
}

/*****************************************************************
 * Section: Downlink Function Definitions
 *****************************************************************/
LonStatusCode NetworkInterfaceSend(void)
{
	LonStatusCode status = LonStatusNoError;
	status = ProcessDownlinkRequests();
	return status;
}

/*
 * Processes retry and downlink requests for a specified LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   This function processes downlink messages and retry attempts for failed
 *   messages from the downlink queue and writes them to the USB interface.
 *   It is called periodically to handle downlink traffic.
 */
LonStatusCode ProcessDownlinkRequests()
{
    LonUsbLinkState *state = &iface_state;
	static OsalTickCount last_report_time = 0;
	if (OsalGetTickCount() - last_report_time > 10000) {
		last_report_time = OsalGetTickCount();
		if (!IsDownlinkQueueEmpty() || !IsUplinkQueueEmpty()) {
			OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError,
					"ProcessDownlinkRequests: uptime %lu sec, %s state, %s state, downlink queue count %zu, uplink queue count %zu",
					(last_report_time - state->start_time) / 1000,
					downlink_state_names[state->downlink_state],
					uplink_state_names[state->uplink_state],
					GetDownlinkBufferCount(),
					GetUplinkBufferCount());
		}
	}
	LonStatusCode status = LonStatusNoError;
	DownlinkState starting_downlink_state = state->downlink_state;
	static DownlinkState last_downlink_state = DOWNLINK_INVALID;
	if (state->downlink_state != last_downlink_state) {
		OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessDownlinkRequests: downlink state changed from %s to %s, uplink state %s",
				downlink_state_names[last_downlink_state],
				downlink_state_names[state->downlink_state], uplink_state_names[state->uplink_state]);
		last_downlink_state = state->downlink_state;
	}
	switch(state->downlink_state) {

	/* --- DOWNLINK START --- */
	case DOWNLINK_START:
		// Flush any pending non-priority downlink messages and restart the downlink
		OsalPrintLog(INFO_LOG, status, "ProcessDownlinkRequests: Processing downlink restart");
		StartLonUsbLink(state->configured_iface_mode);
		state->wait_for_reset = true;	// Start by waiting for a reset to ensure the LON interface is in a known state before sending commands
		state->reset_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_STARTUP_RESET;
		break;

	/* --- INITIAL BOOT & RESYNC --- */
	case DOWNLINK_WAIT_STARTUP_RESET:
		// Waiting for the network interface to reset on startup and after a downlink restart
		if (state->have_reset || (OsalGetTickCount() - state->reset_wait_timer > NI_RESET_WAIT_TIME)) {
			state->have_reset = false;
			// Resynchronize the LON driver and network interface
			if (!LON_SUCCESS(status = SendResync())) {
				return status;
			}
			state->wait_for_null = true; 	// Wait for null code packet to confirm the interface is ready after reset
			state->null_wait_timer = OsalGetTickCount();
			state->downlink_state = DOWNLINK_WAIT_STARTUP_RESYNC_ACK;
		}
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_STARTUP_RESYNC_ACK:
		// Waiting for the network interface null code packet response that follows a resynchronization request on startup
		if (state->have_null) {
			state->have_null = false;		// Clear null flag after handling null response
			state->wait_for_reset = true;	// Wait for reset to ensure the interface is ready before sending commands
			state->reset_wait_timer = OsalGetTickCount();
			state->downlink_state = DOWNLINK_WAIT_RESYNC_RESET;
		} else if (OsalGetTickCount() - state->null_wait_timer > NI_NULL_WAIT_TIME) {
			// Resync ack timeout; try resynchronization again
			if (!LON_SUCCESS(status = SendResync())) {
				return status;
			}
			state->wait_for_null = true; 	// Wait for null code packet to confirm the interface is ready after reset
			state->null_wait_timer = OsalGetTickCount();
		}
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_RESYNC_RESET:
		// Waiting for the network interface to reset after a resynchronization request on startup
		if (state->have_reset || (OsalGetTickCount() - state->reset_wait_timer > NI_RESET_WAIT_TIME)) {
			state->have_reset = false;	// Clear reset flag after handling reset
			if (!state->have_uid) {
				state->uid_retries = MAX_UID_RETRIES;
				state->downlink_state = DOWNLINK_NI_UNIQUE_ID_REQUEST;
			} else if (!state->have_status) {
				state->downlink_state = DOWNLINK_MIP_STATUS_REQUEST;
			} else if (state->lon_stats.l2_l5_mode != LON_IFACE_MODE_LAYER2) {
				state->downlink_state = DOWNLINK_LAYER_2_REQUEST;
			} else {
				state->downlink_state = DOWNLINK_IDLE_START;
			}
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- LAYER 5 TRANSITION & VERIFICATION --- */
	case DOWNLINK_LAYER_5_REQUEST:
		// Set the LON interface layer mode to layer 5 and wait for null code packet
		if (!LON_SUCCESS(status = SetNiLayerMode(LON_IFACE_MODE_LAYER5))) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Failed to set LON interface mode to layer 5");
			return status;
		}
		state->wait_for_null = true; 	// Wait for null code packet to confirm the interface is ready after layer mode change
		state->null_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_LAYER_5_ACK;
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_LAYER_5_ACK:
		// Wait for uplink null frame acknowlowdging the layer 5 mode change
		if (state->have_null) {
			state->have_null = false;	// Clear null flag after handling null response
			state->downlink_state = DOWNLINK_NI_UNIQUE_ID_REQUEST;
		} else if (OsalGetTickCount() - state->null_wait_timer > NI_NULL_WAIT_TIME){
			// Null wait timeout occurred; try resynchronization again
			state->downlink_state = DOWNLINK_WAIT_STARTUP_RESYNC_ACK;
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- READ LON INTERFACE UNIQUE ID --- */
	case DOWNLINK_NI_UNIQUE_ID_REQUEST:
		// Verify the LON interface mode is layer 5
		if (state->current_iface_mode != LON_IFACE_MODE_LAYER5) {
			OsalPrintLog(INFO_LOG, status, "ProcessDownlinkRequests: LON request layer 5 mode for unique ID request, current mode %s",
					state->current_iface_mode == LON_IFACE_MODE_LAYER2 ? "layer 2" :
					(state->current_iface_mode == LON_IFACE_MODE_LAYER5 ? "layer 5" : "unknown"));
			// Try setting layer 5 mode again
			state->downlink_state = DOWNLINK_LAYER_5_REQUEST;
			status = ProcessNextDownlinkMessage();
			break;
		}
		// Send read memory request for the network interface unique ID
		if (!LON_SUCCESS(status = RequestUsbNiUid())) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Failed to request LON interface unique ID after resync");
			return status;
		}
		state->uid_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_NI_UNIQUE_ID;
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_NI_UNIQUE_ID:
		// Waiting for the LON interface unique ID response
		if (state->have_uid) {
			// UID received; change to LON interface layer 2 mode
			state->downlink_state = DOWNLINK_LAYER_2_REQUEST;
		} else if (OsalGetTickCount() - state->uid_wait_timer > NI_UID_WAIT_TIME) {
			// Wait timed out
			if (state->wait_for_uid) {
				// Retry LON interface unique ID request
				state->downlink_state = DOWNLINK_NI_UNIQUE_ID_REQUEST;
			} else {
				// Max retries reached; move to next step in initialization
				OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Cannot read LON unique ID");
				state->downlink_state = DOWNLINK_MIP_STATUS_REQUEST;
			}
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- LAYER 2 TRANSITION & VERIFICATION --- */
	case DOWNLINK_LAYER_2_REQUEST:
		// Set the LON interface mode to layer 2 and wait for null code packet
		if (!LON_SUCCESS(status = SetNiLayerMode(LON_IFACE_MODE_LAYER2))) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Failed to set LON interface mode to layer 2");
			return status;
		}
		state->wait_for_null = true; 	// Wait for null code packet to confirm the interface is ready after layer mode change
		state->null_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_LAYER_2_ACK;
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_LAYER_2_ACK:
		// Wait for uplink null frame acknowlowdging the layer 2 mode change
		if (state->have_null) {
			state->have_null = false;	// Clear null flag after handling null response
			state->status_retries = MAX_STATUS_RETRIES;
			state->downlink_state = DOWNLINK_MIP_STATUS_REQUEST;
		} else if (OsalGetTickCount() - state->null_wait_timer > NI_NULL_WAIT_TIME){
			// Null wait timeout occurred; try resynchronization again
			state->downlink_state = DOWNLINK_WAIT_STARTUP_RESYNC_ACK;
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- LON INTERFACE MIP STATUS READ --- */
	case DOWNLINK_MIP_STATUS_REQUEST:
		// Verify the LON interface mode is layer 2
		if (state->current_iface_mode != LON_IFACE_MODE_LAYER2) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: LON not in expected layer 2 mode after layer 2 mode set, current mode %s",
					state->current_iface_mode == LON_IFACE_MODE_LAYER2 ? "layer 2" :
					(state->current_iface_mode == LON_IFACE_MODE_LAYER5 ? "layer 5" : "unknown"));
			// Try setting layer 2 mode again
			state->downlink_state = DOWNLINK_LAYER_2_REQUEST;
			status = ProcessNextDownlinkMessage();
			break;
		}
		// Send MIP status request to the LON interface
		if (!LON_SUCCESS(status = RequestUsbNiStatus())) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Failed to request LON interface status after resync");
			return status;
		}
		state->status_wait_timer = OsalGetTickCount();
		state->downlink_state = DOWNLINK_WAIT_MIP_STATUS;
		status = ProcessNextDownlinkMessage();
		break;
	case DOWNLINK_WAIT_MIP_STATUS:
		// Waiting for the LON interface MIP status response
		if (state->have_status) {
			// MIP status received; start the downlink idle state
			state->downlink_state = DOWNLINK_IDLE_START;
		} else if (OsalGetTickCount() - state->status_wait_timer > NI_STATUS_WAIT_TIME) {
			// Wait timed out
			if (state->wait_for_status) {
				// Retry LON interface status request
				state->downlink_state = DOWNLINK_MIP_STATUS_REQUEST;
			} else {
				// Max retries reached; move to next step in initialization
				OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Cannot read LON status");
				state->downlink_state = DOWNLINK_IDLE_START;
			}
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- NORMAL OPERATIONS --- */
	case DOWNLINK_IDLE_START:
		// Verify the LON interface mode is layer 2
		if (state->current_iface_mode != LON_IFACE_MODE_LAYER2) {
				state->downlink_state = DOWNLINK_LAYER_2_REQUEST;
				status = ProcessNextDownlinkMessage();
				break;
		}
		// Send null frame to clear any message timers
		status = WriteDownlinkCodePacket(NULL_FRAME_CMD, 0);
		if (!LON_SUCCESS(status)) {
			OsalPrintLog(ERROR_LOG, status, "ProcessDownlinkRequests: Cannot write downlink null code packet");
			return status;
		}
		// Change state to ready
		state->ready = true;
		state->downlink_state = DOWNLINK_IDLE;
		status = ProcessNextDownlinkMessage();
		state->lon_stats.startup_time = LonWatchElapsed(&state->lon_stats.runtime);
		OsalPrintLog(INFO_LOG, status, "ProcessDownlinkRequests: Initialization complete, entering idle state");
		break;
	case DOWNLINK_IDLE:
		status = ProcessNextDownlinkMessage();
		break;

	/* --- MESSAGE ACKNOWLEDGMENT AND RESPONSE WAIT STATES --- */
	case DOWNLINK_WAIT_CP_RESPONSE:
	case DOWNLINK_WAIT_CP_ACK:
	case DOWNLINK_WAIT_MSG_ACK:
		// Waiting for an ACK or response
		if (OsalGetTickCount() - state->uplink_ack_timer < state->uplink_ack_timeout) {
			// Timer has not timed out
			break;
		}
		// Wait timed out; handle timeout
		state->downlink_state = DOWNLINK_IDLE;
		Increment32(state->lon_stats.uplink.rx_ack_timeout_errors);
		// Look for runt CPs in the RX path that could clog up things
		if (state->uplink_state == UPLINK_IDLE && state->uplink_msg_index) {
			ResetUplinkState();
		}
		if (++state->uplink_ack_timeouts >= 2) {
			if (state->uplink_ack_timeout_phase < 5) {
				// Many ack timeouts occurred; re-sync by requesting ND Status
				IncrementDownlinkSequence();
				// Overwrite the downlink message at state->downlink_buffer.usb_ni_message
				memcpy(&state->downlink_buffer.usb_ni_data_frame, &MsgReportStatus, sizeof(MsgReportStatus));
				WriteDownlinkMessage();
				if (++state->uplink_ack_timeout_phase == 0) {
					state->uplink_ack_timeout_phase--;
				}
				OsalPrintLog(PACKET_TRACE_LOG, LonStatusTimeout, "ACK timeout phase %d, sequence #%02X, requesting status",
					state->uplink_ack_timeout_phase, state->downlink_seq_number);
				OsalPrintLog(PACKET_TRACE_LOG, LonStatusTimeout, "checksum errors: %d  code packet failures: %d",
					state->lon_stats.uplink.rx_checksum_errors, state->lon_stats.uplink.rx_code_packet_failures);
				OsalPrintLog(PACKET_TRACE_LOG, LonStatusTimeout, "packets received: %d, bytes received: %d",
					state->lon_stats.uplink.packets_received, state->lon_stats.uplink.bytes_received);
				OsalPrintLog(PACKET_TRACE_LOG, LonStatusTimeout, "TX state %s, RX state %s",
					downlink_state_names[starting_downlink_state], uplink_state_names[state->uplink_state]);
			} else {		// Do this once
				OsalTickCount current_time = OsalGetTickCount();
				// If we have a MAC address and it's been more than 60 seconds since startup or the last timeout
				// then panic and reset the interface
				if (!state->wait_for_uid && (current_time - state->last_timeout) > STARTUP_DELAY) {
					OsalPrintLog(ERROR_LOG, LonStatusTimeout, "ACK timeout limit #%d, restart now", state->uplink_ack_timeout_phase);
					state->last_timeout = current_time;		// Don't keep doing this too rapidly
				} else {	// Try again
					state->uplink_ack_timeout_phase = 0;
				}
			}
			state->uplink_ack_timeouts = 0;
		} else {
			OsalPrintLog(PACKET_TRACE_LOG, LonStatusTimeout, "ACK timeout (first case)");
		}
		status = ProcessNextDownlinkMessage();
		break;

	/* --- INVALID STATE; RESET TO STARTUP --- */
	default:
		state->downlink_state = DOWNLINK_START;
		break;
	}
	return status;
}

/*
 * Processes the next available downlink message for a specified LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   This function writes the next available downlink message from the
 *   downlink queue and to the USB interface.  It is called from 
 *   multiple states in ProcessDownlinkRequests().
 */
LonStatusCode ProcessNextDownlinkMessage()
{
    LonUsbLinkState *state = &iface_state;
    LonStatusCode status = LonStatusNoError;
    //DownlinkState starting_downlink_state = state->downlink_state;   N/U
    if (state->downlink_ack_seq_number >= 0) {
        // Downlink ACK requested but not yet sent; send a null CP to send the ACK
        OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessNextDownlinkMessage: Sending null code packet to send ACK, ack sequence #%02X",
                state->downlink_ack_seq_number);
        status = WriteDownlinkCodePacket(NULL_FRAME_CMD, 0);
        if (!LON_SUCCESS(status)) {
            OsalPrintLog(ERROR_LOG, status, "ProcessNextDownlinkMessage: Cannot write downlink null code packet");
            return status;
        }
        status = IncrementDownlinkSequence();
        if (!LON_SUCCESS(status)) {
            OsalPrintLog(ERROR_LOG, status, "ProcessNextDownlinkMessage: Cannot increment sequence number");
            return status;
        }
    }
    // Look for a downlink buffer to process
    status = ReadDownlinkBuffer(&state->downlink_buffer);
    if ((status == LonStatusNoBufferAvailable) ) {//|| !&state->downlink_buffer) /* this condition is always true*/{
       // No downlink message available
        status = LonStatusNoError;
        return status;
    }
    if (!LON_SUCCESS(status)) {
        OsalPrintLog(ERROR_LOG, status, "ProcessNextDownlinkMessage: Cannot read downlink buffer");
        return status;
    }
    // Message is available; process it
    size_t short_pdu_length = state->downlink_buffer.usb_ni_data_frame.short_pdu_length;
    LonNiCommand cmd = state->downlink_buffer.usb_ni_data_frame.ni_command; // Extracted early
	OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessNextDownlinkMessage: Processing downlink message, command 0x%02X, short PDU length %d, current downlink state %s",
			cmd, short_pdu_length, downlink_state_names[state->downlink_state]);
    if ((state->lon_usb_iface_type == LON_USB_INTERFACE_U50) && 
	        (short_pdu_length == 1) && 
    	    (cmd != LonNiSetL5ModeCmd) && (cmd != LonNiSetL2ModeCmd)) {
        // Process local NI command that is not a layer mode change;
		// PDU size of 1 means there is no payload
        ResetDownlinkState();
        WriteDownlinkLocalNiCmd();
        return status;
    }
    // Handle a standard message or a layer mode change command
    //uint8_t *pdu = state->downlink_buffer.usb_ni_data_frame.pdu;			//N/U
    status = WriteDownlinkMessage();
    if (!LON_SUCCESS(status)) {
        OsalPrintLog(ERROR_LOG, status, "ProcessNextDownlinkMessage: Cannot write downlink message");
        return status;
    }
    return status;
}

/*
 * Writes a downlink message to the LON USB network interface.
 * Parameters:
 *   input_frame: pointer to LonDataFrame or LdvExtendedMessage structure
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Typically called by layer 2 of a LON stack with the network interface
 *   in layer 2 mode.  During LON driver initialization, called by the
 *   LON driver with the network interface in layer 5 or layer 2 mode.
 *   May also be called by a host application with the network interface
 *   in layer 5 mode.
 */
LonStatusCode WriteLonUsbMsg(const LonDataFrame* input_frame)
{
    LonUsbLinkState *state = &iface_state;
	LonStatusCode status = LonStatusNoBufferAvailable;
	OsalLockMutex(&state->state_lock);
	if (GetDownlinkBufferCount() < MAX_LON_DOWNLINK_BUFFERS) {
		// Build downlink buffer -- the sequence number and checksum
		// will be updated when the buffer is written to the interface
		status = CreateFrameHeader(&state->downlink_buffer_staging.frame_header,
				MSG_FRAME_CMD, 0, 0, true); // TODO: Determine if parameter should be 1 for 1 message following
		OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError,
				"WriteLonUsbMsg: Created frame header with frame sync 0x%02X, frame code 0x%02X, parameter 0x%02X, checksum 0x%02X",
				state->downlink_buffer_staging.frame_header.frame_sync,
				state->downlink_buffer_staging.frame_header.frame_code,
				state->downlink_buffer_staging.frame_header.parameter,
				state->downlink_buffer_staging.frame_header.checksum);
		if (LON_SUCCESS(status)) {
			if (input_frame->short_pdu_length == EXT_LENGTH) {
				// This is an extended message (length > 254)--cast source to extended frame
				const LonExtDataFrame *input_ext_frame = (LonExtDataFrame *) input_frame;
				// Create extended USB network interface destination frame
				LonNiExtendedFrame *usb_ni_ext_frame = (LonNiExtendedFrame *) &state->downlink_buffer_staging.usb_ni_data_frame;
				// Get extended length from source frame and convert to host byte order
				size_t ext_length = ntohs(input_ext_frame->ext_length_be);
				if(ext_length <= MAX_LON_MSG_EX_LEN) {
					// Set the extended length flag
					usb_ni_ext_frame->ext_flag = EXT_LENGTH;
					// Increment length to include network interface command byte
					// and store in network byte order (big-endian)
					usb_ni_ext_frame->ext_length_be = htons(ext_length + 1);
					// Copy network interface command
					usb_ni_ext_frame->ni_command = input_ext_frame->ni_command;
					// Copy source to destination extended PDU, excluding length and flag bytes copied above
					memcpy(&usb_ni_ext_frame->ext_pdu, &input_ext_frame->ext_pdu, ext_length - 3);
					// Update sizes
					// TODO: Check this calculation; the ext_length includes the ni_command byte and the ext_pdu length, but the ext_pdu in the destination frame does not include the ni_command byte
					state->downlink_buffer_staging.buf_size = sizeof(LonFrameHeader) + sizeof(LonNiExtendedFrame) - sizeof(usb_ni_ext_frame->ext_pdu) + ext_length;
					state->downlink_buffer_staging.data_frame_size = state->downlink_buffer_staging.buf_size - sizeof(LonFrameHeader);
					status = LonStatusNoError;
				} else {
					status = LonStatusInvalidBufferLength;
				}
			} else {
				// This is a non-extended message (length <= 254)--create
				// non-extended USB network interface destination frame
				LonNiFrame *usb_ni_data_frame = &state->downlink_buffer_staging.usb_ni_data_frame;
				// Test for network interface layer mode command from the client
				LonNiCommand lonNiCommand = input_frame->ni_command;
				size_t adjusted_short_pdu_length = input_frame->short_pdu_length;
				if (state->lon_usb_iface_type == LON_USB_INTERFACE_U50 
						&& (lonNiCommand == LonNiLayerModeCmd && adjusted_short_pdu_length)) {
					// For MIP/U50 layer mode command, eliminate the network interface command
					// parameter byte from the PDU and adjust lengths
					adjusted_short_pdu_length = 0;
					usb_ni_data_frame->short_pdu_length = adjusted_short_pdu_length + 1;
					usb_ni_data_frame->ni_command = (input_frame->pdu[0] == 0) ? LonNiSetL5ModeCmd : LonNiSetL2ModeCmd;
					// Set the interface mode used for this link after NI reset
					state->configured_iface_mode = (input_frame->pdu[0] == 0) ? LON_IFACE_MODE_LAYER5 : LON_IFACE_MODE_LAYER2;
				} else {
					// Increment length to include network interface command byte (TODO: Is this checksum instead?)
					usb_ni_data_frame->short_pdu_length = adjusted_short_pdu_length + 1;
					// Copy network interface command
					usb_ni_data_frame->ni_command = input_frame->ni_command;
				}
				if (adjusted_short_pdu_length) {
					// Copy source to destination PDU
					memcpy(usb_ni_data_frame->pdu, input_frame->pdu, adjusted_short_pdu_length);
				}
				// Update sizes
				size_t unused_pdu_bytes = sizeof(usb_ni_data_frame->pdu) - adjusted_short_pdu_length;
				state->downlink_buffer_staging.buf_size = sizeof(LonUsbQueueBuffer) - unused_pdu_bytes;
				state->downlink_buffer_staging.data_frame_size = sizeof(LonNiFrame) - unused_pdu_bytes;
				status = LonStatusNoError;
			}
		}
		state->downlink_buffer_staging.priority = (input_frame->ni_command == (LonNiCommandMask|LonNiPriorityTxnQueue)) ? HIGH_PRIORITY : NORMAL_PRIORITY;
		if (LON_SUCCESS(status)) {
			OsalPrintLog(INFO_LOG, LonStatusNoError,
					"Queue downlink buffer with %zu bytes, 0x%02X command",
					state->downlink_buffer_staging.buf_size, state->downlink_buffer_staging.usb_ni_data_frame.ni_command);
			status = QueueDownlinkBuffer(&state->downlink_buffer_staging);
			if (!LON_SUCCESS(status)) {
				OsalPrintLog(ERROR_LOG, status, "WriteLonUsbMsg: Failed to queue downlink buffer");
			}
		}
	}
	OsalUnlockMutex(&state->state_lock);
	return status;
}

/*
 * Creates a LON frame header.
 * Parameters:
 *   frame_header: pointer to LonFrameHeader structure to populate
 *   frame_command: frame command value
 *   sequence_num: frame sequence number
 *   ack: acknowledgment flag
 *   parameter: frame parameter value
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode CreateFrameHeader(LonFrameHeader* frame_header,
				uint8_t frame_command, uint8_t parameter,
				uint8_t sequence_num, bool ack)
{
	OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, "CreateFrameHeader: Creating frame header with frame command 0x%02X, parameter 0x%02X, sequence %d, ack %s",
			frame_command, parameter, sequence_num, ack ? "true" : "false");
	frame_header->frame_sync = FRAME_SYNC;
	IZOT_SET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_CMD, frame_command);
	IZOT_SET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_SEQ, sequence_num);
	IZOT_SET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_ACK, ack);
	frame_header->parameter = parameter;
	frame_header->checksum = -ComputeChecksum((uint8_t *)frame_header, sizeof(LonFrameHeader) - 1);
	OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, "CreateFrameHeader: Created frame header with frame sync 0x%02X, frame code 0x%02X, parameter 0x%02X, checksum 0x%02X",
			frame_header->frame_sync, frame_header->frame_code, frame_header->parameter, frame_header->checksum);
	return LonStatusNoError;
}

/*
 * Computes the checksum for a LON USB packet.
 * Parameters:
 *   pkt: pointer to the packet data
 *   len: length of the packet data
 * Returns:
 *   Computed checksum byte
 * Notes:
 *   Checksum is computed as the 8-bit sum of all bytes in the packet.
 *   The packet may be the frame header or the full message payload.
 *   When computing a checksum for a packet, the checksum byte is negated
 *   and appended to the end of the packet.  When verifying a packet checksum,
 *   the checksum byte is included in the sum and the result should be zero.
 *   Called by WriteLonUsbMsg() and ProcessUplinkBytes().
 */
uint8_t ComputeChecksum(uint8_t *pkt, size_t len)
{
	size_t i;
	uint8_t sum = 0;
	for(i=0; i<len; i++) {
		sum += pkt[i];
	}
	return sum;
}

/*
 * Validates the checksum for a LON USB packet.
 * Parameters:
 *   pkt: pointer to the packet data
 *   len: length of the packet data
 * Returns:
 *   true if the checksum is valid; false otherwise
 * Notes:
 *   Called by ProcessUplinkBytes().
 */
static bool ValidateChecksum(uint8_t *pkt, size_t len)
{
	LonUsbLinkState *state = &iface_state;

	bool is_valid = ComputeChecksum(pkt, len) == 0;
	if (!is_valid) {
		OsalPrintLog(ERROR_LOG, LonStatusChecksumError, "ValidateChecksum: Invalid checksum computed for %zu-byte packet",len);
		Increment32(state->lon_stats.uplink.rx_checksum_errors);
	}
	return is_valid;
}

/*
 * Increments the downlink sequence number modulo 8, skipping 0.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode IncrementDownlinkSequence()
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	state->downlink_seq_number += 1;
	state->downlink_seq_number &= SEQ_NUM_MASK;	// Modulo 8
	if(state->downlink_seq_number == 0) {
		state->downlink_seq_number = 1;	// Sequence number 0 is not used
	}
	return LonStatusNoError;
}

/*
 * Writes the downlink message to the LON USB interface with byte expansion.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Sends a code packet, expands the message by stuffing a FRAME_SYNC byte
 *   after any embedded FRAME_SYNC bytes, computes and adds a checksum,
 *   and writes the expanded message to the USB interface.  Called by
 *   ProcessDownlinkRequests().
 */
LonStatusCode WriteDownlinkMessage()
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	uint8_t *dest_exp_ni_data_frame_ptr, *src_data_frame, *src_data_frame_ptr, message_byte;
 	size_t unexpanded_length, remaining;
 	size_t expanded_length;
 	uint8_t checksum;
	// Send code packet with ACK prior to the message packet
	state->downlink_ack_seq_number = ACK_WITH_NEXT_SEQ_NUM;
	WriteDownlinkCodePacket(MSG_FRAME_CMD, 1);
	// Build the downlink message packet with byte expansion for FRAME_SYNC bytes
 	src_data_frame = (uint8_t *)&state->downlink_buffer.usb_ni_data_frame;
	src_data_frame_ptr = src_data_frame;
	unexpanded_length = expanded_length = remaining 
			= state->downlink_buffer.data_frame_size;
	dest_exp_ni_data_frame_ptr = state->exp_downlink_ni_data_frame;
	checksum = -ComputeChecksum(src_data_frame_ptr, unexpanded_length);
	while(remaining--) {
		message_byte = *dest_exp_ni_data_frame_ptr++ = *src_data_frame_ptr++;
		if(message_byte == FRAME_SYNC) {
			*dest_exp_ni_data_frame_ptr++ = FRAME_SYNC;
			expanded_length++;
		}
		if(expanded_length >= sizeof(state->exp_downlink_ni_data_frame) - 2) {
			status = LonStatusInvalidBufferLength;
			OsalPrintLog(ERROR_LOG, status, "WriteDownlinkMessage: Expanded downlink message exceeds buffer size");
			return status;
		}
	}
	if((*dest_exp_ni_data_frame_ptr++ = checksum) == FRAME_SYNC) {
		*dest_exp_ni_data_frame_ptr++ = FRAME_SYNC;
		expanded_length++;	// For stuffed checksum FRAME_SYNC byte
	}
	expanded_length++;		// For checksum byte
	// Optionally log the message being sent
	OsalPrintLog(DETAIL_TRACE_LOG, status, "WriteDownlinkMessage: Send message to USB with code 0x%02X, %zu bytes before expansion, %zu bytes after expansion, seq %d",
			state->downlink_buffer.usb_ni_data_frame.ni_command,
			unexpanded_length, expanded_length, state->downlink_seq_number);
	OsalPrintMessage(DETAIL_TRACE_LOG, "WriteDownlinkMessage (not expanded): ",
			src_data_frame, unexpanded_length);
	OsalPrintMessage(DETAIL_TRACE_LOG, "WriteDownlinkMessage (expanded):     ",
			state->exp_downlink_ni_data_frame, expanded_length);
	if (state->downlink_state >= DOWNLINK_IDLE_START) {
		// Start the ACK timer for the downlink message
		if (!LON_SUCCESS(status = StartAckTimer(DOWNLINK_WAIT_MSG_ACK))) {
			OsalPrintLog(ERROR_LOG, status, "WriteDownlinkMessage: Failed to start ACK timer for downlink message");
			return status;
		}
	}
	// Write the expanded message to the LON USB interface
	size_t bytes_written = 0;
	status = HalWriteUsb(0, state->exp_downlink_ni_data_frame, expanded_length, &bytes_written);
	if (!LON_SUCCESS(status) || bytes_written != expanded_length) {
		OsalPrintLog(INFO_LOG, status, "WriteDownlinkMessage: Wrote %zu of %d bytes to LON USB interface", bytes_written, expanded_length);
		Increment32(state->lon_stats.downlink.tx_aborted_errors);
		return (!LON_SUCCESS(status)) ? LonStatusWriteFailed : status;
	}
	// Update statistics
	Increment32(state->lon_stats.downlink.packets_sent);
	Add32(state->lon_stats.downlink.bytes_sent, bytes_written);
	return status;
}

/*
 * Starts the acknowledgment timer for the uplink.
 * Parameters:
 *   next_state: next downlink state to set
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode StartAckTimer(DownlinkState next_state)
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	state->uplink_ack_timer = OsalGetTickCount();
	state->uplink_ack_timeouts = 0;
	state->downlink_state = next_state;
	return LonStatusNoError;
}

/*
 * Writes a downlink code packet to the LON USB network interface.
 * Parameters:
 *   frame_cmd: frame command value
 *   parameter: frame parameter value
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode WriteDownlinkCodePacket(LonUsbFrameCommand frame_cmd, uint8_t parameter)
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	LonFrameHeader frame_header;
	int sequence_num = 0;
	// Build the frame header
	memset(&frame_header, 0, sizeof(frame_header));
	frame_header.frame_sync = FRAME_SYNC;
	IZOT_SET_ATTRIBUTE(frame_header, IZOT_U50_FRAME_CODE_CMD, frame_cmd);
	if ((state->downlink_ack_seq_number >= 0)) {
		sequence_num = state->downlink_ack_seq_number;
		IZOT_SET_ATTRIBUTE(frame_header, IZOT_U50_FRAME_CODE_SEQ, sequence_num);
	} else if ((frame_cmd == MSG_FRAME_CMD) || (state->downlink_ack_seq_number == ACK_WITH_NEXT_SEQ_NUM)) {
		sequence_num = state->downlink_seq_number;
		IZOT_SET_ATTRIBUTE(frame_header, IZOT_U50_FRAME_CODE_SEQ, sequence_num);
	}
	if (state->downlink_ack_seq_number > NO_ACK_REQUIRED) {
		if (frame_cmd != FAIL_FRAME_CMD) {
			// Frame command is not reporting a failure; set the ACK flag
			IZOT_SET_ATTRIBUTE(frame_header, IZOT_U50_FRAME_CODE_ACK, 1);
		}
		state->downlink_ack_seq_number = NO_ACK_REQUIRED;
	}
	frame_header.parameter = parameter;
	// Compute the checksum and negate it; don't include the checksum byte itself
	frame_header.checksum = -ComputeChecksum((uint8_t *)&frame_header, sizeof(frame_header)-1);
	// Send the code packet
	size_t bytes_written = 0;
	status = HalWriteUsb(0, (uint8_t *)&frame_header, sizeof(frame_header), &bytes_written);
	if (!LON_SUCCESS(status)) {
		OsalPrintLog(ERROR_LOG, status, "WriteDownlinkCodePacket: 'HalWriteUsb' Failed to write downlink code packet with command 0x%02X, parameter 0x%02X, sequence %02X",
				frame_cmd, parameter, sequence_num);
		return status;
	}
	if (bytes_written != sizeof(frame_header)) {
		OsalPrintLog(ERROR_LOG, LonStatusLniWriteFailure, "WriteDownlinkCodePacket: 'bytes_written' Failed to write downlink code packet with command 0x%02X, parameter 0x%02X, sequence %02X",
				frame_cmd, parameter, sequence_num);
		return LonStatusLniWriteFailure;
	}
	// Reset sequence number to 0 for the next message if this is a sync frame code
	if (frame_cmd == NI_RESYNC_FRAME_CMD) {
		state->downlink_seq_number = 0;
		state->uplink_seq_number = 0;
	}
	OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, "WriteDownlinkCodePacket: Sent downlink code packet with command 0x%02X, parameter 0x%02X, sequence %02X",
			frame_cmd, parameter, sequence_num);
	return status;
}

/*
 * Writes a downlink local NI command to the LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode WriteDownlinkLocalNiCmd()
{
    LonUsbLinkState *state = &iface_state;
	if (state == NULL || state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	uint8_t cmd = (uint8_t)state->downlink_buffer_staging.usb_ni_data_frame.ni_command;
	state->uplink_expected_rsp = GetExpectedResponse(cmd);
	DownlinkState next_downlink_state = state->uplink_expected_rsp ? DOWNLINK_WAIT_CP_RESPONSE : DOWNLINK_WAIT_CP_ACK;
	if (state->downlink_state == DOWNLINK_WAIT_STARTUP_RESET) {
		// This is a startup command that may include
		// an interface mode change; keep the current state
		next_downlink_state = DOWNLINK_WAIT_STARTUP_RESET;
	}
	OsalPrintLog(INFO_LOG, LonStatusNoError, "Send code packet with local NI command 0x%02X with 0x%02X expected response",
			cmd, state->uplink_expected_rsp);
	status = WriteDownlinkCodePacket(SHORT_NI_CMD_FRAME_CMD, cmd);
	if (!LON_SUCCESS(status)) {
		OsalPrintLog(ERROR_LOG, status, "Failed to write downlink local NI command 0x%02X", cmd);
		return status;
	}
	if (state->downlink_state >= DOWNLINK_IDLE_START) {
		// Start the ACK timer for the local NI command
		if (!LON_SUCCESS(status = StartAckTimer(next_downlink_state))) {
			OsalPrintLog(ERROR_LOG, status, "Failed to start ACK timer for local NI command 0x%02X", cmd);
			return status;
		}
	}
	return status;
}

/*
 * Gets the expected response NI command for a given NI command.
 * Parameters:
 *   cmd: NI command byte
 * Returns:
 *   Expected response NI command; LonNiClearCmd if no response is expected
 * Notes:
 *   Some versions of the MIP firmware have the following behavior:
 *   1. The host sends a SHORT_NI_CMD_FRAME_CMD code packet that requires a response.
 *   2. The MIP responds with a code packet ACK, but the ACK is in response to a
 *      previous command, not the current one.  For example, the MIP may respond
 *      with an ACK to a previous command because the host has not yet acknowledged
 *      a previously pending uplink short command.
 *   3. The host sends another SHORT_NI_CMD_FRAME_CMD code packet overwriting the
 *      command sent in number 1.
 *   To prevent losing the first command, a downlink short command is not acknowledged
 *   if there is another downlink short command pending.  Instead, the host waits for
 *   the expected response to the command before sending an ACK.
 *   Called by WriteDownlinkLocalNiCmd().
 */
LonNiCommand GetExpectedResponse(uint8_t cmd)
{
	LonNiCommand expected_response = LonNiClearCmd;
	switch(cmd) {
		case LonNiStatusCmd:
		case LonNiSetL5ModeCmd:
		case LonNiSetL2ModeCmd:
			expected_response = cmd;
			break;
		case LonNiInitiateCmd:
			expected_response = LonNiChallengeResponse;
			break;
	}
	return expected_response;
}

/*
 * Resets the downlink state for a specified LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode ResetDownlinkState()
{
    LonUsbLinkState *state = &iface_state;
	LonStatusCode status = LonStatusNoError;
	OsalLockMutex(&state->state_lock);
	if (state->downlink_buffer.buf_size > 0) {
		memset(&state->downlink_buffer, 0, sizeof(state->downlink_buffer));
	}
	state->downlink_reject_timer = 0;
	OsalUnlockMutex(&state->state_lock);
	// Clear the ack timer and change state to idle
	status = StopAckTimer();
	return status;
}

/*
 * Stops the acknowledgment timer for the uplink and sets the downlink state
 * to idle if the current downlink state is not the startup state.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode StopAckTimer()
{
    LonUsbLinkState *state = &iface_state;
	if (state->downlink_state > DOWNLINK_IDLE) {
		state->downlink_state = DOWNLINK_IDLE;
	}
	state->uplink_ack_timer = 0;
	state->uplink_ack_timeout = ACK_WAIT_TIME;
	state->uplink_ack_timeout_phase = 0;
	state->uplink_ack_timeouts = 0;
	return LonStatusNoError;
}

// TODO: Verify that length stuffing is correct for extended messages (see the U61 code)
// TODO: Zero out the downlink buffer before use to avoid garbage in unused fields
// TODO: Verify handling of local NI commands
// TODO: Verify handling of LON protocol V1

/*****************************************************************
 * Section: Uplink Function Definitions
 *****************************************************************/
/*
 * Reads an uplink message from the LON USB interface, if available.
 * Parameters:
 *   out_msg: pointer to L2Frame or LdvExtendedMessage structure
 * Returns:
 *   LonStatusNoError if a message is successfully read;
 *   LonStatusNoMessageAvailable if no full message is available;
 *   LonStatusCode error code if unsuccessful
 * Notes:
 *   Non-blocking; reads available bytes and returns a single message if
 *   available.  If no full message is available, tests for timeout waiting
 *   for the LON interface unique ID (UID) and retries the UID read request.
 *   TODO: The UID retry may be redundant with the uplink state machine.
 */
LonStatusCode ReadLonUsbMsg(LonDataFrame *out_msg)
{
    LonUsbLinkState *state = &iface_state;
	if (!out_msg) {
		return LonStatusInvalidParameter;
	}
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;

	// Stage 1: attempt a non-blocking read into a temporary buffer
	// Check available ring capacity under queue_lock to avoid over-pulling from USB
	size_t rb_avail = 0;
	// Lock ring to safely check available capacity; another thread may feed concurrently
	// Protect queue head access; other threads may enqueue/dequeue concurrently
	OsalLockMutex(&state->queue_lock);
	rb_avail = RingBufferAvail(&state->lon_usb_uplink_ring_buffer);
	OsalUnlockMutex(&state->queue_lock); // Keep critical section minimal
	if (rb_avail > MAX_BYTES_PER_USB_READ) {
		uint8_t temp_read_buf[MAX_BYTES_PER_USB_READ];
		ssize_t bytes_read = 0;
		status = HalReadUsb(0, temp_read_buf, sizeof(temp_read_buf), &bytes_read);
		if (LON_SUCCESS(status) && bytes_read > 0) {
			// Lock ring while writing staged bytes and updating staging stats
			OsalLockMutex(&state->queue_lock);
			size_t written = RingBufferWrite(&state->lon_usb_uplink_ring_buffer, temp_read_buf, (size_t)bytes_read);
			size_t occ = RingBufferSize(&state->lon_usb_uplink_ring_buffer);
			state->lon_stats.usb_rx.bytes_fed += written;
			if (occ > state->lon_stats.usb_rx.max_occupancy) state->lon_stats.usb_rx.max_occupancy = occ;
			if (written < (size_t)bytes_read) {
				size_t dropped = (size_t)bytes_read - written;
				state->lon_stats.usb_rx.bytes_dropped += dropped;
				OsalUnlockMutex(&state->queue_lock); // Release before logging
				OsalPrintLog(ERROR_LOG, LonStatusOverflow, "RX ring overflow: dropped %zu of %zd bytes", dropped, bytes_read);
			} else {
				OsalUnlockMutex(&state->queue_lock); // Leave lock before returning
			}
		} else if (!LON_SUCCESS(status) && status != LonStatusNoMessageAvailable) {
			// Propagate hard error (timeout/device/read failure)
			return status;
		}
	}

	// Stage 2: drain any accumulated bytes from the ring buffer in modest
	// chunks and feed the parser--this allows smoothing of bursty UART/USB
	// input and reduces immediate stack usage; skip if no space in uplink buffer;
	// lock ring to snapshot count--parser will process outside of lock;
	// protect pop of head node to avoid races with concurrent writers/readers
	OsalLockMutex(&state->queue_lock);
	size_t ring_count = RingBufferSize(&state->lon_usb_uplink_ring_buffer);
	OsalUnlockMutex(&state->queue_lock); // Free outside of lock to keep critical section small
	if (ring_count > 0 && GetUplinkBufferCount() < MAX_LON_UPLINK_BUFFERS) {
		uint8_t parse_chunk[MAX_BYTES_PER_USB_PARSE_CHUNK];
		size_t processed_in_window = 0;
		for (;;) {
			// Lock ring for atomic read of a chunk--keep scope tight;
			// protect tail walk and append under lock--single-linked O(n) by design
			OsalLockMutex(&state->queue_lock);
			size_t cur_count = RingBufferSize(&state->lon_usb_uplink_ring_buffer);
			if (cur_count == 0) {
				OsalUnlockMutex(&state->queue_lock);
				break;
			}
			size_t chunk_size = RingBufferRead(&state->lon_usb_uplink_ring_buffer, parse_chunk, sizeof(parse_chunk));
			OsalUnlockMutex(&state->queue_lock);
			if (chunk_size == 0) break; // Should not happen
			ProcessUplinkBytes(parse_chunk, chunk_size);
			// Update staging stats under lock to avoid races with feeder
			// Protect O(n) count traversal to avoid concurrent mutation
			OsalLockMutex(&state->queue_lock);
			state->lon_stats.usb_rx.bytes_read += chunk_size;
			OsalUnlockMutex(&state->queue_lock);
			processed_in_window += chunk_size;
			// Break early if a message has been queued to minimize per-call latency
			if (GetUplinkBufferCount() > 0) break;
			// Guard against monopolizing CPU if ring is very full; parse only a window per call
			if (processed_in_window > MAX_BYTES_PER_USB_PARSE_WINDOW) break; // Arbitrary processing budget
		}
	}

	// Stage 3: if a message is available, return it
	static LonUsbQueueBuffer buffer;
	if ((!LON_SUCCESS(status = ReadUplinkBuffer(&buffer))) && status != LonStatusNoBufferAvailable) {
		return status;
	}
	if (status != LonStatusNoBufferAvailable) {
		// Message is available; copy to output buffer
		memset(out_msg, 0, sizeof(*out_msg));
		// Field assignment converts from LonNiFrame to LonDataFrame; the PDU is copied separately below
		out_msg->ni_command = buffer.usb_ni_data_frame.ni_command;
		// TODO: Verify extended message handling.  short_pdu_length includes the command byte
		// TODO: Verify the -1 for PDU length; the length sent by the MIP includes the command byte, but the command byte is not included in the PDU field in the output message
		out_msg->short_pdu_length = buffer.usb_ni_data_frame.short_pdu_length - 1;
		//if (out_msg->short_pdu_length > sizeof(out_msg->pdu)) {   condition is always false 
			//out_msg->short_pdu_length = (uint8_t) sizeof(out_msg->pdu);
		//}
		memcpy(out_msg->pdu, buffer.usb_ni_data_frame.pdu, out_msg->short_pdu_length);
		return LonStatusNoError;
	}

	// No uplink message available
	return LonStatusNoBufferAvailable;
}

/* 
 * Processes uplink bytes received from the LON USB interface.
 * Parameters:
 *   iface_index: interface index returned by OpenLonUsbLink()
 *   chunk: pointer to buffer of received bytes
 *   chunk_size: number of bytes in chunk
 * Returns:
 *   None
 * Notes:
 *   Called by ReadLonUsbMsg() after reading available bytes from the
 *   LON USB interface into a temporary buffer.  Calls CheckUplinkCompleted()
 *   to check for completed uplink messages.
 * 
 *   Code packets contain the following, with byte-stuffing of FRAME_SYNC (0x7E)
 *   bytes in the message data:
 *   - Byte 0: FRAME_SYNC (0x7E)
 *   - Byte 1: 0x00 for MIP/U61 (U10 FT Rev A/B, U20 PL, U60 TP-1250, U70 PL)
 * 			   <SSSA:CCCC> for MIP/U50 (U10 FT Rev C, U60 FT) where:
 * 			   -- SSS = 3-bit sequence number; 0xE0 mask; increments per message;
 * 				  used only for messages, all other set to 0
 * 			   -- A = 1-bit ACK/NAK flag; 0x10 mask; set to 1 for ACK, 0 for NAK
 * 			   -- CCCC = 4-bit frame code; 0x0F mask; see U50_PACKET_CODE enumeration
 *   - Byte 2: For MIP/U50 only: code packet parameter, for example:
 * 			   -- 0: for CpNull (0), CpNiResync (7) (see U50LinkStart)
 * 			   -- 1: for CpMsg (2)
 * 			   -- <command>: for CpNiCmdShort (6); LonNiResetDeviceCmd (0x50) (see smpDlNiCmdLocal)
 *   - Bytes 3: For MIP/U50 only: checksum byte (negated sum of all bytes modulo 256)
 *   - Bytes 4+: PDU data bytes (if any)
 * 
 *   For MIP/U50: 
 *   - Send CpNiCmdShort/LonNiSetL2ModeCmd (0xD1), CpNiCmdShort/LonNiStatusCmd (0xE0),
 *     and CpNiCmdShort/0 commands to set layer 2 mode
 *   - Send LonNiResetDeviceCmd (0x50) command to reset the NIC; create a ResetNi()
 *     function to reset the network interface when the NI no longer accepts
 *     outgoing messages, for example after a timeout waiting for an ACK; process incoming
 *   - Send CpMsg (2) code packets for downlink messages
 *   - Send CpNiResync (7) followed by a 0 parameter to resync
 *   - Send CpNiCmdShort/LonNiStatusCmd (0xE0) to get TX errors, RX errors,
 *     network input buffer count, and application input buffer count for the MIP
 *   - Respond to LonNiStatusCmd (0xE0), LonNiSetL5ModeCmd (0xD0), and LonNiSetL2ModeCmd (0xD1)
 *     commands with the same command
 *   - Respond to nicbMODE_SSI (0x42) command with nicbMODE (0x40) command
 *   - Respond to LonNiInitiateCmd (0x51) command with LonNiChallengeResponse (0x52) command
 *   - Ignore nicbACK (0xC0) and nicbNACK (0xC1) commands
 *   - Use LonNiError (0x30) to process runt packets
 * 
 * TODO: See smpCodePacketRxd() in U50Link.c; reviewed through line 900
 * TODO: Add new uplink_state values for U50_LINK
 * TODO: Add line discipline mapping for the U50_LINK and U61_LINK types
 */
LonStatusCode ProcessUplinkBytes(uint8_t *chunk, size_t chunk_size)
{
	bool completed = false;
	LonStatusCode status = LonStatusNoError;
    LonUsbLinkState *state = &iface_state;
	static UplinkState last_uplink_state = UPLINK_INVALID; // For debugging
	if (last_uplink_state != state->uplink_state) {
		OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, "ProcessUplinkBytes: Uplink state changed from %s to %s",
				uplink_state_names[last_uplink_state], uplink_state_names[state->uplink_state]);
		last_uplink_state = state->uplink_state;
	}
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	while (chunk_size && !state->shutdown && LON_SUCCESS(status)) {
		switch(state->uplink_state) {
		case UPLINK_IDLE:
			// Looking for frame sync byte
			if (*chunk == FRAME_SYNC) {
				state->uplink_buffer.frame_header.frame_sync = *chunk;
				state->uplink_state = UPLINK_FRAME_CODE;
				state->uplink_frame_error = FALSE;
				OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, "ProcessUplinkBytes: Uplink frame sync received, change state to UPLINK_FRAME_CODE");
			} else if (!state->uplink_frame_error) {
				// Frame sync expected but not received; increment error count
				// and stay in UPLINK_IDLE to continue looking for frame sync
				Increment32(state->lon_stats.uplink.rx_frame_errors);
				OsalPrintLog(ERROR_LOG, LonStatusFrameError, "ProcessUplinkBytes: First uplink frame error, %02X received 0x7E expected", *chunk);
				state->uplink_frame_error = TRUE; // TODO: Check this
			}
			chunk++; chunk_size--;
			break;
		case UPLINK_FRAME_CODE:
			if (state->lon_usb_iface_type == LON_USB_INTERFACE_U50) {
				// MIP/U50: save frame code byte fields
				IZOT_SET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_ACK, (*chunk & 0x10) ? 1 : 0);
				IZOT_SET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_SEQ, (*chunk >> 5) & 0x07);
				IZOT_SET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_CMD, *chunk & 0x0F);
				state->uplink_state = UPLINK_FRAME_PARAMETER;
				OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError,
						"ProcessUplinkBytes: Uplink frame code received: cmd 0x%02X, seq %d, ack %d; change state to UPLINK_FRAME_PARAMETER",
						IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_CMD),
						IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_SEQ),
						IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_ACK));
				chunk++; chunk_size--;
			} else {
				// MIP/U61: frame code byte is always 0
				if (*chunk == 0) {
					chunk++; chunk_size--;
					state->uplink_msg_timer = OsalGetTickCount();
					state->uplink_state = UPLINK_MESSAGE;
				} else {
					Increment32(state->lon_stats.uplink.rx_frame_errors);
					OsalPrintLog(ERROR_LOG, LonStatusFrameError,
							"ProcessUplinkBytes: Uplink frame error (2) - %02X", *chunk);
					state->uplink_state = UPLINK_IDLE;
					state->uplink_frame_error = TRUE;
				}
			}
			break;
		case UPLINK_FRAME_PARAMETER:
			// For MIP/U50 only: save frame parameter byte
			state->uplink_buffer.frame_header.parameter = *chunk;
			chunk++; chunk_size--;
			state->uplink_state = UPLINK_CODE_PACKET_CHECKSUM;
			OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError,
					"ProcessUplinkBytes: Uplink frame parameter received: 0x%02X; change state to UPLINK_CODE_PACKET_CHECKSUM",
					state->uplink_buffer.frame_header.parameter);
			break;
		case UPLINK_CODE_PACKET_CHECKSUM:
			// For MIP/U50 only: the checksum byte is the final byte of a code packet;
			// save frame checksum and process the code packet
			state->uplink_buffer.frame_header.checksum = *chunk;
			if (!ValidateChecksum((uint8_t *) &state->uplink_buffer.frame_header,
					sizeof(state->uplink_buffer.frame_header))) {
				// Code packet checksum is invalid; flag frame error and go back to idle
				state->uplink_frame_error = TRUE;
				state->downlink_ack_seq_number = NO_ACK_REQUIRED;
				OsalPrintLog(ERROR_LOG, LonStatusFrameError, "ProcessUplinkBytes: Uplink code packet checksum error");
				state->uplink_state = UPLINK_IDLE;
			} else {
				// Process the code packet
				status = ProcessUplinkCodePacket();
				if (!LON_SUCCESS(status)) {
					return status;
				}
				if (IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_CMD) == MSG_FRAME_CMD) {
					// Message frame; receive uplink message
					state->uplink_msg_timer = OsalGetTickCount();
					state->uplink_state = UPLINK_MESSAGE;
				} else {
					// Not a message frame; go back to idle
					state->uplink_state = UPLINK_IDLE;
				}
			}
			chunk++; chunk_size--;
			break;
		case UPLINK_MESSAGE:
			while (chunk_size) {
				uint8_t msg_byte 
						= ((uint8_t *)&state->uplink_buffer.usb_ni_data_frame)[state->uplink_msg_index++] 
						= *chunk++;
				chunk_size--;
				if (msg_byte == FRAME_SYNC) {
					state->uplink_state = UPLINK_ESCAPED_DATA;
					break;
				}
				status = CheckUplinkCompleted(&completed);
				if (!LON_SUCCESS(status)) {
					return status;
				}
				if (completed) {
						break;
				}
			}
			if (state->uplink_state != UPLINK_IDLE
					&& (OsalGetTickCount() - state->uplink_msg_timer) > DEFAULT_LLP_TIMEOUT) {
				Increment32(state->lon_stats.uplink.rx_timeout_errors);
				OsalPrintLog(INFO_LOG, LonStatusFrameTimeout, "ProcessUplinkBytes: Uplink frame timeout");
				ResetUplinkState();
			}
			break;
		case UPLINK_ESCAPED_DATA:
			state->uplink_state = UPLINK_MESSAGE;
			if (*chunk != FRAME_SYNC) {
				Increment32(state->lon_stats.uplink.rx_frame_errors);
				state->downlink_ack_seq_number = NO_ACK_REQUIRED;
				OsalPrintLog(ERROR_LOG, LonStatusFrameError, "ProcessUplinkBytes: Uplink frame error (data)");
				if (*chunk == 0) {
					state->uplink_msg_index = 0;
				} else {
					ResetUplinkState();
				}
			}
			chunk++; chunk_size--;
			status = CheckUplinkCompleted(&completed);
			break;

		default: break; // UPLINK_INVALID is not handled 
		}
	}
	return status;
}

LonStatusCode ProcessUplinkCodePacket()
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	PrintUplinkCodePacket("ProcessUplinkCodePacket: ");
	LonFrameHeader *frame_header = (LonFrameHeader *)&state->uplink_buffer.frame_header;
	LonUsbFrameCommand frame_cmd = IZOT_GET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_CMD);
	uint8_t parameter = frame_header->parameter;
	bool ack_flag = IZOT_GET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_ACK) != 0;
	uint8_t sequence_number = IZOT_GET_ATTRIBUTE(*frame_header, IZOT_U50_FRAME_CODE_SEQ);
	bool cp_fail_or_reject = ((frame_cmd == FAIL_FRAME_CMD) || (frame_cmd == MSG_REJECT_FRAME_CMD));
	if (frame_cmd) {
		if (state->uplink_seq_number == sequence_number && !cp_fail_or_reject) {
			// Duplicate sequence number; process it partially to receive and
			// dump any following message into the uplink buffer without queuing it
			state->uplink_duplicate = true;
			Increment32(state->lon_stats.uplink.rx_duplicates);
		}
		state->uplink_seq_number = sequence_number;
	}
	if (ack_flag && !cp_fail_or_reject) {
		OsalPrintLog(DETAIL_TRACE_LOG, status, "ProcessUplinkCodePacket: ACK flag set for sequence %d",
				sequence_number);
		if (state->downlink_state != DOWNLINK_WAIT_CP_RESPONSE) {
			// Process ACK flag for good code packets that are not local request/response commands
			ClearUplinkTransaction();
		}
	}
	switch(frame_cmd) {
	case MSG_FRAME_CMD:
		// Message frame header received; change state to receive message in CheckUplinkCompleted()
		OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessUplinkCodePacket: Received uplink message frame with sequence %d, ack %d, parameter 0x%02X",
				sequence_number, ack_flag, parameter);
		state->downlink_ack_seq_number = sequence_number;
		if (ack_flag) {
			OsalPrintLog(DETAIL_TRACE_LOG, status, "ProcessUplinkCodePacket: ACK flag set for message with sequence %d",
					sequence_number);
		}
		state->uplink_state = UPLINK_MESSAGE;
		break;
	case SHORT_NI_CMD_FRAME_CMD:
		// Short NI command received; process the command
		OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessUplinkCodePacket: Received uplink short NI command with sequence %d, ack %d, NI command 0x%02X",
				sequence_number, ack_flag, parameter);
		state->downlink_ack_seq_number = sequence_number;
		if (ack_flag) {
			OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessUplinkCodePacket: ACK flag set for short NI command with sequence %d",
					sequence_number);
		}
		if (!state->uplink_duplicate) {
			if (parameter == LonNiAckCmd || parameter == LonNiNackCmd) {
				// Downlink ACK not required for these commands
				// TODO: Is "state->downlink_ack_required = false;" required here, or is it sufficient that these commands are not processed as part of an uplink transaction in ClearUplinkTransaction()?
				break;
			}
			if(parameter == LonNiResetDeviceCmd) {
				// Reset transmit state and force a pending downlink flush
				state->downlink_seq_number = 0;
				state->downlink_state = DOWNLINK_START;
			}
			// Translate short NI command to a local NI command and queue it uplink
			state->uplink_buffer.usb_ni_data_frame.ni_command = parameter;
	        if((parameter & 0xF0) == LonNiError) {
				state->uplink_buffer.usb_ni_data_frame.short_pdu_length = 5;
				// Zero payload fields for an error message
				memset(state->uplink_buffer.usb_ni_data_frame.pdu, 0, 4);
			} else {
      			state->uplink_buffer.usb_ni_data_frame.short_pdu_length = 1;
			}
			if (!LON_SUCCESS(status = ProcessUplinkMessage())) {
				OsalPrintLog(ERROR_LOG, status, "ProcessUplinkCodePacket: Failed to queue uplink short NI command 0x%02X",
						parameter);
				return status;
			}
		}
		// Clear uplink state
		ResetUplinkState();
		break;
	case MSG_REJECT_FRAME_CMD:
		// No room downlink; start or check a reject timer
		Increment32(state->lon_stats.downlink.tx_rejects);
		if(state->downlink_state == DOWNLINK_WAIT_MSG_ACK || state->downlink_state == DOWNLINK_WAIT_CP_ACK) {
			// Stop the ACK timer and set downlink state to idle
			StopAckTimer();
			// This timer is stopped when an ACK is received
			if (!state->downlink_reject_timer) {
				state->downlink_reject_timer = OsalGetTickCount();
				OsalPrintLog(INFO_LOG, status, "ProcessUplinkCodePacket: Reject received from downlink reject timer started");
			} else if (OsalGetTickCount() - state->downlink_reject_timer > DOWNLINK_WAIT_TIME) {
				state->downlink_reject_timer = 0;
				// Downlink reject timer limit reached; TBD: reset the network interface
				status = LonStatusLniWriteFailure;
				OsalPrintLog(ERROR_LOG, status, "ProcessUplinkCodePacket: Downlink reject timer expired; resetting network");
				WriteDownlinkCodePacket(SHORT_NI_CMD_FRAME_CMD, LonNiResetDeviceCmd);
			}
			// TODO: Ensure sequence number is cycled for DOWNLINK_WAIT_MSG_ACK
			// because the code packet was accepted
		} else {
			// This isn't necessarily an error
			OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessUplinkCodePacket: Unknown CP or message reject cause in state %d.",
					state->downlink_state);
		}
		break;
	case FAIL_FRAME_CMD:
		ClearUplinkTransaction(); //Don't keep repeating a failed message
		StopAckTimer();
		Increment32(state->lon_stats.uplink.rx_code_packet_failures);
		status = LonStatusFrameError;
		OsalPrintLog(ERROR_LOG, status, "ProcessUplinkCodePacket: Received uplink failure frame with sequence %d, ack %d, parameter 0x%02X",
				sequence_number, ack_flag, parameter);
		break;
	case NULL_FRAME_CMD:
		OsalPrintLog(PACKET_TRACE_LOG, status, "ProcessUplinkCodePacket: Received uplink null frame with sequence %d, ack %d, parameter 0x%02X",
			sequence_number, ack_flag, parameter);
		if (state->wait_for_null) {
			state->wait_for_null = false;
			state->have_null = true;
		}
		break;
	default:
		// Unknown frame command; log and increment error count
		Increment32(state->lon_stats.uplink.rx_frame_errors);
		status = LonStatusFrameError;
		OsalPrintLog(ERROR_LOG, status, "ProcessUplinkCodePacket: Received uplink frame with unknown command 0x%02X",
				frame_cmd);
		break;
	}
	return status;
}

// TODO: Verify message processing in the original LON U60 driver after ProcessUplinkCodePacket()

/*
 * Clears any pending uplink transaction for a specified LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called to complete or abort any pending uplink transaction before starting
 *   a new uplink transaction.
 */
LonStatusCode ClearUplinkTransaction()
{
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	LonStatusCode status = LonStatusNoError;
	if (state->downlink_state == DOWNLINK_WAIT_CP_ACK || state->downlink_state == DOWNLINK_WAIT_CP_RESPONSE) {
		state->downlink_buffer.usb_ni_data_frame.ni_command = 0;
		status = StopAckTimer();
	} else if (state->downlink_state == DOWNLINK_WAIT_MSG_ACK) {
		status = ResetDownlinkState();
	}
	if (!LON_SUCCESS(status)) {
		return status;
	}
	status = IncrementDownlinkSequence();
	state->uplink_expected_rsp = LonNiClearCmd;
	state->uplink_ack_timeout_phase = 0;
	return status;
}

/*
 * Checks for completed uplink (read) messages from the LON USB
 * network interface.
 * Parameters:
 *   state: pointer to LonUsbLinkState structure for the interface
 *   completed: pointer to BOOL set to TRUE if an uplink message
 * 		   is complete and has been processed; FALSE if not yet complete
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called by ReadLonUsbMsg() during incoming byte stream parsing. 
 *   If an uplink message is complete, it is processed and queued for
 *   the LON stack or application using ProcessUplinkMessage().  If an
 *   uplink message is not yet complete, the function returns FALSE.
 *   The message length is determined from the first one or three bytes
 *   of the message, depending on whether the message is an expanded or
 *   non-expanded message.  The length is in the first byte
 *   of a non-expanded uplink message as follows:
 *     <[7E][00]>[LEN][CMD][...]
 *   and is in the second and third bytes of an expanded uplink
 *   message as follows:
 *     <[7E][00]>[FF][LHI][LLO][CMD][...]
 */
LonStatusCode CheckUplinkCompleted(bool *completed)
{
	*completed = false;
	LonStatusCode status = LonStatusNoError, uplink_status = LonStatusNoError;
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
	if (state->uplink_msg_index == 1) {		// Length is at index 0; cmd is at index 1
		state->uplink_msg_length = state->uplink_buffer.usb_ni_data_frame.short_pdu_length; // TODO: Verify this is correct and that short_pdu_length is populated at this point in the code; if so, change the condition to check for short_pdu_length instead of index 1 and use the field instead of index 0
	} else if (state->uplink_msg_length == EXT_LENGTH && state->uplink_msg_index == 3){
		// Get the extended length; ((uint8_t *)&state->uplink_buffer.usb_ni_data_frame)[0] is still EXT_LENGTH
		state->uplink_msg_length = ntoh16(((LonExtDataFrame *)&state->uplink_buffer.usb_ni_data_frame)->ext_length_be);
	} else if (state->uplink_msg_index && (state->uplink_msg_index > (state->uplink_msg_length + 1))) {
		// TODO: Verify +1 is correct in the above condition
		// Full message received
		uint8_t ni_cmd = state->uplink_buffer.usb_ni_data_frame.ni_command; // TODO: Verify this is correct and that ni_command is populated at this point in the code; if so, use the field instead of index 1
		if ((ni_cmd & NI_CMD_MASK) == LonNiDriverCmdMask || ni_cmd == LonNiLayerModeCmd) {
			// Filter LonNiDriverCmdMask commands
			OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError,
					"Processed 0x%02X uplink network interface command", ni_cmd);
			if (ni_cmd == LonNiLayerModeCmd) {
				OsalPrintLog(INFO_LOG, LonStatusNoError, 
						"CheckUplinkCompleted: NI Layer Mode setting received: %d",
						state->uplink_buffer.usb_ni_data_frame.pdu[0]);
			}
		} else {
			OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError, 
					"CheckUplinkCompleted: Received 0x%02X network interface command with %d bytes",
					ni_cmd, state->uplink_msg_length);
			OsalPrintMessage(DETAIL_TRACE_LOG, "CheckUplinkCompleted: ",
					(uint8_t *)&state->uplink_buffer.usb_ni_data_frame, state->uplink_msg_length + 2);
			switch(ni_cmd) {
			case LonNiResetDeviceCmd:
				// Process and shrink any LonNiResetDeviceCmd data
				if (state->uplink_buffer.usb_ni_data_frame.short_pdu_length) {
					char channel_type_name[] = "Custom channel type";
					state->lon_stats.channel_type_id = state->uplink_buffer.usb_ni_data_frame.pdu[1];
					if (state->lon_stats.channel_type_id != IZOT_CUSTOM_CHANNEL_TYPE_ID) {
						snprintf(channel_type_name, sizeof(channel_type_name),
								"channel type %d", state->lon_stats.channel_type_id);
					}
					state->lon_stats.l2_l5_mode = state->current_iface_mode = state->uplink_buffer.usb_ni_data_frame.pdu[0];
					char lon_ni_layer_mode_name[] = "layer 5";
					if (state->lon_stats.l2_l5_mode) {
						strncpy(lon_ni_layer_mode_name, "layer 2", sizeof(lon_ni_layer_mode_name));
					}
					OsalPrintLog(INFO_LOG, LonStatusNoError, 
						"CheckUplinkCompleted: NI Reset received, %s, %s",
						channel_type_name, lon_ni_layer_mode_name);
				} else {
					OsalPrintLog(INFO_LOG, LonStatusNoError, "CheckUplinkCompleted: NI Reset received, no data");
				}
				state->uplink_buffer.usb_ni_data_frame.short_pdu_length = 0;
				state->uplink_msg_length = 1; 	// Adjust length for no data (TODO: Check this and correct checksum)
				if (state->wait_for_reset) {
					state->wait_for_reset = false;
					state->have_reset = true;		// Signal reset received to ProcessDownlinkRequests()
				}
				// TODO: Clear any pending transactions?
				// TODO: Alternate handling if not waiting for reset, for example if reset is unexpected or unsolicited
  				break;

			case LonNiCrcError:
				OsalPrintLog(INFO_LOG, LonStatusNoError, "CheckUplinkCompleted: NI CRC error received");
				Increment32(state->lon_stats.uplink.rx_crc_errors);
				break;
			case LonNiIncomingCmd:
				if (state->uplink_buffer.usb_ni_data_frame.short_pdu_length > (UNICAST_MSG_CODE_OFFSET) 
						&& state->uplink_buffer.usb_ni_data_frame.pdu[UNICAST_MSG_CODE_OFFSET] 
						== IzotNmWink) {
					// Process uplink network Wink message
					IdentifyLonNi();
				}
				break;
			case LonNiIncomingL2Cmd:
				if (state->uplink_buffer.usb_ni_data_frame.short_pdu_length == BROADCAST_MSG_CODE_OFFSET + 18
						&& state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET]
						== IzotNmServicePin) {
					// Log received unique ID message
					OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError, 
							"CheckUplinkCompleted: Received uplink unique ID from %2.2X%2.2X:%2.2X%2.2X:%2.2X%2.2X",
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 1],
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 2],
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 3],
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 4],
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 5],
							state->uplink_buffer.usb_ni_data_frame.pdu[BROADCAST_MSG_CODE_OFFSET + 6]);
				}
				break;
			case LonNiFlushCompleteCmd:
				OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError, "CheckUplinkCompleted: NI Flush Complete message received");
				break;
			default:
				// Other NI command; no special processing at this time
				break;
			}
            // Trashed uplink data can result in 0xFF error value in the len field
            if (((unsigned int)state->uplink_msg_length + 2) > (sizeof(LonDataFrame) + 2)) {
                OsalPrintLog(INFO_LOG, LonStatusNoError,
						"CheckUplinkCompleted: Uplink message had invalid length: %d",
						state->uplink_msg_length);
            } else {
                // Queue it
                uplink_status = ProcessUplinkMessage();
            }
		}
		ResetUplinkState();
		*completed = true;
	}
	return !LON_SUCCESS(uplink_status) ? uplink_status : status;
}

/*
 * Queues an uplink (read) message from the LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called by CheckUplinkCompleted() which is called via ReadLonUsbMsg()
 *   parsing.  If the have_uid flag is set and the message is a read unique ID
 *   memory response, the unique ID is copied to the LON interface state.
 *   If the wait_for_uid flag is set and the message is not a read unique ID
 *   memory response, the message is dropped. Otherwise the message is queued 
 *   for upstream processing.
 */
LonStatusCode ProcessUplinkMessage()
{
	LonStatusCode status = LonStatusNoError;
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return LonStatusInvalidInterfaceId;
	}
    size_t copy_length = TotalMessageLength(&state->uplink_buffer.usb_ni_data_frame) + 1;
// the buf is unused, even in lon_usb_link.c	
	//uint8_t *buf = (uint8_t *)&state->uplink_buffer.usb_ni_data_frame;
    if (state->wait_for_uid) {
		// Test for a read memory response to the unique ID read request
		// by testing for a message length greater than or equal to 23 bytes
		// (including NI command but not length) and then checking for
		// LonNiResponseCmd at byte 1 and 0x2D read memory response command
		// at byte 16
		if (copy_length >= 23 
				&& state->uplink_buffer.usb_ni_data_frame.ni_command == LonNiResponseCmd 
				&& state->uplink_buffer.usb_ni_data_frame.pdu[14] == (IzotNmReadMemory & (IZOT_NM_OPCODE_MASK | IZOT_NM_RESPONSE_SUCCESS))) { 
			int i;
			for (i=0; i < 6; i++) {
				state->uid[i] = state->uplink_buffer.usb_ni_data_frame.pdu[i+15]; // TODO: Verify the correct offset for the UID bytes in the message
			}
			state->wait_for_uid = false;
            state->have_uid = true;
			OsalPrintLog(INFO_LOG, LonStatusNoError, 
					"QueueUplinkMessage:LON interface %d unique ID %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
                	state->uid[0], state->uid[1], state->uid[2],
                	state->uid[3], state->uid[4], state->uid[5]);
		}
    } else if (state->wait_for_status) {
		// Test for a request response to a MIP status request by testing
		// for a message length greater than or equal to 6 bytes (including
		// NI command but not length) and then checking for the 0xE0 response
		// code at byte 1 and 0x05 PDU length at byte 1
		// TODO: Replace buf with direct access to uplink_buffer fields and verify the correct offsets for the response code and length fields in the message
		if (copy_length >= 6 && state->uplink_buffer.usb_ni_data_frame.ni_command == 0xE0 && state->uplink_buffer.usb_ni_data_frame.short_pdu_length == 5) { 
			IzotLonInterfaceStatusResponse *response = (IzotLonInterfaceStatusResponse *)&state->uplink_buffer.usb_ni_data_frame.pdu[0];
			state->wait_for_status = false;
            state->have_status = true;
			state->lon_stats.l2_l5_mode = state->current_iface_mode = response->layer_mode;
			state->lon_stats.ni_model_id = response->model_id;
			state->lon_stats.ni_fw_version = response->fw_version;
			state->lon_stats.ni_status = response->status;
			OsalPrintLog(INFO_LOG, LonStatusNoError, 
					"ProcessUplinkMessage:LON, model %d, firmware version %d.%d, layer %d mode, status %d",
					response->model_id,
					response->fw_version / 10, response->fw_version % 10,
					response->layer_mode ? 2 : 5, response->status);
		}
	} else {
		Increment32(state->lon_stats.uplink.packets_received);
		Add32(state->lon_stats.uplink.bytes_received, copy_length);
        status = QueueUplinkBuffer();
		OsalPrintLog(DETAIL_TRACE_LOG, status, "ProcessUplinkMessage: Queued uplink message with %d bytes", copy_length);
		OsalPrintMessage(DETAIL_TRACE_LOG, "ProcessUplinkMessage: ", (const uint8_t *)&state->uplink_buffer.usb_ni_data_frame, copy_length);
    }
	return status;
}

/*
 * Returns the total length of a LON USB network interface message,
 * including length fields.
 * Parameters:
 *   msg: Pointer to LonDataFrame or LonExtDataFrame structure
 * Returns:
 *   Total message length including length fields; 0 if msg is NULL
 * Notes:
 *   Called by ProcessUplinkMessage() to determine the total length of
 *   an uplink message for statistics and processing.
 */
size_t TotalMessageLength(const LonNiFrame *msg)
{
	if (!msg) {
		return 0;
	} else if (msg->short_pdu_length < EXT_LENGTH) {
		return msg->short_pdu_length + 1;  // +1 for length byte
	} else {
		const LonNiExtendedFrame *ext_msg = (const LonNiExtendedFrame *)msg;
		return ntoh16(ext_msg->ext_length_be) + 1 + 2;	
				// +1 for length byte, +2 for extended length bytes
	}
}

/*
 * Feeds received bytes into the RX ring buffer for a LON USB interface.
 * Parameters:
 *   data: pointer to buffer of received bytes
 *   len: number of bytes in data buffer
 * Returns:
 *   Number of bytes successfully fed into the RX ring buffer
 * Notes:
 *   Called by an optional asynchronous OS-specific LON USB interface
 *   receive handler when data is received from the LON USB network 
 *   interface.  For example, this function can be called by a 
 *   platform-specific interrupt handler for the USB interface.  
 *   The data is copied into the RX ring buffer for later processing
 *   by ReadLonUsbMsg().
 */
size_t LonUsbFeedRx(const uint8_t *data, size_t len) 
{
	if (!data || len == 0) {
		return 0;
	}
    LonUsbLinkState *state = &iface_state;
	if (state->shutdown) {
		return 0;
	}
	// Feed thread may run concurrently: lock around ring write and stats
	// Protect emptiness check; head pointer may be updated concurrently
	OsalLockMutex(&state->queue_lock);
	size_t written = RingBufferWrite(&state->lon_usb_uplink_ring_buffer, data, len);
	size_t occ = RingBufferSize(&state->lon_usb_uplink_ring_buffer);
	state->lon_stats.usb_rx.bytes_fed += written;
	if (occ > state->lon_stats.usb_rx.max_occupancy) {
		state->lon_stats.usb_rx.max_occupancy = occ;
	}
	if (written < len) {
		state->lon_stats.usb_rx.bytes_dropped += (len - written);
	}
	OsalUnlockMutex(&state->queue_lock);
	return written;
}

/*
 * Closes a LON USB network interface.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
// This never close on MT823
/*
LonStatusCode CloseLonUsbLink()
{
    LonStatusCode status = LonStatusCloseFailed;
    LonUsbLinkState *state = &iface_state;
//	if (state == NULL) {
//		status = LonStatusCloseFailed;
//		OsalPrintLog(INFO_LOG, status, "Error closing interface %d", iface_index);
//	} else {
//		HalCloseUsb(state->usb_fd);
		ResetUplinkState();
//		state->shutdown = true;
//		state->assigned = false;
//		status = LonStatusNoError;
	OsalPrintDebug(LonStatusNoError, "Close interface");
//	}
//	return status;
	return LonStatusNoError;
}
*/
/*****************************************************************
 * Section: State Array Management Function Definitions
 *****************************************************************/
/* 
 * Initializes all iface_state entries to default values.
 * Parameters:
 *   None
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 * Notes:
 *   Called once at first OpenLonUsbLink() to initialize all states.
 *   Calls ResetUplinkState() for each state.
 */
LonStatusCode InitIfaceStates(void) 
{
    LonStatusCode status = LonStatusNoError;
		LonUsbLinkState *state = &iface_state;
		OsalInitMutex(&state->queue_lock);
		OsalInitMutex(&state->state_lock);
//		state->iface_index = iface_index;
		state->assigned = true; //false;					// Set to true in AssignIfaceState()
		state->shutdown = false;//true; 					// Set to false in OpenLonUsbLink()
		state->wait_for_reset = false;				// Set to true in OpenLonUsbLink() while waiting for an uplink reset command
		state->have_reset = true;//false;					// Set to true when a reset command is received in CheckUplinkCompleted()
		state->wait_for_null = false;				// Set to true in OpenLonUsbLink() while waiting for an uplink null frame
		state->have_null = false;					// Set to true when a null frame is received in CheckUplinkCompleted()
		state->wait_for_uid = false;				// Set to true in RequestNiUid()
		state->have_uid = false;					// Set to true when UID is received in QueueUplinkMessage()
		state->wait_for_status = false;				// Set to true in OpenLonUsbLink() while waiting for an uplink status response
		state->have_status = false;					// Set to true when a status response is received in CheckUplinkCompleted()
		state->ready = true;//false;						// Set to true when interface is ready for use
		//state->usb_params = default_lon_usb_config;
		memset(&state->uid, 0, sizeof(state->uid));
		state->configured_iface_mode = LON_IFACE_MODE_UNKNOWN;	// Default to unknown mode for startup
		state->current_iface_mode = LON_IFACE_MODE_UNKNOWN;		// Default to unknown mode for startup
		state->start_time = state->last_timeout = 0;
		// Initialize downlink state
		state->downlink_state = DOWNLINK_IDLE; //DOWNLINK_INVALID;
		state->downlink_reject_timer = 0;
		state->downlink_ack_seq_number = NO_ACK_REQUIRED;
		state->downlink_seq_number = 0;
		memset(&state->downlink_buffer_staging, 0, sizeof(state->downlink_buffer_staging));
		memset(&state->downlink_buffer, 0, sizeof(state->downlink_buffer));
		// Initialize uplink state
		state->uplink_state = UPLINK_IDLE;
		state->uplink_ack_timer = 0;
		state->uplink_ack_timeout = STARTUP_ACK_WAIT_TIME;	// Use longer timeout to allow LON interface reset
		state->uplink_ack_timeouts = 0;
		state->uplink_ack_timeout_phase = 0;
		state->uplink_duplicate = false;
		state->uplink_frame_error = false;
		state->uplink_msg_index = 0;
		state->uplink_msg_length = 0;
		state->uplink_msg_timer = 0;
		state->uid_wait_timer = 0;
		state->status_wait_timer = 0;
		state->uplink_seq_number = ~FRAME_CODE_SEQ_NUM_MASK; // Force mismatch on first frame
		state->uplink_tail_escaped = false;
		// Allocate and initialize queues and buffers
	    // Allocate and initialize LON USB downlink queues
		if (((status = QueueInit(&state->lon_usb_downlink_normal_queue, "link layer downlink", sizeof(LonUsbQueueBuffer), MAX_LON_DOWNLINK_BUFFERS)) != LonStatusNoError)
				|| ((status = QueueInit(&state->lon_usb_downlink_priority_queue, "link layer priority downlink", sizeof(LonUsbQueueBuffer), MAX_LON_DOWNLINK_BUFFERS)) != LonStatusNoError)) {
			OsalPrintLog(ERROR_LOG, LonStatusStackInitializationFailure, "InitIfaceStates: Failed to initialize link layer downlink queue");
			return status;
		}
	    // Allocate and initialize LON USB uplink queues
		if (((status = QueueInit(&state->lon_usb_uplink_normal_queue, "link layer uplink", sizeof(LonUsbQueueBuffer), MAX_LON_UPLINK_BUFFERS)) != LonStatusNoError)
				|| ((status = QueueInit(&state->lon_usb_uplink_priority_queue, "link layer priority uplink", sizeof(LonUsbQueueBuffer), MAX_LON_UPLINK_BUFFERS)) != LonStatusNoError)) {
			OsalPrintLog(ERROR_LOG, LonStatusStackInitializationFailure, "InitIfaceStates: Failed to initialize link layer uplink queue");
			return status;
		}
		// Initialize the LON USB uplink ring buffer (capacity set to full internal max)
		if (!LON_SUCCESS(status = RingBufferInit(&state->lon_usb_uplink_ring_buffer, RING_BUFFER_MAX_CAPACITY))) {
			OsalPrintLog(ERROR_LOG, status, "InitIfaceStates: Failed to initialize LON USB uplink ring buffer");
			return status;
		}
		RingBufferClear(&state->lon_usb_uplink_ring_buffer);
		state->lon_stats.usb_rx.capacity = RING_BUFFER_MAX_CAPACITY;
		state->lon_stats.usb_rx.max_occupancy = RingBufferSize(&state->lon_usb_uplink_ring_buffer);
		// Initialize LON USB link statistics
		memset(&state->lon_stats, 0, sizeof(state->lon_stats));
		state->lon_stats.l2_l5_mode = LON_IFACE_MODE_UNKNOWN;
		state->lon_stats.ni_status = LON_NI_STATUS_UNKNOWN;
		state->lon_stats.size = sizeof(state->lon_stats);
	OsalPrintLog(INFO_LOG, status, "Initialized LON interface states with %d byte uplink ring buffer and %d entry uplink and downlink queues",
			RING_BUFFER_MAX_CAPACITY, MAX_LON_UPLINK_BUFFERS);
	return status;
}

/*****************************************************************
 * Section: Downlink Queue Management Function Definitions
 *****************************************************************/
/*
 * Downlink queue helpers (singly linked with head pointer)
 * ----------------------------------------------------------
 * Head:   LonUsbLinkState.downlink_queue points to first node, or NULL if empty.
 * Nodes:  Each node embeds a LonUsbQueueBuffer in entry->data.
 * Ops:
 *  - QueueDownlinkBuffer: copy src into a new node; if head NULL set head, else append.
 *  - ReadDownlinkBuffer: pop head (advance pointer), copy into dst, free node.
 *  - GetDownlinkBufferCount: O(n) walk via next pointers.
 */

/*
 * Removes and returns the front buffer from a downlink queue.
 * Parameters:
 *   dst: pointer to LonUsbQueueBuffer to receive the data
 * Returns:
 *   LonUsbQueueBuffer in dst if available; NULL if no buffer is available
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode ReadDownlinkBuffer( LonUsbQueueBuffer *dst) {
    if (!dst) {
        return LonStatusInvalidParameter;
    }
    LonUsbLinkState *state = &iface_state;
	LonStatusCode status = LonStatusNoError;
	OsalLockMutex(&state->queue_lock);
	// Get source queue: priority if not empty, else normal
	Queue *source_queue = &state->lon_usb_downlink_priority_queue;
	if (QueueEmpty(source_queue)) {
		source_queue = &state->lon_usb_downlink_normal_queue;
	}
	// Get head of source queue, if any
	LonUsbQueueBuffer* buffer = QueuePeek(source_queue);
    if (!buffer) {
		status = LonStatusNoBufferAvailable;
    } else {
    	// Copy the queue entry to dst
    	memcpy(dst, buffer, buffer->buf_size);
    	// Advance source queue head past the first entry
		QueueDropHead(source_queue);
		OsalPrintLog(DETAIL_TRACE_LOG, status, 
				"ReadDownlinkBuffer: Read %zu byte entry from %s queue, new entry count %d, head index %d (%p), tail index %d (%p)",
				dst->buf_size, source_queue->queueName, source_queue->queueEntries,
				source_queue->headIndex, source_queue->head, source_queue->tailIndex, source_queue->tail);
		OsalPrintMessage(DETAIL_TRACE_LOG,"ReadDownlinkBuffer: ", (const uint8_t *)dst, dst->buf_size);
	}
	OsalUnlockMutex(&state->queue_lock);
    return status;
}

/*
 * Flushes all downlink buffers from the specified queues.
 * Parameters:
 *   priority: MessagePriorityLevel specifying which queues to flush
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode FlushDownlinkQueue(MessagePriorityLevel priority)
{
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	if ((priority == HIGH_PRIORITY) || (priority == ALL_PRIORITIES)) {
		QueueFlush(&state->lon_usb_downlink_priority_queue);
	}
	if ((priority == NORMAL_PRIORITY) || (priority == ALL_PRIORITIES)) {
		QueueFlush(&state->lon_usb_downlink_normal_queue);
	}
	OsalUnlockMutex(&state->queue_lock);
	return LonStatusNoError;
}

/*
 * Appends a copy of src to the end of the downlink queue.
 * Parameters:
 *   src: pointer to LonUsbQueueBuffer containing the data to append
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode QueueDownlinkBuffer(LonUsbQueueBuffer *src) {
    LonUsbLinkState *state = &iface_state;
	if (!src) {
		OsalPrintLog(ERROR_LOG, LonStatusInvalidParameter, "QueueDownlinkBuffer: NULL src");
		return LonStatusInvalidParameter;
	}
	Queue *queue;
	LonUsbQueueBuffer *tail;
	LonStatusCode status = LonStatusNoError;
	OsalLockMutex(&state->queue_lock);
	if (src->priority == HIGH_PRIORITY) {
		queue = &state->lon_usb_downlink_priority_queue;
	} else {
		queue = &state->lon_usb_downlink_normal_queue;
	}
	tail = QueueTail(queue);
	if (!tail) {
		// Invalid queue
		status = LonStatusInvalidParameter;
		OsalPrintLog(ERROR_LOG, status, "QueueDownlinkBuffer: Invalid downlink queue");
	} else {
		// Append the new entry to the queue
		memcpy(tail, src, src->buf_size);
		QueueWrite(queue);
	    OsalPrintLog(DETAIL_TRACE_LOG, LonStatusNoError, 
            	"QueueDownlinkBuffer: Wrote %zu byte entry to %s queue, new entry count %d, head index %d (%p), tail index %d (%p)",
            	src->buf_size, queue->queueName, queue->queueEntries,
				queue->headIndex, queue->head, queue->tailIndex, queue->tail);
	    OsalPrintMessage(DETAIL_TRACE_LOG, "QueueDownlinkBuffer: ", (const uint8_t *)tail, src->buf_size);
	}
	OsalUnlockMutex(&state->queue_lock);
	return status;
}

/*
 * Gets the number of downlink buffers currently queued.
 * Parameters:
 * Returns:
 *   Number of buffers in the downlink queues; 0 if interface is invalid
 */
static size_t last_downlink_buffer_count = -1;

size_t GetDownlinkBufferCount() {
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	size_t cnt = QueueEntries(&state->lon_usb_downlink_priority_queue) + 
				 QueueEntries(&state->lon_usb_downlink_normal_queue);
	OsalUnlockMutex(&state->queue_lock);
	if (cnt != last_downlink_buffer_count) {
		last_downlink_buffer_count = cnt;
		OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError,
				"GetDownlinkBufferCount: downlink buffer count: %zu",
				cnt);
	}
	return cnt;
}

/*
 * Returns TRUE if the downlink queues are empty for the given interface.
 * Parameters:
 * Returns:
 *   TRUE if the downlink queues are empty or the interface is invalid
 */
bool IsDownlinkQueueEmpty() {
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	bool empty = QueueEmpty(&state->lon_usb_downlink_priority_queue)
			&& QueueEmpty(&state->lon_usb_downlink_normal_queue);
	OsalUnlockMutex(&state->queue_lock);
	return empty;
}


/*****************************************************************
 * Section: Uplink Queue Management Function Definitions
 *****************************************************************/
/*
 * Uplink queue helpers (singly linked with head pointer)
 * ------------------------------------------------------
 * Head:   LonUsbLinkState.uplink_queue points to first node, or NULL if empty.
 * Nodes:  Each node embeds a LonUsbQueueBuffer in entry->data.
 * Ops:
 *  - QueueUplinkBuffer: copy src into a new node; if head NULL set head, else append.
 *  - ReadUplinkBuffer: pop head (advance pointer), copy into dst, free node.
 *  - GetUplinkBufferCount: O(n) walk via next pointers.
 */

/*
 * Removes and returns the front buffer from an uplink queue.
 * Parameters:
 *   dst: pointer to LonUsbQueueBuffer to receive the data
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode ReadUplinkBuffer(LonUsbQueueBuffer *dst) 
{
    if (!dst) {
        return LonStatusInvalidParameter;
    }
    LonUsbLinkState *state = &iface_state;
	LonStatusCode status = LonStatusNoError;
	OsalLockMutex(&state->queue_lock);
	// Get source queue: priority if not empty, else normal
	Queue *source_queue = &state->lon_usb_uplink_priority_queue;
	if (QueueEmpty(source_queue)) {
		source_queue = &state->lon_usb_uplink_normal_queue;
	}
	// Get head of source queue, if any
	LonUsbQueueBuffer* buffer = QueuePeek(source_queue);
    if (!buffer) {
		status = LonStatusNoBufferAvailable;
    } else {
		// Copy the queue entry to dst
		memcpy(dst, buffer, sizeof(LonUsbQueueBuffer));
		// Advance source queue head past the first entry
		QueueDropHead(source_queue);
	}
	OsalUnlockMutex(&state->queue_lock);
    return status;
}

/*
 * Flushes all uplink buffers from the specified queue.
 * Parameters:
 *   priority: MessagePriorityLevel specifying which queues to flush
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode FlushUplinkQueue(MessagePriorityLevel priority)
{
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	if ((priority == HIGH_PRIORITY) || (priority == ALL_PRIORITIES)) {
		QueueFlush(&state->lon_usb_uplink_priority_queue);
	}
	if ((priority == NORMAL_PRIORITY) || (priority == ALL_PRIORITIES)) {
		QueueFlush(&state->lon_usb_uplink_normal_queue);
	}
	OsalUnlockMutex(&state->queue_lock);
	return LonStatusNoError;
}

/*
 * Appends a copy of src to the end of the uplink queue.
 * Parameters:
 * Returns:
 *   LonStatusNoError on success; LonStatusCode error code if unsuccessful
 */
LonStatusCode QueueUplinkBuffer() {
    LonUsbLinkState *state = &iface_state;
	Queue *queue;
	LonUsbQueueBuffer *tail;
	LonStatusCode status = LonStatusNoError;
	OsalLockMutex(&state->queue_lock);
	if (state->uplink_buffer.priority == HIGH_PRIORITY) {
		queue = &state->lon_usb_uplink_priority_queue;
	} else {
		queue = &state->lon_usb_uplink_normal_queue;
	}
	tail = QueueTail(queue);
	if (!tail) {
		// Invalid queue
		status = LonStatusInvalidParameter;
		OsalPrintLog(ERROR_LOG, status, "QueueUplinkBuffer: Invalid uplink queue");
	} else {
		// Append the new entry to the queue
		memcpy(tail, &state->uplink_buffer, sizeof(LonUsbQueueBuffer));
		QueueWrite(queue);
	}
	OsalUnlockMutex(&state->queue_lock);
    return status;
}

/*
 * Gets the number of uplink buffers currently queued.
 * Parameters:
 * Returns:
 *   Number of buffers in the uplink queues; 0 if interface is invalid
 */
size_t GetUplinkBufferCount() {
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	size_t cnt = QueueEntries(&state->lon_usb_uplink_priority_queue) + 
				 QueueEntries(&state->lon_usb_uplink_normal_queue);
	OsalUnlockMutex(&state->queue_lock);
	return cnt;
}

/*
 * Returns TRUE if the uplink queues are empty for the given interface.
 * Parameters:
 *   iface_index: interface index returned by OpenLonUsbLink()
 * Returns:
 *   TRUE if the uplink queues are empty or the interface is invalid
 */
bool IsUplinkQueueEmpty() {
    LonUsbLinkState *state = &iface_state;
	OsalLockMutex(&state->queue_lock);
	bool empty = QueueEmpty(&state->lon_usb_uplink_priority_queue)
			&& QueueEmpty(&state->lon_usb_uplink_normal_queue);
	OsalUnlockMutex(&state->queue_lock);
	return empty;
}

/*****************************************************************
 * Section: Debugging Support Function Definitions
 *****************************************************************/

/*
 * Prints the contents of an uplink code packet for debugging.
 * Parameters:
 *   prefix: string prefix to print before each line
 * Returns:
 *   None
 * Notes:
 *   Only prints if the PACKET_TRACE_LOG log category is enabled.
 */
void PrintUplinkCodePacket(char *prefix)
{
	if ((OsalGetLogCategories() & PACKET_TRACE_LOG) == 0) {
		return;    // Fast path: packet tracing disabled
	}
    LonUsbLinkState *state = &iface_state;
	LonUsbFrameCommand frame_cmd = IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_CMD);
	unsigned int sequence_num = IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_SEQ);
	bool ack_flag = IZOT_GET_ATTRIBUTE(state->uplink_buffer.frame_header, IZOT_U50_FRAME_CODE_ACK) != 0;
	uint8_t parameter = state->uplink_buffer.frame_header.parameter;
	uint8_t checksum = state->uplink_buffer.frame_header.checksum;
	OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError, 
			"%sReceived uplink code packet with 0x%02X frame command, %u sequence, %s ACK flag, 0x%02X parameter",
			prefix, frame_cmd, sequence_num, (ack_flag) ? "Set" : "Clear", parameter);
	OsalPrintLog(PACKET_TRACE_LOG, LonStatusNoError,
			"%sUplink state %s, downlink state %s, checksum 0x%02X", 
			prefix, uplink_state_names[state->uplink_state], downlink_state_names[state->downlink_state], checksum);
}

/*****************************************************************
 * Section: Implementation-specific Function Definitions
 *****************************************************************/
/*
 * Responds to a LON Wink command.
 * Parameters:
 * Returns:
 *   None
 * Notes:
 *   Called by CheckUplinkCompleted() when a LON Wink command is received.
 *   The implementation can be customized to perform a physical action to
 *   identify the LON device, for example by flashing an LED.  For now,
 *   this function just logs the event.
 */
void IdentifyLonNi()
{
	// Alain Set flash led on MT823 using mode wink !!  
	OsalPrintLog(INFO_LOG, LonStatusNoError, "IdentifyLonNi: LON Wink command received");
}

#endif // LINK_IS(SINGLE_USB)