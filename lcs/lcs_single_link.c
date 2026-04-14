/*
 * lcs_single_link.c
 *
 * Copyright (c) 2022-2026 EnOcean
 * SPDX-License-Identifier: MIT
 * See LICENSE file for details.
 * 
 * Title:   LON Stack Data Link Layer for LON USB and MIP Data Links
 * Purpose: Implements the LON data link layer (Layer 2) of the 
 *          ISO/IEC 14908-1 LON protocol stack.
 * Notes:   The functions in this file support LON data links using a
 *          LON USB network interface such as the U10 or U60.
 */

#include "lcs/lcs_link.h"

#if !LINK_IS(ETHERNET) && !LINK_IS(WIFI)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcs/lcs_timer.h"
#include "lcs/lcs_node.h"
#include "lcs/lcs_queue.h"
#include "lcs/lcs_netmgmt.h"
#include "lon_usb/lon_usb_single_link.h"
//#include "lon_usb/lon_usb_single_link.h"

// Unique ID fetch interval in milliseconds
#define UNIQUE_ID_FETCH_INTERVAL 500
// LON PL transceiver parameters fetch interval in milliseconds
#define XCVR_PARAM_FETCH_INTERVAL 10000

// 1-byte header definition of an LPDU in the queue
typedef struct {
	BITS3(priority,		1,
		  altPath,		1,
		  deltaBL,		6)
} LPDUHeader;

// Number of LON network interfaces to be supported
#define NUM_LON_NI 1

#define LNM_TAG 0x0F	// Tag reserved for local network management

/*****************************************************************
 * Section: Function Declarations
 *****************************************************************/
void LKFetchXcvrPl(int niIndex); // Fetch PL transceiver params for a LON NI

/*****************************************************************
 * Section: Function Definitions
 *****************************************************************/

/*
 * Initializes the LON Stack link layer including queues used by the link layer.
 * Parameters:
 *   None
 * Returns:
 *   LonStatusNoError if successful, LonStatusCode error code otherwise
 * Notes:
 *   Sets gp->resetOk to false if unable to reset properly.
 *   For a MIP LON link, the input queue is also used by the physical
 *   layer and is not a regular queue.  Each item in the queue has the
 *   following form:
 *     <flag> <LPDUSize> <LPDU>
 *   where
 *     <flag> is 1 byte
 *     <LPDUSize> is 2 bytes
 *     <LPDU> is of the form LPDU_HEADER RESTOFLPDU CRC
 *     LPDU_HEADER is 1 byte (LPDU does not include the syncbits)
 *     CRC uses 2 bytes.
 *   Total # bytes in addition to NPDU is thus 6 bytes.
 */
LonStatusCode LinkLayerReset(void)
{
    IzotUbits16 queueItemSize;
    IzotByte   *p;  // Used to initialize lkInQ
    IzotUbits16 i;
    LonStatusCode status = LonStatusNoError;

    // Allocate and initialize the input queue
    if (!LON_SUCCESS(status = DecodeBufferSize(LK_IN_BUF_SIZE, &gp->lkInBufSize))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to decode input link buffer size");
        gp->resetOk = FALSE;
        return status;
    }
    gp->lkInBufSize += 6;
    if (!LON_SUCCESS(status = DecodeBufferCnt(LK_IN_Q_CNT, &gp->lkInQCnt))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to decode input link queue count");
        gp->resetOk = FALSE;
        return status;
    }
    gp->lkInQ = OsalAllocateMemory((size_t)(gp->lkInBufSize * gp->lkInQCnt));
    if (gp->lkInQ == NULL) {
        OsalPrintLog(ERROR_LOG, LonStatusNoMemoryAvailable, "LinkLayerReset: Unable to initialize the input queue");
        gp->resetOk = FALSE;
        return status;
    }
    // Initialize the flag in each item of the queue to 0
    p = gp->lkInQ;
    for (i=0; i < gp->lkInQCnt; i++) {
        *p = 0;
        p = (uint8_t *)((char *)p + gp->lkInBufSize);
    }
    gp->lkInQHeadPtr = gp->lkInQTailPtr = gp->lkInQ;

    // Allocate and initialize the output queue
    if (!LON_SUCCESS(status = DecodeBufferSize(LK_OUT_BUF_SIZE, &gp->lkOutBufSize))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to decode output link buffer size");
        gp->resetOk = FALSE;
        return status;
    }
    if (!LON_SUCCESS(status = DecodeBufferCnt(LK_OUT_Q_CNT, &gp->lkOutQCnt))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to decode output link queue count");
        gp->resetOk = FALSE;
        return status;
    }
    queueItemSize    = gp->lkOutBufSize + sizeof(LKSendParam);
    status = QueueInit(&gp->lkOutQ, "link layer output", queueItemSize, gp->lkOutQCnt);
    if (status != LonStatusNoError) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to initialize the output queue");
        gp->resetOk = FALSE;
        return status;
    }

    // Allocate and initialize the priority output queue
    gp->lkOutPriBufSize = gp->lkOutBufSize;
    if (!LON_SUCCESS(status = DecodeBufferCnt(LK_OUT_PRI_Q_CNT, &gp->lkOutPriQCnt))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to decode priority output link queue count");
        gp->resetOk = FALSE;
        return status;
    }
    queueItemSize = gp->lkOutPriBufSize + sizeof(LKSendParam);

    if (!LON_SUCCESS(status = QueueInit(&gp->lkOutPriQ, "link layer priority output", queueItemSize, gp->lkOutPriQCnt))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to initialize the priority output queue");
        gp->resetOk = FALSE;
        return status;
    }
    OsalPrintLog(INFO_LOG, LonStatusNoError, "LinkLayerReset: Link layer queues initialized");

    if (!LON_SUCCESS(status = OpenLonUsbLink(LON_IFACE_MODE_LAYER2))) {
        OsalPrintLog(ERROR_LOG, status, "LinkLayerReset: Unable to open LON link");
        gp->resetOk = FALSE;
        return status;
    }

        OsalPrintLog(INFO_LOG, LonStatusNoError, "LinkLayerReset: LON link  opened");

    gp->resetOk = TRUE;
    return status;
}

/*
 * Receives an NPDU from the network layer, adds link layer contents to the
 * NPDU to create an LPDU, and send the LPDU to the LON network interface.
 * Parameters:
 *   None
 * Returns:
 *   None
 * Notes:
 *   The LON network interface is typically a U10 or U60 LON USB interface.
 *   Extra bytes are allocated in the buffers to accomodate layer-specific
 *   additions to the buffer contents.
 */
void LinkLayerUsbSend(void)
{
    LKSendParam     *lkSendParamPtr;
    Queue           *lkSendQueuePtr;
    uint8_t         *npduPtr;
    LPDUHeader      *lpduHeaderPtr;
    bool             priority;
	LonDataFrame     sicb;

	static OsalTickCount last_report_time = 0;
	if (OsalGetTickCount() - last_report_time > 1000) {
		last_report_time = OsalGetTickCount();
        if (!QueueEmpty(&gp->lkOutPriQ) || !QueueEmpty(&gp->lkOutQ)) {
            OsalPrintLog(INFO_LOG, LonStatusNoError,
                "LinkLayerUsbSend: %zu output priority queue entries,  %zu output normal queue entries",
                QueueEntries(&gp->lkOutPriQ), QueueEntries(&gp->lkOutQ));
        }
	}

    // Make variables point to the new queue
    if (!QueueEmpty(&gp->lkOutPriQ)) {
        priority       = TRUE;
        lkSendQueuePtr = &gp->lkOutPriQ;
    } else if (!QueueEmpty(&gp->lkOutQ)) {
        priority       = FALSE;
        lkSendQueuePtr = &gp->lkOutQ;
    } else {
        return; // Nothing to send
    }

	lkSendParamPtr = QueuePeek(lkSendQueuePtr);
    // TODO: this assumes that the NPDU is located in the buffer immediately
    // following the first byte of the LKSendParam structure in the queue item;
    // consider changing to have the first entry in the buffer be the size of
    // the NPDU and then the NPDU itself
	npduPtr = (uint8_t *) (lkSendParamPtr + 1);

	sicb.ni_command = LonNiNetworkMgmtCmd;
	sicb.short_pdu_length = lkSendParamPtr->pduSize+1;
	lpduHeaderPtr = (LPDUHeader *)sicb.pdu;
	lpduHeaderPtr->priority = priority;
	lpduHeaderPtr->altPath = lkSendParamPtr->altPath;
	lpduHeaderPtr->deltaBL = lkSendParamPtr->deltaBL;
	// Copy the NPDU
	if (lkSendParamPtr->pduSize <= sizeof(sicb.pdu)) {
		memcpy(&sicb.pdu[1], npduPtr, lkSendParamPtr->pduSize);
	}
	
    // Send the LPDU to LON network interface downlink queues
    WriteLonUsbMsg(&sicb);

    // Remove the LPDU from the link layer output queue
	QueueDropHead(lkSendQueuePtr);
    return;
}

/*
 * Receives an LPDU from a LON interface, extracts the NPDU, and transfers
 * the NPDU to the network layer.
 * Parameters:
 *   None
 * Returns:
 *   None
 * Notes:
 *   The LON interface is typically a U10 or U60 LON USB interface.
 *   Each item of the queue gp->lkInQ has the following form:
 *     flag pduSize LPDU
 *       flag is 1 byte long
 *       pduSize is 2 bytes long
 *       LPDU has header followed by the rest of the LPDU and then CRC
 *         LPDU header is 1 byte long
 *         CRC is 2 bytes
 *   If a packet is in lkInQ then it will fit into nwInQ.
 */
void LinkLayerUsbReceive(void)
{
    NWReceiveParam *nwReceiveParamPtr;
    IzotByte       *npduPtr;
    LPDUHeader     *lpduHeaderPtr;
    IzotByte       *tempPtr;
    IzotUbits16     lpduSize;
	LonDataFrame	sicb;
	
    if (ReadLonUsbMsg(&sicb) != LonStatusNoError)
    {
	  	// No packets to process
	  	return;
    }

    lpduSize      =	sicb.short_pdu_length;
    lpduHeaderPtr = (LPDUHeader*)&sicb.pdu[0];

	// Throw away layer 2 mode 2 packets that are smaller than 8 bytes long;
    // layer 2 mode 2 network interfaces report CRC errors as a packet with
    // a short length
	if (((sicb.ni_command == LonNiIncomingL2Mode2Cmd) && (lpduSize < 8)) ||
		((sicb.ni_command&0xF0) == (LonNiError&0xF0))) {
	  	INCR_STATS(LcsTxError);
		return;
	} else if (sicb.ni_command != LonNiIncomingL2Mode2Cmd) {
        // Not a layer 2 mode 2 packet--ignore the packet
	  	return;
	}

    // CRC check was performed by the LON network interface;
    // increment the valid packet received count
    INCR_STATS(LcsL2Rx);

    // Check if the packet is for us
    if (sicb.ni_command != LonNiIncomingL2Mode2Cmd || sicb.pdu[0] != LonNiLocalNetMgmtCmd) {
        INCR_STATS(LcsMissed);
        return;
    }

    // CRC check was performed by the LON interface;
    // increment the valid packet received count
    INCR_STATS(LcsL2Rx);
    if (QueueFull(&gp->nwInQ)) {
        // Network layer input queue is full--lose this packet
        INCR_STATS(LcsMissed);
    } else {
        // Network layer input queue entry available--receive the packet
        nwReceiveParamPtr = QueueTail(&gp->nwInQ);
        npduPtr           = (uint8_t *)(nwReceiveParamPtr + 1);

        nwReceiveParamPtr->priority = lpduHeaderPtr->priority;
        nwReceiveParamPtr->altPath  = lpduHeaderPtr->altPath;
        tempPtr = (uint8_t *)((char *)lpduHeaderPtr + 1);
        nwReceiveParamPtr->pduSize  = lpduSize - 3;
        // TODO: The following line has been commented out because
        // there is no xcvrParams field in NWReceiveParam and
        // transceiver parameters are only fetched for a PL
        // network interface
 		// nwReceiveParamPtr->xcvrParams = lonLinkXcvrPlParam;
 
        // Copy the NPDU; if it was in link layer's queue, then the size
        // should be sufficient in the network layer's queue as they differ
        // by 3; play safe by checking the size first
        if (nwReceiveParamPtr->pduSize <= gp->nwInBufSize) {
            memcpy(npduPtr, tempPtr, nwReceiveParamPtr->pduSize);
        } else {
            OsalPrintLog(ERROR_LOG, LonStatusNoMemoryAvailable, "LinkLayerUsbReceive: NPDU size is too large");
        }
        QueueWrite(&gp->nwInQ);
    }
    *(gp->lkInQHeadPtr) = 0;
    gp->lkInQHeadPtr = gp->lkInQHeadPtr + gp->lkInBufSize;
    if (gp->lkInQHeadPtr ==
            (gp->lkInQ + gp->lkInBufSize * gp->lkInQCnt)) {
        gp->lkInQHeadPtr = gp->lkInQ; // Wrap around
    }
    return;
}

/*
 * Reads the Unique ID (Neuron ID or MAC ID) from a LON USB network interface.
 * Parameters:
 *   uidBuf: Buffer to receive the Unique ID
 * Returns:
 *   LonStatusNoError if successful, LonStatusCode error code otherwise
 * Notes:
 *   This function is called by IzotGetUniqueId() to get the Unique ID from a LON
 *   USB network interface.
 */
LonStatusCode LinkLayerReadUsbUid(IzotUniqueId *uidBuf)
{
    //if (!lonNi.linkOpened) {
        //return LonStatusNotOpen;
    //}
    return ReadUsbNiUid(uidBuf);
}

/*
 * Computes 16-bit CRC.
 * Parameters:
 *   bufInOut: Input buffer containing data to be checksummed; the checksum
 *     will be appended to the end of this buffer
 *   sizeIn: Number of bytes in the input buffer to checksum
 * Returns:
 *   None
 */
 /*
void CRC16(uint8_t bufInOut[], IzotUbits16 sizeIn)
{
    IzotUbits16 poly = 0x1021;       // Generator polynomial
    IzotUbits16 crc = 0xffff;
    IzotUbits16 i,j;
    unsigned char byte, crcbit, databit;
    for (i = 0; i < sizeIn; i++) {
        byte = bufInOut[i];
        for (j = 0; j < 8; j++) {
            crcbit = crc & 0x8000 ? 1 : 0;
            databit = byte & 0x80 ? 1 : 0;
            crc = crc << 1;
            if (crcbit != databit) {
                crc = crc ^ poly;
            }
            byte = byte << 1;
        }
    }
    crc = crc ^ 0xffff;
    bufInOut[sizeIn]     = (crc >> 8);
    bufInOut[sizeIn + 1] = (crc & 0x00FF);
    return;
}
*/
/*
 * Gets a pointer to the transceiver parameters for the specified LON interface.
 * Parameters:
 *   index: The index of the LON interface
 *   p: Pointer to an XcvrParam structure to receive the parameters
 * Returns:
 *   None
 */
/*
void LKGetXcvrParams(XcvrParam *p)
{
  	*p = lonNi.xcvrParams;
}
*/
/*
 * Fetches transceiver parameters from the specified LON PL interface.
 * Parameters:
 *   index: The index of the LON interface
 * Returns:
 *   None
 * Notes:
 *   This function is called periodically to fetch the transceiver parameters
 *   from a LON PL interface.  It is also called once during initialization
 *   of a LON PL interface to kick off the process.  The function just returns
 *   if there is no LON PL interface.
 */
/*
void LKFetchXcvrPl()
{
	const int msgLen = 1;
	const LonDataFrame sicbOut = {LonNiLocalNetMgmtCmd, 14+msgLen,{ 0x70|LNM_TAG, 0x00, msgLen, 
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							 ND_opcode_base|ND_QUERY_XCVR}};
    if (lonNi.isPowerLine) {
	    // Send the fetch message; if send fails, set the fetch flag to try again next time
	    lonNi.fetchXcvrParams = WriteLonUsbMsg((LonDataFrame*)&sicbOut) != LonStatusNoError;
    }
}
*/
#endif  // !LINK_IS(ETHERNET) && !LINK_IS(WIFI)
