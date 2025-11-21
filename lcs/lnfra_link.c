/*
 * infra_link.c
 *
 * Copyright (c) Micro-thermo
 * 
 * Title:   LON Stack Data Link Layer for LON USB
 * Purpose: Implements layer 2 (data link layer) of the ISO/IEC 14908-1
 *          LON protocol stack.
 * Notes:   The functions in this file support LON data links using a
 *          LON USB network interface U60, on a Neuron
 *          processor with MIP firmware.
 */

#include "lcs/lcs_link.h"
#include "vldv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcs/lcs_eia709_1.h"
#include "lcs/lcs_timer.h"
#include "lcs/lcs_node.h"
#include "lcs/lcs_queue.h"
#include "lcs/lcs_netmgmt.h"

// Unique ID fetch interval in milliseconds
#define UNIQUE_ID_FETCH_INTERVAL 500
// LON PL transceiver parameters fetch interval in milliseconds
#define XCVR_PARAM_FETCH_INTERVAL 10000

typedef struct {
	IzotByte cmd;
	IzotByte len;
	IzotByte pdu[MAX_PDU_SIZE];
} L2Frame;

// 1-byte header definition of an LPDU in the queue
typedef struct {
	BITS3(priority,		1,
		  altPath,		1,
		  deltaBL,		6)
} LPDUHeader;

// Number of LON network interfaces to be supported
#define NUM_LON_NI 1

// LON network interface definition structure
typedef struct {
    char *name;
    LonLinkHandle lonLinkhandle;
    Bool linkOpened;
    Bool fetchXcvrParams;
    XcvrParam xcvrParams;
} LonNiDef;

// LON network interface definition array
LonNiDef lonNi =
{
	"LON1", -1, false, false, {{0, 0, 0, 0, 0, 0, 0}},
};

#define LNM_TAG 0x0F	// Tag reserved for local network management

/*****************************************************************
 * Section: Function Definitions
 *****************************************************************/

/*
 * Allocates space for link layer queues.
 * Parameters:
 *   None
 * Returns:
 *   None
 * Notes:
 *   Sets gp->resetOk to FALSE if unable to reset properly.
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
void LKReset(void)
{
    IzotUbits16     queueItemSize;
    IzotByte*       p;  // Used to initialize lkInQ
    IzotUbits16     i;
    LDVCode         lonNiSts = LDV_OK;

    // Allocate and initialize the input queue
    gp->lkInBufSize = DecodeBufferSize((IzotUbits16)gp->nwInBufSize) + 6;
    gp->lkInQCnt    = DecodeBufferCnt((IzotUbits16)gp->nwInQCnt);
    gp->lkInQ       = AllocateStorage((IzotUbits16)(gp->lkInBufSize * gp->lkInQCnt));

    if(gp->lkInQ == NULL)
    {
        ErrorMsg("LKReset: Unable to initialize the input queue.\n");
        gp->resetOk = FALSE;
        return;
    }

    // Initialize the flag in each item of the queue to 0
    p = gp->lkInQ;
    
    for(i = 0; i < gp->lkInQCnt; i++)
    {
        *p = 0;
        p = (Byte *)((char *)p + gp->lkInBufSize);
    }
    
    gp->lkInQHeadPtr = gp->lkInQTailPtr = gp->lkInQ;

    // Allocate and initialize the output queue
    gp->lkOutBufSize = DecodeBufferSize((IzotUbits16)gp->nwOutBufSize);
    gp->lkOutQCnt    = DecodeBufferCnt((IzotUbits16)gp->nwOutQCnt);
    queueItemSize    = gp->lkOutBufSize + sizeof(LKSendParam);

    if(QueueInit(&gp->lkOutQ, queueItemSize, gp->lkOutQCnt)!= LS_SUCCESS)
    {
        ErrorMsg("LKReset: Unable to init the output queue.\n");
        gp->resetOk = FALSE;
        return;
    }

    // Allocate and initialize the priority output queue
    gp->lkOutPriBufSize = gp->lkOutBufSize;
    gp->lkOutPriQCnt    = DecodeBufferCnt((IzotUbits16)gp->nwOutPriQCnt);
    queueItemSize       = gp->lkOutPriBufSize + sizeof(LKSendParam);

    if(QueueInit(&gp->lkOutPriQ, queueItemSize, gp->lkOutPriQCnt) != LS_SUCCESS)
    {
        ErrorMsg("LKReset: Unable to initialize the priority output queue.\n");
        gp->resetOk = FALSE;
        return;
    }

    lonNiSts = OpenLonLink();

    if(lonNiSts != LDV_OK)
    {
        DBG_vPrintf(TRUE, "LKReset: Unable to open LON link %s, error %d\n", lonNi.name, lonNiSts);
        lonNi.linkOpened = false;
    }
    
    lonNi.linkOpened    = true;
    gp->resetOk = TRUE;

    return;
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
void LKSend(void)
{
    LKSendParam*    lkSendParamPtr;
    Queue*          lkSendQueuePtr;
    Byte*           npduPtr;
    LPDUHeader*     lpduHeaderPtr;
  //  Bool            fetchTimerExpired;
    Bool            priority;
	L2Frame		    sicb;

   // fetchTimerExpired = LonTimerExpired(&lonLinkXcvrPlFetchTimer);

    // Make variables point to the new queue
    if(!QueueEmpty(&gp->lkOutPriQ))
    {
        priority       = TRUE;
        lkSendQueuePtr = &gp->lkOutPriQ;
    }
    else if(!QueueEmpty(&gp->lkOutQ))
    {
        priority       = FALSE;
        lkSendQueuePtr = &gp->lkOutQ;
    }
    else
    {
        return; // Nothing to send
    }

	lkSendParamPtr = QueueHead(lkSendQueuePtr);
	npduPtr        = (Byte *) (lkSendParamPtr + 1);

	sicb.cmd = 0x12;
	sicb.len = lkSendParamPtr->pduSize + 1;

	lpduHeaderPtr           = (LPDUHeader *)sicb.pdu;
	lpduHeaderPtr->priority = priority;
	lpduHeaderPtr->altPath  = lkSendParamPtr->altPath;
	lpduHeaderPtr->deltaBL  = lkSendParamPtr->deltaBL;

	// Copy the NPDU
	if(lkSendParamPtr->pduSize <= sizeof(sicb.pdu))
    {
		memcpy(&sicb.pdu[0], npduPtr, lkSendParamPtr->pduSize);
	}
	
    // Send the LPDU to all LON network interfaces
    WriteLonLink(&sicb);

	DeQueue(lkSendQueuePtr);

    return;
}

/*
 * Receives an LPDU from a LON network interface, extracts the NPDU, and transfers
 * the NPDU to the network layer.
 * Parameters:
 *   None
 * Returns:
 *   None
 * Notes:
 *   The LON network interface is typically a U10 or U60 LON USB interface.
 *   Each item of the queue gp->lkInQ has the following form:
 *     flag pduSize LPDU
 *       flag is 1 byte long
 *       pduSize is 2 bytes long
 *       LPDU has header followed by the rest of the LPDU and then CRC
 *         LPDU header is 1 byte long
 *         CRC is 2 bytes
 *   If a packet is in lkInQ then it will fit into nwInQ.
 */
void LKReceive(void)
{
    TSAReceiveParam*    nwReceiveParamPtr;
    IzotByte*           npduPtr;
    LPDUHeader*         lpduHeaderPtr;
    IzotByte*           tempPtr;
    IzotUbits16         lpduSize;
	L2Frame			    sicb;
	
    if(ReadLonLink(&sicb) != LDV_OK)
    {
	  	// No packets to process
	  	return;
	}

    lpduSize 	  =	sicb.len - 3;	                // Subtract 2 for register info and 1 for zero crossing info
    lpduHeaderPtr = (LPDUHeader*)&sicb.pdu[1];	    // Offset is 1 because of zero crossing info
	
	// Throw away layer 2 mode 2 packets that are smaller than 8 bytes long;
    // layer 2 mode 2 network interfaces report CRC errors as a packet with
    // a short length
	if(((sicb.cmd == nicbINCOMING_L2M2) && (lpduSize < 8)) || ((sicb.cmd & 0xF0) == (nicbERROR & 0xF0)))
    {
	  	INCR_STATS(LcsTxError);
		return;
	}
    else if(sicb.cmd != nicbINCOMING_L2M2)
    {
	  	return;
	}

    // CRC check was performed by the LON network interface;
    // increment the valid packet received count
    INCR_STATS(LcsL2Rx);

    // Check if the packet is for us
    if(sicb.cmd != nicbINCOMING_L2M2 || sicb.pdu[0] != nicbLOCALNM)
    {
        INCR_STATS(LcsMissed);
        return;
    }

    // Check if the packet is too small
    if(lpduSize < 8)
    {
        INCR_STATS(LcsRxError);
        return;
    }
    
    if(QueueFull(&gp->nwInQ))
    {
        // Queue is full--lose this packet
        INCR_STATS(LcsMissed);
    }
    else
    {
        // Queue entry available--receive the packet
        nwReceiveParamPtr = QueueTail(&gp->nwInQ);
        npduPtr           = (Byte *)(nwReceiveParamPtr + 1);

        nwReceiveParamPtr->priority = lpduHeaderPtr->priority;
        nwReceiveParamPtr->altPath  = lpduHeaderPtr->altPath;
        tempPtr = (Byte *)((char *)lpduHeaderPtr + 1);
        nwReceiveParamPtr->pduSize  = lpduSize - 3;
        // The following line has been commented out because
        // there is no xcvrParams field in NWReceiveParam and
        // transceiver parameters are only fetched for a PL
        // network interface
 		// nwReceiveParamPtr->xcvrParams = lonLinkXcvrPlParam;
 
        // Copy the NPDU; if it was in link layer's queue, then the size
        // should be sufficient in the network layer's queue as they differ
        // by 3; play safe by checking the size first
        if(nwReceiveParamPtr->pduSize <= gp->nwInBufSize)
        {
            memcpy(npduPtr, tempPtr, nwReceiveParamPtr->pduSize);
        }
        else
        {
            ErrorMsg("LKReceive: NPDU size is too large.\n");
        }
        
        EnQueue(&gp->nwInQ);
    }
    
    *(gp->lkInQHeadPtr) = 0;
    gp->lkInQHeadPtr = gp->lkInQHeadPtr + gp->lkInBufSize;
    
    if(gp->lkInQHeadPtr == (gp->lkInQ + gp->lkInBufSize * gp->lkInQCnt))
    {
        gp->lkInQHeadPtr = gp->lkInQ; // Wrap around
    }
	
    return;
}
