/**
\brief General IWSN 02bMAClow-high 'tsch' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __TSCH_H
#define __TSCH_H

//=========================== include =========================================

#include "stdint.h"
#include "platform.h"
#include "iwsn.h"
#include "stdbool.h"


//=========================== define ==========================================

#define DEFAULT_FRAME_LENGTH	 100    // the slot number of per superframe
#define MAXACTIVESLOTS            50    // the maximum number of active slots
#define NUMSLOTSPERBLOCK           4    // number of slots per block assigned to a node
#define RESERVEDSLOTS              0    // the slot number of reserved slots for join and re-TX


#define SYNCHSLOT                 11    //v3
//rev.2
#define SINKADV				      0		// the slot number of first hop node monitor sink's ADV
#define ADVSTART			      0 	// the slot start of ADV slots exclude sink
#define ADVEND				      10 	// the slot end of ADV slots exclude sink

#define ONEDOWNSLOTS			  6		// the slot number of first hop down
#define ONEDOWNSTARTSLOT		  12	// the slot start of first hop down
#define ONEDOWNENDSLOT			  17	// the slot end of first hop down

#define TWODOWNSLOTS			  6		// the slot number of first hop down
#define TWODOWNSTARTSLOT		  18	// the slot start of first hop down
#define TWODOWNENDSLOT			  23	// the slot end of first hop down

#define NOTIFYSLOTS			      4		// the slot number of Notify node reselect slots
#define NOTIFYSTARTSLOT			  24    // the slot number of Notify node reselect slots start
#define NOTIFYENDSLOT			  27    // the slot number of Notify node reselect slots end

#define DEDICATEDSLOTS            10    // the slot number of dedicated slots for join trx
#define DEDICATEDSTARTSLOT		  28	// the slot number of dedicated slots start
#define DEDICATEDENDSLOT		  37	// the slot number of dedicated slots start

#define TWOUPSLOTS				  6		// the slot number of first hop down
#define TWOUPSTARTSLOT	 		  38	// the slot start of first hop down
#define TWOUPENDSLOT		   	  43	// the slot end of first hop down

#define ONEUPSLOTS			  	  6		// the slot number of first hop down
#define ONEUPSTARTSLOT		  	  44	// the slot start of first hop down
#define ONEUPENDSLOT			  49	// the slot end of first hop down

#define MINBE                     0     // min backoff exponent, used in shared TX slots	//v2
#define MAXBE                     4     // max backoff exponent, used in shared TX slots

#define LEVEL1			1
#define LEVEL2			2
#define LEVEL3			3
//=========================== typedef =========================================

typedef uint8_t    channelOffset_t;
typedef uint8_t    slotOffset_t;
typedef uint16_t   frameLength_t;

typedef enum {
  CELLTYPE_OFF              = 0,
  CELLTYPE_ADV              = 1,
  CELLTYPE_MON              = 2,
  CELLTYPE_TX               = 3,
  CELLTYPE_RX               = 4,
  CELLTYPE_TXRX             = 5,
  CELLTYPE_SERIALRX         = 6,
  CELLTYPE_CANTRX           = 7,
  CELLTYPE_SYNC             = 8,
  CELLTYPE_JOIN
} cellType_t;

#pragma pack(1)
typedef struct {
  slotOffset_t    slotOffset;
  cellType_t      type;
  bool            shared;
  uint8_t         channelOffset;
  iwsn_addr_t     neighbor;
  uint8_t         backoffExponent;
  uint8_t         backoff;
  void*           next;
}__attribute__ ((packed)) tschEntry_t;
#pragma pack()

typedef struct {
  tschEntry_t     tschBuf[MAXACTIVESLOTS];
  tschEntry_t*    currentTschEntry;
  uint16_t        frameLength;
  bool		  	  UpdateFlag;
  slotOffset_t    MyAdvSlotOffset;
  slotOffset_t    MyDedicatedSlot;
  slotOffset_t    ChildSlotOffset;
} tsch_vars_t;

typedef struct {
	uint8_t 	 slotType;
	uint8_t      slotOffset;
}tsch_sinkSlot_t;





//=========================== variables =======================================

//=========================== prototypes ======================================

void            tsch_init();

void            tsch_setFrameLength(frameLength_t newFrameLength);
void 			tsch_updateChildDedicatedSlotoffset(slotOffset_t slotOffset);   //v3
void 			tsch_updateMydedicatedSlotoffset(slotOffset_t slotOffset);  //v3
void 			tsch_updateAdvSlotoffset(slotOffset_t slotOffset);  //v3
frameLength_t   tsch_getFrameLength();
cellType_t      tsch_getCurCellType();
cellType_t      tsch_getCellType(slotOffset_t slotOffset);
channelOffset_t tsch_getChannelOffset();
void            tsch_getNeighbor(iwsn_addr_t* addrToWrite);
void            tsch_addActiveSlot(slotOffset_t    slotOffset,
                                   cellType_t      type,
                                   bool            shared,
                                   uint8_t         channelOffset,
                                   iwsn_addr_t*    neighbor);
void            tsch_delActiveSlot(slotOffset_t    slotOffset,
                                   uint8_t         channelOffset);
void            tsch_updateActiveSlot(slotOffset_t    slotOffset,
                                      cellType_t      type,
                                      bool            shared,
                                      uint8_t         channelOffset,
                                      iwsn_addr_t*    neighbor);
void            tsch_advanceSlot();
void            tsch_syncSlotOffset(slotOffset_t targetSlotOffset);
slotOffset_t    tsch_getNextActiveSlotOffset();
slotOffset_t    tsch_getMirrorSlotOffset(slotOffset_t slotOffset);
slotOffset_t    tsch_getMyDedicatedSlotOffset();   //v3
slotOffset_t    tsch_getMyChildSlotOffset(); //v3
slotOffset_t    tsch_getMyAdvSlot(); //v3
bool            tsch_getOkToSend();
bool            tsch_isSharedSlot();
bool 		    tsch_isDSlotSelected();  //v3
void            tsch_updateNodeSlot();  //v3
void 			tsch_updateSinkSlot();  //v3
void 			tsch_sinkStoreDSlot(uint8_t actType, uint8_t slotType, uint8_t slotOffset);  //v3
#endif
