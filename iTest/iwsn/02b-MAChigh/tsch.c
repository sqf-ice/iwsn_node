/**
\brief General IWSN 02bMAClow-high 'tsch' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "string.h"
#include "tsch.h"
#include "idmanager.h"
#include "ieee802154e.h"
#include "neighbors.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

tsch_vars_t tsch_vars;
tsch_sinkSlot_t tsch_sinkSlot[DEDICATEDSLOTS];
//=========================== prototypes ======================================

void tsch_resetEntry(tschEntry_t* pTschEntry);
//void tsch_SinkSerialSlotInit();
//=========================== main ============================================

//=========================== public ==========================================

void tsch_init()
{
  uint8_t i;
  iwsn_addr_t neighbor;
  uint8_t myRole = idmanager_getMyRole();
  iwsn_addr_t* myAddr = idmanager_getMyID(ADDR_16B);
  //the initial adv slotoffset is the low 4bits of my address
  slotOffset_t myAdvSlotoffset = myAddr->addr_16b[1] & 0x0F;
  // reset local variables
  memset(&tsch_vars, 0, sizeof(tsch_vars_t));
  memset(&neighbor, 0, sizeof(iwsn_addr_t));
  
  for (i = 0; i < MAXACTIVESLOTS+1; i++)
  {
    tsch_resetEntry(&tsch_vars.tschBuf[i]);
  }

  // set frame length
  tsch_setFrameLength(DEFAULT_FRAME_LENGTH);

  // initiate the slots according to the node role
  if (idmanager_isGateway(myRole) == true)
  {
    for (i = 0; i < DEDICATEDSLOTS; i++)
    {
    	memset(&tsch_sinkSlot[i], 0x0F, sizeof(tsch_sinkSlot_t));
    }

    for(i = ADVSTART; i <= ADVEND; i++)
    {
    	if(i == SINKADV)
        tsch_addActiveSlot(i, CELLTYPE_ADV, false, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
    	else
        tsch_addActiveSlot(i, CELLTYPE_MON, false, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
    }
    for(i = ONEDOWNSTARTSLOT; i <= ONEDOWNENDSLOT; i++)
    {
    	tsch_addActiveSlot(i, CELLTYPE_TX, true, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
    }
    for(i = ONEUPSTARTSLOT; i <= ONEUPENDSLOT; i++)
    {
    	tsch_addActiveSlot(i, CELLTYPE_RX, true, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
    }
  } else {
	tsch_updateAdvSlotoffset(myAdvSlotoffset);
	tsch_updateMydedicatedSlotoffset(0xFF);
	tsch_updateChildDedicatedSlotoffset(0xFF);
	for(i = ADVSTART; i <= ADVEND; i++)
	{
		if(i == myAdvSlotoffset)
		tsch_addActiveSlot(i, CELLTYPE_ADV, false, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
		else
		tsch_addActiveSlot(i, CELLTYPE_MON, false, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
	}
  }
  for(i = NOTIFYSTARTSLOT; i <= NOTIFYENDSLOT; i++)
  {
	  tsch_addActiveSlot(i, CELLTYPE_TXRX, true, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
  }
  tsch_addActiveSlot(SYNCHSLOT,CELLTYPE_SYNC,false, (SYNCHRONIZING_CHANNEL - HEAD_CHANNEL), &neighbor);
}

/**
\brief Set frame length.

\param newFrameLength The new frame length.
 */
void tsch_setFrameLength(frameLength_t newFrameLength)
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  tsch_vars.frameLength = newFrameLength;
  ENABLE_INTERRUPTS();
}

/**
 * update my dedicated slotoffset
 * param slotOffset The new slotoffset.
 */
void tsch_updateMydedicatedSlotoffset(slotOffset_t slotOffset)  //v3
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  tsch_vars.MyDedicatedSlot = slotOffset;
  ENABLE_INTERRUPTS();
}

/**
 * update my child dedicated slotoffset
 * param slotOffset The new slotoffset.
 */
void tsch_updateChildDedicatedSlotoffset(slotOffset_t slotOffset)  //v3
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  tsch_vars.ChildSlotOffset = slotOffset;
  ENABLE_INTERRUPTS();
}

/**
 * update my child dedicated slotoffset
 * param slotOffset The new slotoffset.
 */
void tsch_updateAdvSlotoffset(slotOffset_t slotOffset)  //v3
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  tsch_vars.MyAdvSlotOffset = slotOffset;
  ENABLE_INTERRUPTS();
}

/**
\brief Get the frame length.

\returns The frame length.
*/
frameLength_t tsch_getFrameLength()
{
  frameLength_t ret;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  ret = tsch_vars.frameLength;
  ENABLE_INTERRUPTS();
  return ret;   
}

/**
\brief Get the type of the current schedule tsch entry.

\returns The type of the current schedule tsch entry.
 */
cellType_t tsch_getCurCellType()
{
  cellType_t res = CELLTYPE_OFF;
  INTERRUPT_DECLARATION();
  
  DISABLE_INTERRUPTS();
  if (tsch_vars.currentTschEntry != NULL)
  {
    res= tsch_vars.currentTschEntry->type;
  }
  ENABLE_INTERRUPTS();
  
  return res;
}

cellType_t tsch_getCellType(slotOffset_t slotOffset)
{
  tschEntry_t* nextSlotWalker;
  cellType_t res = CELLTYPE_OFF;
  INTERRUPT_DECLARATION();
  
  DISABLE_INTERRUPTS();
  
  nextSlotWalker = tsch_vars.currentTschEntry;
  while (nextSlotWalker != NULL)
  {
    if (nextSlotWalker->slotOffset == slotOffset)
    {
      res = nextSlotWalker->type;
      break;
    }
    
    nextSlotWalker = nextSlotWalker->next;
    if (nextSlotWalker == tsch_vars.currentTschEntry)
    {
      break;
    }
  }
  
  ENABLE_INTERRUPTS();
  
  return res;
}

/**
\brief Get the channel offset of the current schedule tsch entry.

\returns The channel offset of the current schedule tsch entry.
 */
channelOffset_t tsch_getChannelOffset()
{
  channelOffset_t res;
  INTERRUPT_DECLARATION();
  
  DISABLE_INTERRUPTS();
  res= tsch_vars.currentTschEntry->channelOffset;
  ENABLE_INTERRUPTS();
  
  return res;
}

/**
\brief Get the neighbor associated wit the current schedule tsch entry.

\returns The neighbor associated wit the current schedule tsch entry.
*/
void tsch_getNeighbor(iwsn_addr_t* addrToWrite)
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();  
  memcpy(addrToWrite, &(tsch_vars.currentTschEntry->neighbor), sizeof(iwsn_addr_t));
  ENABLE_INTERRUPTS();
}

slotOffset_t tsch_getMyDedicatedSlotOffset() //v3
{
	slotOffset_t res;
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	res = tsch_vars.MyDedicatedSlot;
	ENABLE_INTERRUPTS();
	return res;
}

slotOffset_t tsch_getMyChildSlotOffset() //v3
{
	slotOffset_t res;
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	res = tsch_vars.ChildSlotOffset;
	ENABLE_INTERRUPTS();
	return res;
}

slotOffset_t tsch_getMyAdvSlot() //v3
{
	slotOffset_t res;
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	res = tsch_vars.MyAdvSlotOffset;
	ENABLE_INTERRUPTS();
	return res;
}


/**
\brief Add a new active slot into the tsch schedule.

\param    slotOffset The new active slot slotOffset.
\param          type The new active slot type.
\param        shared The new active slot shared.
\param channelOffset The new active slot channelOffset.
\param      neighbor The new active slot neighbor.
 */
void tsch_addActiveSlot(slotOffset_t    slotOffset,
                        cellType_t      type,
                        bool            shared,
                        uint8_t         channelOffset,
                        iwsn_addr_t*    neighbor)
{
  tschEntry_t* slotContainer;
  tschEntry_t* previousSlotWalker;
  tschEntry_t* nextSlotWalker;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  // find an empty tsch schedule entry container
  slotContainer = &tsch_vars.tschBuf[0];
  while (slotContainer->type != CELLTYPE_OFF 
         && slotContainer <= &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    slotContainer++;
  }
  if (slotContainer > &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    // tsch has overflown
    ENABLE_INTERRUPTS();
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_TSCH,
                     ERR_TSCH_FULL,
                     (errorparameter_t)MAXACTIVESLOTS,
                     (errorparameter_t)0);
#endif
    return;
  }
  
  // fill that tsch entry with parameters passed
  slotContainer->slotOffset                = slotOffset;
  slotContainer->type                      = type;
  slotContainer->shared                    = shared;
  slotContainer->channelOffset             = channelOffset;
  memcpy(&slotContainer->neighbor, neighbor, sizeof(iwsn_addr_t));

  if (tsch_vars.currentTschEntry == NULL)
  {
    // this is the first active slot added
    // the next slot of this slot is this slot
    slotContainer->next                    = slotContainer;
    
    // current slot points to this slot
    tsch_vars.currentTschEntry    = slotContainer;  
  }
  else
  {
    // this is NOT the first active slot added
    // find position in schedule tsch list
    previousSlotWalker                     = tsch_vars.currentTschEntry;
    while (1)
    {
      nextSlotWalker                       = previousSlotWalker->next;
      if (((previousSlotWalker->slotOffset < slotContainer->slotOffset) 
           && (slotContainer->slotOffset < nextSlotWalker->slotOffset))
       || ((previousSlotWalker->slotOffset < slotContainer->slotOffset) 
           && (nextSlotWalker->slotOffset <= previousSlotWalker->slotOffset)) 
       || ((slotContainer->slotOffset < nextSlotWalker->slotOffset) 
           && (nextSlotWalker->slotOffset <= previousSlotWalker->slotOffset)))
      {
        break;
      }
      previousSlotWalker                   = nextSlotWalker;
    }    
    // insert between previousSlotWalker and nextSlotWalker
    previousSlotWalker->next               = slotContainer;
    slotContainer->next                    = nextSlotWalker;
  }
  
  ENABLE_INTERRUPTS();
}

/**
\brief Delete an active slot from the tsch schedule.

\param    slotOffset The active slot slotOffset.
\param channelOffset The active slot channelOffset.
 */
void tsch_delActiveSlot(slotOffset_t    slotOffset,
                        uint8_t         channelOffset)
{
  tschEntry_t* slotContainer;
  tschEntry_t* previousSlotWalker;
  tschEntry_t* nextSlotWalker;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  // find the position of the tsch schedule entry
  slotContainer = &tsch_vars.tschBuf[0];
  while ((slotContainer->type == CELLTYPE_OFF 
         || slotContainer->slotOffset != slotOffset
         || slotContainer->channelOffset != channelOffset)
         && slotContainer <= &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    slotContainer++;
  };
  
  if (slotContainer > &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    // tsch has overflown
    ENABLE_INTERRUPTS();
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_TSCH,
                     ERR_NO_TSCH_ENTRY,
                     (errorparameter_t)slotOffset,
                     (errorparameter_t)channelOffset);
#endif
    return;
  }
  
  // remove the tsch entry from the list
  if (tsch_vars.currentTschEntry == slotContainer 
      && slotContainer->next == slotContainer)
  {
    tsch_vars.currentTschEntry = NULL;
  }
  else
  {
    previousSlotWalker                    = tsch_vars.currentTschEntry;
    if (previousSlotWalker->next != previousSlotWalker 
        && tsch_vars.currentTschEntry == slotContainer)
    {
      tsch_vars.currentTschEntry          = previousSlotWalker->next;
      previousSlotWalker                  = tsch_vars.currentTschEntry;		
    }
	
    while (previousSlotWalker->next != tsch_vars.currentTschEntry)  
    {
      nextSlotWalker = previousSlotWalker->next;
      if (nextSlotWalker == slotContainer)
      {
        previousSlotWalker->next		= nextSlotWalker->next;
	break;
      }
      previousSlotWalker                 = nextSlotWalker;
    }
  }
  
  slotContainer->type = CELLTYPE_OFF;
  //slotContainer->slotOffset     = 0;
  //slotContainer->channelOffset  = 0;
  //slotContainer->shared         = false;
  
  ENABLE_INTERRUPTS();
}

/**
\brief Update an active slot info. in the tsch schedule.

\param    slotOffset The new active slot slotOffset.
\param          type The new active slot type.
\param        shared The new active slot shared.
\param channelOffset The new active slot channelOffset.
\param      neighbor The new active slot neighbor.
 */
void tsch_updateActiveSlot(slotOffset_t    slotOffset,
                           cellType_t      type,
                           bool            shared,
                           uint8_t         channelOffset,
                           iwsn_addr_t*    neighbor)
{
  tschEntry_t* slotContainer;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  slotContainer = &tsch_vars.tschBuf[0];
  while ((slotContainer->slotOffset != slotOffset || 
          slotContainer->type == CELLTYPE_OFF) &&
         slotContainer <= &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    slotContainer++;
  }
  if (slotContainer > &tsch_vars.tschBuf[MAXACTIVESLOTS - 1])
  {
    // tsch has overflown
    ENABLE_INTERRUPTS();
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_TSCH,
                     ERR_NO_TSCH_ENTRY,
                     (errorparameter_t)slotOffset,
                     (errorparameter_t)0);
#endif
    tsch_addActiveSlot(slotOffset, type, shared, channelOffset, neighbor);
    return;
  }
  
  // update the slot entry info.
  slotContainer->type                      = type;
  slotContainer->shared                    = shared;
  slotContainer->channelOffset             = channelOffset;
  memcpy(&slotContainer->neighbor, neighbor, sizeof(iwsn_addr_t));

  ENABLE_INTERRUPTS();
  return;
}

void tsch_advanceSlot()
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  // advance to next active slot
  if (tsch_vars.currentTschEntry != NULL)
  {
    tsch_vars.currentTschEntry = tsch_vars.currentTschEntry->next;
  }

  ENABLE_INTERRUPTS();
}

void tsch_syncSlotOffset(slotOffset_t targetSlotOffset)
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  while (tsch_vars.currentTschEntry->slotOffset != targetSlotOffset)
  {
    tsch_advanceSlot();
  }
  ENABLE_INTERRUPTS();
}

slotOffset_t tsch_getNextActiveSlotOffset()
{
  slotOffset_t res = 0;  
  INTERRUPT_DECLARATION();
  
  // return next active slot's slotOffset
  DISABLE_INTERRUPTS();
  if (tsch_vars.currentTschEntry != NULL)
  {
    res = ((tschEntry_t*)(tsch_vars.currentTschEntry->next))->slotOffset;
  }
  ENABLE_INTERRUPTS();
  
  return res;
}

slotOffset_t tsch_getMirrorSlotOffset(slotOffset_t slotOffset)
{
  slotOffset_t  res = 0;
  uint16_t      mirrorBlockNum;

  mirrorBlockNum = (tsch_getFrameLength() - RESERVEDSLOTS)/NUMSLOTSPERBLOCK - slotOffset/NUMSLOTSPERBLOCK;
  res = mirrorBlockNum * NUMSLOTSPERBLOCK + slotOffset % NUMSLOTSPERBLOCK;

  return res;
}

/**
\brief Check whether I can send on this slot.

This function is called at the beginning of every TX slot. If the slot is not a
shared slot, it always return true. If the slot is a shared slot, it decrements
the backoff counter and returns true only if it hits 0.

\returns true if it is OK to send on this slot, false otherwise.
 */
bool tsch_getOkToSend()
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  // decrement backoff of that slot
  if (tsch_vars.currentTschEntry->backoff > 0)
  {
    tsch_vars.currentTschEntry->backoff--;
  }
  // check whether backoff has hit 0
  if (tsch_vars.currentTschEntry->shared == false
      || (tsch_vars.currentTschEntry->shared == true 
          && tsch_vars.currentTschEntry->backoff == 0))
  {
    ENABLE_INTERRUPTS();
    return true;
  }
  else
  {
    ENABLE_INTERRUPTS();
    return false;
  }
}

bool tsch_isSharedSlot()
{
  bool res;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  res = tsch_vars.currentTschEntry->shared;
  ENABLE_INTERRUPTS();
  return res;
}

bool tsch_isDSlotSelected()
{
  bool res = false;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  if(tsch_vars.MyDedicatedSlot != 0xFF)
  {
	  res = true;
  }
  ENABLE_INTERRUPTS();
  return res;
}

void tsch_updateNodeSlot()
{
	uint8_t i;
	iwsn_addr_t   tempNeighbor;
	memset(&tempNeighbor, 0, sizeof(iwsn_addr_t));
	uint8_t channeloffset = RADIO_CHANNEL - HEAD_CHANNEL;
	iwsn_addr_t* childAddr = neighbors_getMyChildAddress();
	iwsn_addr_t* parentAddr = neighbors_getMyParentAddress();
	slotOffset_t myDedicatedSlot = tsch_getMyDedicatedSlotOffset();
	slotOffset_t myChildSlot = tsch_getMyChildSlotOffset();
	if(myDedicatedSlot != 0xFF)
		tsch_updateActiveSlot(myDedicatedSlot+DEDICATEDSTARTSLOT, CELLTYPE_TX, false, channeloffset, parentAddr);

	if(childAddr->type == ADDR_16B)
	{
		tsch_updateActiveSlot(myChildSlot + DEDICATEDSTARTSLOT, CELLTYPE_RX, false, channeloffset, childAddr);
		for(i=TWODOWNSTARTSLOT;i<=TWODOWNENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_TX, false, channeloffset, childAddr);
		}
		for(i=TWOUPSTARTSLOT;i<=TWOUPENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_RX, false, channeloffset, childAddr);
		}
	}else{
		for(i=TWODOWNSTARTSLOT;i<=TWODOWNENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_CANTRX, false, channeloffset, childAddr);
		}
		for(i=TWOUPSTARTSLOT;i<=TWOUPENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_CANTRX, false, channeloffset, childAddr);
		}
	}
	if(neighbors_isParentSink())
	{
		for(i=ONEDOWNSTARTSLOT;i<=ONEDOWNENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_RX, false, channeloffset, parentAddr);
		}
		for(i=ONEUPSTARTSLOT;i<=ONEUPENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_TX, false, channeloffset, parentAddr);
		}
	}else{
		for(i=TWODOWNSTARTSLOT;i<=TWODOWNENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_RX, false, channeloffset, &tempNeighbor);
		}
		for(i=TWOUPSTARTSLOT;i<=TWOUPENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_TX, false, channeloffset, parentAddr);
		}
		for(i=ONEDOWNSTARTSLOT;i<=ONEDOWNENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_CANTRX, false, channeloffset, &tempNeighbor);
		}
		for(i=ONEUPSTARTSLOT;i<=ONEUPENDSLOT;i++)
		{
			tsch_updateActiveSlot(i, CELLTYPE_CANTRX, false, channeloffset, &tempNeighbor);
		}
	}
}

void tsch_sinkStoreDSlot(uint8_t actType, uint8_t slotType, uint8_t slotOffset)
{
	uint8_t i = 0, mark = 0xff;
	if(actType == SLOT_ADD)
	{
		while(i < DEDICATEDSLOTS)
		{
			if(tsch_sinkSlot[i].slotOffset == slotOffset)
			{
				mark = 0xff;
				break;
			}
			if(mark == 0xff && tsch_sinkSlot[i].slotOffset == 0x0F)
			{
				mark = i;
			}
			i++;
		}
		if(mark != 0xff)
		{
			tsch_sinkSlot[mark].slotOffset = slotOffset;
			tsch_sinkSlot[mark].slotType   = slotType;
		}

	}else if(actType == SLOT_DEL)
	{
		while(i < DEDICATEDSLOTS)
		{
			if(tsch_sinkSlot[i].slotOffset == slotOffset)
			{
				tsch_sinkSlot[i].slotOffset = 0x0F;
				tsch_sinkSlot[i].slotType   = 0x0F;
				break;
			}
			i++;
		}
	}
}

void tsch_updateSinkSlot()
{
	uint8_t i;
	iwsn_addr_t tempNeighbor;
	uint8_t channeloffset = RADIO_CHANNEL - HEAD_CHANNEL;
	memset(&tempNeighbor, 0, sizeof(iwsn_addr_t));
	for(i = 0; i < DEDICATEDSLOTS; i++)
	{
		if(tsch_sinkSlot[i].slotOffset != 0x0F)
		{
			if(tsch_sinkSlot[i].slotType == CELLTYPE_RX)
			{
				tsch_updateActiveSlot(tsch_sinkSlot[i].slotOffset + DEDICATEDSTARTSLOT, tsch_sinkSlot[i].slotType,
						false, channeloffset, &tempNeighbor);
			}else if(tsch_sinkSlot[i].slotType == CELLTYPE_OFF)
			{
				tsch_delActiveSlot(tsch_sinkSlot[i].slotOffset + DEDICATEDSTARTSLOT, channeloffset);
			}

		}
	}
}

//=========================== private =========================================

void tsch_resetEntry(tschEntry_t* pTschEntry)
{
  pTschEntry->type                     = CELLTYPE_OFF;
  pTschEntry->shared                   = false;
  pTschEntry->backoffExponent          = MINBE - 1;
  pTschEntry->backoff                  = 0;
  pTschEntry->channelOffset            = 0;
  pTschEntry->neighbor.type            = ADDR_NONE;
  pTschEntry->neighbor.addr_16b[0]     = 0x00;
  pTschEntry->neighbor.addr_16b[1]     = 0x00;
}




//=========================== interrupt handlers ==============================
