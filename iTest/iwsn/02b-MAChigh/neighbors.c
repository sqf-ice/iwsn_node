/**
\brief General IWSN 02bMAClow-high 'neighbors' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
#include "config.h"    //SET_NEIGHBOR
#include "neighbors.h"
#include "ieee802154e.h"
#include "packet.h"
#include "idmanager.h"
#include "monitor.h"

#ifdef HAVE_DEBUG
#include "debug.h"
#endif

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

neighbors_vars_t neighbors_vars;

//=========================== prototypes ======================================

void registerNewNeighbor(iwsn_addr_t* address,
                         int8_t       rssi,
                         asn_t*       asnTimestamp);
void removeNeighbor(uint8_t neighborIndex);
bool isThisRowMatching(iwsn_addr_t* address, uint8_t rowNumber);
bool isMonChildNeighbor(uint8_t rowNumber);
bool isPeerNeighbor(uint8_t rowNumber);
uint8_t neighbors_getNeighborNum(iwsn_addr_t* neighbor);	//rev.2
neighborRow_t* neighbor_addNewNeishbor(iwsn_addr_t* address);   //v3
//=========================== main ============================================

//=========================== public ==========================================

void neighbors_init()
{
  uint8_t i;
  uint8_t myRole = idmanager_getMyRole();
  // Initialize variables
  memset(&neighbors_vars, 0, sizeof(neighbors_vars_t));
  
  for (i = 0; i < MAXNUMNEIGHBORS; i++)
  {
    neighbors_vars.neighbors[i].used = false;
  }
  if(idmanager_isGateway(myRole))
  {
	iwsn_addr_t* sinkAddr = idmanager_getMyID(ADDR_16B);
	memcpy(&neighbors_vars.sinkAddr, sinkAddr, sizeof(iwsn_addr_t));
  }
}

void neighbors_sinkCheckChild()
{
  uint8_t      i;
  uint16_t     timeSinceHeard;
  iwsn_addr_t* monChildNeighbor = NULL;

  i = 0;
  while (i < MAXNUMNEIGHBORS)
  {
    if (isMonChildNeighbor(i) == true)
    {
      timeSinceHeard = iwsn_asnDiff(&neighbors_vars.neighbors[i].asn, ieee802154e_getCurASN());
      if (timeSinceHeard > MONCHILDTIMEOUT)
      {
    	  monChildNeighbor = &(neighbors_vars.neighbors[i].addr_16b);
    	  ieee802154e_updateNodeSlotInTable(SLOT_DEL, monChildNeighbor, 0);
    	  tsch_sinkStoreDSlot(SLOT_DEL, 0, neighbors_vars.neighbors[i].dedicatedSlot);
    	  removeNeighbor(i);
      }
    }
    i++;
  }
}

void neighbors_nodeCheckChildTimeout()
{
  iwsn_addr_t* childAddr = neighbors_getMyChildAddress();
  uint16_t     timeSinceHeard;
  uint8_t i = 0;
  if(childAddr->type == ADDR_NONE)
  {
	  return;
  }
  while (i < MAXNUMNEIGHBORS)
  {
	if (isThisRowMatching(childAddr, i))
	{
		timeSinceHeard = iwsn_asnDiff(&neighbors_vars.neighbors[i].asn, ieee802154e_getCurASN());
		if (timeSinceHeard > MONCHILDTIMEOUT)
		{
			removeNeighbor(i);
			tsch_updateChildDedicatedSlotoffset(0xFF);
			ieee802154e_updateNodeSlotInTable(SLOT_DEL, childAddr, 0);
			memset(&neighbors_vars.myChildAddr_16b, 0, sizeof(iwsn_addr_t));
			break;
		}else{
			break;
		}
	}
	i++;
  }
}

iwsn_addr_t* neighbors_getChildAddr(iwsn_addr_type_t type)
{
  uint8_t      i;
  iwsn_addr_t* monChildAddr = NULL;

  i = 0;
  while (i < MAXNUMNEIGHBORS)
  {
    if (isMonChildNeighbor(i) == true)
    {
		switch (type)
		{
		  case ADDR_16B:
			  monChildAddr = &(neighbors_vars.neighbors[i].addr_16b);
			break;
		  default:
	#ifdef HAVE_DEBUG
			debug_printError(COMPONENT_NEIGHBORS, ERR_WRONG_ADDR_TYPE,
							 (errorparameter_t)address->type,
							 (errorparameter_t)3);
	#endif
			break;

		}
		return monChildAddr;
    }
    i++;
  }

  return monChildAddr;
}

iwsn_addr_t* neighbors_getTimeoutPeerNeighbor(iwsn_addr_type_t type)
{
  uint8_t      i;
  uint16_t     timeSinceHeard;
  iwsn_addr_t* peerNeighbor = NULL;

  i = 0;
  while (i < MAXNUMNEIGHBORS)
  {
    if (isPeerNeighbor(i) == true)
    {
      timeSinceHeard = iwsn_asnDiff(&neighbors_vars.neighbors[i].asn, ieee802154e_getCurASN());
      if (timeSinceHeard > MONPEERTIMEOUT)
      {
        switch (type)
        {
          case ADDR_16B:
            peerNeighbor = &(neighbors_vars.neighbors[i].addr_16b);
            break;
          default:
#ifdef HAVE_DEBUG
            debug_printError(COMPONENT_NEIGHBORS, ERR_WRONG_ADDR_TYPE,
                             (errorparameter_t)address->type,
                             (errorparameter_t)3);
#endif
            break;
            
        }
        return peerNeighbor;
      }
    }
    i++;
  }
  
  return peerNeighbor;
}

bool neighbors_isNeighbor(iwsn_addr_t* neighbor)
{
  uint8_t i;
  
  for (i = 0;i < MAXNUMNEIGHBORS; i++)
  {
    if (isThisRowMatching(neighbor, i))
    {
      return true;
    }
  }
  return false;
}

bool neighbors_isPreferredParent(iwsn_addr_t* neighbor)
{
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	if (idmanager_isSameAddress(neighbor, &neighbors_vars.myParentAddr_16b))
	{
		ENABLE_INTERRUPTS();
		return true;
	}
	ENABLE_INTERRUPTS();
	return false;
}

bool neighbors_isParentSelected() //v3
{
	if(neighbors_vars.myParentAddr_16b.type == ADDR_16B)
	{
		return true;
	}
	return false;
}

bool neighbors_isParentSink() //v3
{
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	if (idmanager_isSameAddress(&neighbors_vars.myParentAddr_16b, &neighbors_vars.sinkAddr))
	{
		ENABLE_INTERRUPTS();
		return true;
	}
	ENABLE_INTERRUPTS();
	return false;
}

bool neighbors_isSink(iwsn_addr_t* address) //v3
{
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	if (idmanager_isSameAddress(address, &neighbors_vars.sinkAddr))
	{
		ENABLE_INTERRUPTS();
		return true;
	}
	ENABLE_INTERRUPTS();
	return false;
}

iwsn_addr_t*  neighbors_getSinkAddress()   //v3
{
	return &neighbors_vars.sinkAddr;
}

iwsn_addr_t*  neighbors_getMyParentAddress()   //v3
{
	return &neighbors_vars.myParentAddr_16b;
}

iwsn_addr_t*  neighbors_getMyChildAddress()   //v3
{
	return &neighbors_vars.myChildAddr_16b;
}

iwsn_addr_t* neighbors_getParentNeighbor()
{
  uint8_t      i;
  iwsn_addr_t* parentNeighbor = NULL;
  
  // scan through the neighbor table, and populate addrPreferred and addrOther
  for (i = 0; i < MAXNUMNEIGHBORS; i++)
  {
    if (neighbors_vars.neighbors[i].used == true &&
        neighbors_vars.neighbors[i].relationship == NR_PARENT)
    {
      parentNeighbor = &neighbors_vars.neighbors[i].addr_16b;
      break;
    }
  }
  return parentNeighbor;
}

iwsn_addr_t*  neighbors_getManagerAddr()
{
	if (neighbors_vars.managerAddr.type == ADDR_NONE)
	{
		neighbors_vars.managerAddr.type = ADDR_16B;
		neighbors_vars.managerAddr.addr_16b[0] = 0x88;
		neighbors_vars.managerAddr.addr_16b[1] = 0x88;
	}
	return &neighbors_vars.managerAddr;
}

void neighbors_selectParent()  //V3
{
	uint8_t         i;
	neighborRow_t*  parentNeighbor = NULL;
	// scan through the neighbor table, and populate addrPreferred and addrOther
	for (i = 0; i < MAXNUMNEIGHBORS; i++)
	{
		if ((neighbors_vars.neighbors[i].used == true)
			&& (neighbors_vars.neighbors[i].relationship != NR_CHILD)
			&& (neighbors_vars.neighbors[i].isHaveChild == false)
			&& (neighbors_vars.neighbors[i].level != TWOHOP)
#if SET_RSSI
			&& (neighbors_vars.neighbors[i].rssi > RSSI_THRESHOLD)
#endif
#if  SET_NEIGHBOR
			&& (neighbors_vars.neighbors[i].addr_16b.addr_16b[1] == ASSIGNED_PARENT)
#endif
			)
		{
			if (parentNeighbor == NULL)
			{
				parentNeighbor = &neighbors_vars.neighbors[i];
			}else{
				if(parentNeighbor->rssi + 5 < neighbors_vars.neighbors[i].rssi)
				{
					parentNeighbor = &neighbors_vars.neighbors[i];
				}
			}

		}
	}
	parentNeighbor->relationship = NR_PARENT;
	memcpy(&neighbors_vars.myParentAddr_16b, &parentNeighbor->addr_16b, sizeof(iwsn_addr_t));
	if(neighbors_isSink(&parentNeighbor->addr_16b))
	{
		//begin work
		ieee802154e_changeIsWork(true);
	}
	//update adv information
	//ieee802154e_updateAdvAddrInfo(PARENT_ADDR, &parentNeighbor->addr_16b);
}

void neighbors_removeNeighbor(iwsn_addr_t* neighbor)
{
  uint8_t i;

  for (i = 0; i < MAXNUMNEIGHBORS; i++) 
  {
    if (isThisRowMatching(neighbor, i) == true) 
    {
      removeNeighbor(i);
      break;
    }
  }
}

void neighbors_updateNeighborFromadv(iwsn_addr_t* neighbor,
									 uint8_t role,
									 iwsn_addr_t* parentAddr,
									 iwsn_addr_t* childAddr,
									 int8_t rssi,
									 slotOffset_t dedicatedSlot)  //v3
{
	uint8_t i = 0;
	neighborRow_t *tempNeighbor = NULL;
	asn_t* asn = ieee802154e_getCurASN();
	while (i < MAXNUMNEIGHBORS)
	{
		if (isThisRowMatching(neighbor, i))
		{
			tempNeighbor = &neighbors_vars.neighbors[i];
			break;
		}
		i++;
	}

	if (tempNeighbor == NULL)
	{
		if((tempNeighbor = neighbor_addNewNeishbor(neighbor)) == NULL)
		{
			return;
		}
	}
    memcpy(&tempNeighbor->asn, asn, sizeof(asn_t));
	tempNeighbor->role = role;
	tempNeighbor->rssi = rssi;
	tempNeighbor->dedicatedSlot = dedicatedSlot;
	if(neighbors_isSink(neighbor))
	{
		tempNeighbor->level = SINK;
	}
	if(parentAddr->type == ADDR_16B)
	{
		memcpy(&tempNeighbor->parentAddr_16b, parentAddr, sizeof(iwsn_addr_t));
		if(neighbors_isSink(parentAddr))
			tempNeighbor->level = ONEHOP;
		else
			tempNeighbor->level = TWOHOP;
		if(idmanager_isMyAddress(parentAddr))
		{
			if(neighbors_vars.myChildAddr_16b.type == ADDR_NONE)
			{
				tempNeighbor->relationship = NR_CHILD;
				memcpy(&neighbors_vars.myChildAddr_16b, neighbor, sizeof(iwsn_addr_t));
				//update RX dedicated slot of my child
				tsch_updateChildDedicatedSlotoffset(dedicatedSlot);

			}
		}
	}
	if(childAddr->type == ADDR_16B)
	{
		memcpy(&tempNeighbor->childAddr_16b, childAddr, sizeof(iwsn_addr_t));
		//it has child, I can't select it as my parent! my parent's isHaveChild is false.
		if(!idmanager_isMyAddress(childAddr))
		{
			tempNeighbor->isHaveChild = true;
			//if it is my parent but its child is not me, i have to reselect parent
			if(idmanager_isSameAddress(neighbor,&neighbors_vars.myParentAddr_16b))
			{
				ieee802154e_changeIsWork(false);
				//reselect parent
				neighbors_selectParent();
			}
		}else{
			//begin work
			ieee802154e_changeIsWork(true);
		}
	}
}


void neighbors_sinkUpdateNeighbors(iwsn_addr_t* neighbor,
									 uint8_t role,
									 iwsn_addr_t* parentAddr,
									 iwsn_addr_t* childAddr,
									 int8_t rssi,
									 slotOffset_t dedicatedSlot,
									 slotOffset_t childSlot)  //v3
{
	uint8_t i = 0;
	neighborRow_t *tempNeighbor = NULL;
	asn_t* asn = ieee802154e_getCurASN();
	while (i < MAXNUMNEIGHBORS)
	{
		if (isThisRowMatching(neighbor, i))
		{
			tempNeighbor = &neighbors_vars.neighbors[i];
			break;
		}
		i++;
	}

	if (tempNeighbor == NULL)
	{
		if((tempNeighbor = neighbor_addNewNeishbor(neighbor)) == NULL)
		{
			return;
		}
	}
    memcpy(&tempNeighbor->asn, asn, sizeof(asn_t));
	tempNeighbor->role = role;
	tempNeighbor->rssi = rssi;
	tempNeighbor->dedicatedSlot = dedicatedSlot;
	if(parentAddr->type == ADDR_16B)
	{
		if(idmanager_isMyAddress(parentAddr))
		{
			tempNeighbor->level = ONEHOP;
			tempNeighbor->relationship = NR_CHILD;
			if(dedicatedSlot >= 0 && dedicatedSlot <= 9)
			{
				//store update slot RX
				tsch_sinkStoreDSlot(SLOT_ADD, CELLTYPE_RX, dedicatedSlot);
			}
		}else{
			tempNeighbor->level = TWOHOP;
			if(dedicatedSlot >= 0 && dedicatedSlot <= 9)
			{
				//store update slot Idle
				tsch_sinkStoreDSlot(SLOT_ADD, CELLTYPE_OFF, dedicatedSlot);
			}
		}
	}
	if(childAddr->type == ADDR_16B)
	{
		memcpy(&tempNeighbor->childAddr_16b, childAddr, sizeof(iwsn_addr_t));
	}else{
		if(tempNeighbor->childAddr_16b.type != ADDR_NONE)
		{
			ieee802154e_updateNodeSlotInTable(SLOT_DEL, childAddr, 0);
		}
	}
}

iwsn_addr_t* neighbors_sinkFindNextHop(iwsn_addr_t* destination)  //v3
{
	uint8_t i=0;
	neighborRow_t* tempNeighbor = NULL;
	if(destination->type != ADDR_16B)
	{
		return NULL;
	}else{
		while (i < MAXNUMNEIGHBORS)
		{
			tempNeighbor = &neighbors_vars.neighbors[i];
			if(tempNeighbor->used &&
			   ((idmanager_isSameAddress(destination, &tempNeighbor->addr_16b) &&
			   tempNeighbor->relationship == NR_CHILD) ||
			   idmanager_isSameAddress(destination, &tempNeighbor->childAddr_16b)))
			{
				return &tempNeighbor->addr_16b;
			}
			i++;
		}
	}
	return NULL;
}

void neighbors_updateMySinkAddr(iwsn_addr_t* sinkAddr)   //v3
{
	memcpy(&neighbors_vars.sinkAddr, sinkAddr, sizeof(iwsn_addr_t));
}

void neighbors_resetVars()
{
	memset(&neighbors_vars.myParentAddr_16b, 0, sizeof(iwsn_addr_t));
	memset(&neighbors_vars.myChildAddr_16b, 0, sizeof(iwsn_addr_t));
}

void neighbors_indicateRx(iwsn_addr_t* l2_src, int8_t rssi, asn_t* asnTimestamp) {
	uint8_t i = 0;
	while (i < MAXNUMNEIGHBORS) {
		if (isThisRowMatching(l2_src, i)) {
			neighbors_vars.neighbors[i].rssi = rssi;
			memcpy(&neighbors_vars.neighbors[i].asn, asnTimestamp, sizeof(asn_t));
			return;
		}
		i++;
	}
}

void neighbors_indicateTx(iwsn_addr_t* l2_src, asn_t* asnTimestamp)
{
	uint8_t i = 0;
	while (i < MAXNUMNEIGHBORS) {
		if (isThisRowMatching(l2_src, i)) {
			memcpy(&neighbors_vars.neighbors[i].asn, asnTimestamp, sizeof(asn_t));
			return;
		}
		i++;
	}
}
//=========================== private =========================================

neighborRow_t* neighbor_addNewNeishbor(iwsn_addr_t* address)   //v3
{
	uint8_t i = 0;
    while (i < MAXNUMNEIGHBORS)
    {
      if (neighbors_vars.neighbors[i].used == false)
      {
        // add this neighbor
        neighbors_vars.neighbors[i].used                   = true;
        switch (address->type)
        {
          case ADDR_16B:
            memcpy(&neighbors_vars.neighbors[i].addr_16b,  address, sizeof(iwsn_addr_t));
            return &neighbors_vars.neighbors[i];
          default:
            neighbors_vars.neighbors[i].used               = false;
            return NULL;
        }
        break;
      }
      i++;
    }
    return NULL;
}


void removeNeighbor(uint8_t neighborIndex)
{
  neighbors_vars.neighbors[neighborIndex].used                      = false;
  neighbors_vars.neighbors[neighborIndex].addr_16b.type             = ADDR_NONE;
  neighbors_vars.neighbors[neighborIndex].childAddr_16b.type        = ADDR_NONE;
  neighbors_vars.neighbors[neighborIndex].parentAddr_16b.type       = ADDR_NONE;
  neighbors_vars.neighbors[neighborIndex].isHaveChild               = false;
  neighbors_vars.neighbors[neighborIndex].rssi                      = 0;
  neighbors_vars.neighbors[neighborIndex].asn.bytes0and1            = 0;
  neighbors_vars.neighbors[neighborIndex].asn.bytes2and3            = 0;
  neighbors_vars.neighbors[neighborIndex].asn.byte4                 = 0;
  neighbors_vars.neighbors[neighborIndex].role                      = 0;
  neighbors_vars.neighbors[neighborIndex].level                     = 0;
  neighbors_vars.neighbors[neighborIndex].relationship              = 0;
  neighbors_vars.neighbors[neighborIndex].dedicatedSlot				= 0x0F;     //mj20160310
}

bool isThisRowMatching(iwsn_addr_t* address, uint8_t rowNumber)
{
  switch (address->type)
  {
    case ADDR_16B:
      return neighbors_vars.neighbors[rowNumber].used 
        && idmanager_isSameAddress(address, &neighbors_vars.neighbors[rowNumber].addr_16b);
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_NEIGHBORS, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)address->type,
                       (errorparameter_t)3);
#endif
      return false;
  }
}

bool isMonChildNeighbor(uint8_t rowNumber)
{
  if (neighbors_vars.neighbors[rowNumber].used == true && 
      neighbors_vars.neighbors[rowNumber].relationship == NR_CHILD)
  {
    return true;
  }
   return false;
}

bool isPeerNeighbor(uint8_t rowNumber)
{
  if (neighbors_vars.neighbors[rowNumber].used == true && 
      neighbors_vars.neighbors[rowNumber].relationship == NR_PEER)
  {
    return true;
  }
   return false;
}

uint8_t neighbors_getNeighborNum(iwsn_addr_t* neighbor)		//rev.2
{
  uint8_t i;

  for (i = 0;i < MAXNUMNEIGHBORS; i++)
  {
    if (isThisRowMatching(neighbor, i))
    {
      return i;
    }
  }
  return 0xFF;
}

//=========================== interrupt handlers ==============================

