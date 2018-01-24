/**
\brief General IWSN 02bMAClow-high 'neighbors' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __NEIGHBOR_H
#define __NEIGHBOR_H

//=========================== include =========================================

#include "string.h"
#include "iwsn.h"
#include "platform.h"
#include "stdbool.h"
#include "tsch.h"

//=========================== define ==========================================

#define MAXNUMNEIGHBORS     10

enum {
  NR_PEER            = 0,
  NR_CHILD           = 1,
  NR_PARENT          = 2,
};

enum {
	SINK			= 0,
	ONEHOP          = 1,
	TWOHOP          = 2,
};

//=========================== typedef =========================================

#pragma pack(1)
typedef struct {
	bool              used;
	iwsn_addr_t       addr_16b;			//address of neighbor
	iwsn_addr_t       parentAddr_16b;	//address of the neighbor's parent（sink:FF）
	iwsn_addr_t       childAddr_16b;	//address of the neighbor's child（sink:FF）
	bool			  isHaveChild;		//whether have child
	int8_t            rssi;				//the RSSI of neighbor
	asn_t             asn;				//latest asn updated
	uint8_t           role;				//role
	uint8_t           level;             //level
	uint8_t           relationship;		//relationship
	slotOffset_t      dedicatedSlot;	//the dedicated slot allocated to this neighbor
}__attribute__ ((packed)) neighborRow_t;
#pragma pack()

typedef struct {
	neighborRow_t     neighbors[MAXNUMNEIGHBORS];
	iwsn_addr_t       managerAddr;
	iwsn_addr_t       sinkAddr;         //address of my sink
	iwsn_addr_t       myParentAddr_16b;	//address of my parent
	iwsn_addr_t       myChildAddr_16b;  //address of my child
} neighbors_vars_t;

typedef enum {
  NEIGHBOR_ROLE			= 0,
  NEIGHBOR_LEVEL		= 1,
  NEIGHBOR_PRIORITY		= 2,
  NEIGHBOR_RELATIONSHIP		= 3,
  NEIGHBOR_HAVELEVELTHREECHILD	= 4,	//rev.2
  NEIGHBOR_CHILDSLOTOFFSET  = 5,
} neighborProperty_t;

typedef struct {
  neighborProperty_t            name;
  uint8_t                       value;
} neighborProperties_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void          neighbors_init();
void          neighbors_indicateRx(iwsn_addr_t* l2_src,
                                   int8_t       rssi,
                                   asn_t*       asnTimestamp);
void          neighbors_indicateTx(iwsn_addr_t* l2_src, asn_t* asnTimestamp);
void 		  neighbors_nodeCheckChildTimeout();
void 		  neighbors_sinkCheckChild();
iwsn_addr_t*  neighbors_getChildAddr(iwsn_addr_type_t type);
iwsn_addr_t*  neighbors_getTimeoutPeerNeighbor(iwsn_addr_type_t type);
bool          neighbors_isNeighbor(iwsn_addr_t* neighbor);
bool          neighbors_isPreferredParent(iwsn_addr_t* neighbor); //v3
bool 		  neighbors_isParentSelected(); //v3
bool 		  neighbors_isParentSink();  //v3
bool 		  neighbors_isSink(iwsn_addr_t* address);  //v3
void          neighbors_updateNeighborProperties(iwsn_addr_t* neighbor,
                                                 uint8_t num,
                                                 neighborProperties_t properties[]);
void          neighbors_updateNeighborProperty(iwsn_addr_t* neighbor,
                                               neighborProperty_t property,
                                               void* value);
void          neighbors_getNeighborProperty(iwsn_addr_t* neighbor,
                                            neighborProperty_t property,
                                            uint8_t* buf);
void 		  neighbors_updateNeighborFromadv(iwsn_addr_t* neighbor,
											  uint8_t role,
											  iwsn_addr_t* parentAddr,
											  iwsn_addr_t* childAddr,
											  int8_t rssi,
											  slotOffset_t dedicatedSlot);  //v3
void 		  neighbors_sinkUpdateNeighbors(iwsn_addr_t* neighbor,
											  uint8_t role,
											  iwsn_addr_t* parentAddr,
											  iwsn_addr_t* childAddr,
											  int8_t rssi,
											  slotOffset_t dedicatedSlot,
											  slotOffset_t childSlot);  //v3
void 		  neighbors_updateMySinkAddr(iwsn_addr_t* sinkAddr);            //v3
void 		  neighbors_resetVars();   //v3
iwsn_addr_t*  neighbors_getSinkAddress();       //v3
iwsn_addr_t*  neighbors_getMyParentAddress();   //v3
iwsn_addr_t*  neighbors_getMyChildAddress();    //v3
iwsn_addr_t*  neighbors_getParentNeighbor();
iwsn_addr_t*  neighbors_selectParentNeighbor();
iwsn_addr_t*  neighbors_getManagerAddr();
void 		  neighbors_selectParent();  //V3
void          neighbors_removeNeighbor(iwsn_addr_t* neighbor);
iwsn_addr_t*  neighbors_sinkFindNextHop(iwsn_addr_t* destination);  //v3

#endif
