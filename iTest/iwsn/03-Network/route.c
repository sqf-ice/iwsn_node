/**
\brief General IWSN 03-Network 'route' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "route.h"
#include "neighbors.h"
#include "idmanager.h"
#include "network.h"
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================



//=========================== prototypes ======================================



//=========================== main ============================================

//=========================== public ==========================================


void route_findNexthopForMsg(QueueEntry_t* msg)
{
  iwsn_addr_t* neighbor;
  uint8_t      myRole = idmanager_getMyRole();
  
  //there is difference between sink and normal node
  if (idmanager_isGateway(myRole) == true)
  {
	  //judge the type of msg
	  switch(msg->l3_pduSpecifier)
	  {
	  case NET_TYPE_CAN_SAMPLE:
	  case NET_TYPE_DATA_AGGREGATE:
		  //send to uplayer,  my parent address is manager
		  neighbor = neighbors_getManagerAddr();
		  break;
	  case NET_TYPE_CMD_CONTROL:
	  case NET_TYPE_NODE_NOTIFY:
		  neighbor = neighbors_sinkFindNextHop(&msg->l3_destinationORsource);
		  break;
	  default:
		  //default if the destination is my neighbor, i will send whatever (problem)
		  if (neighbors_isNeighbor(&msg->l3_destinationORsource) == true)
		  {
			  neighbor = &msg->l3_destinationORsource;
		  }
		  break;
	  }
  }else{
	  //judge the type of msg
	  switch(msg->l3_pduSpecifier)
	  {
	  case NET_TYPE_CAN_SAMPLE:
	  case NET_TYPE_DATA_AGGREGATE:
		  //send to uplayer, find my parent address
		  neighbor = neighbors_getMyParentAddress();
		  break;
	  case NET_TYPE_CMD_CONTROL:
		  //send to downlayer, find my child address
		  neighbor = neighbors_getMyChildAddress();
		  break;
	  default:
		  //default if the destination is my neighbor, i will send whatever (problem)
		  if (neighbors_isNeighbor(&msg->l3_destinationORsource) == true)
		  {
			  neighbor = &msg->l3_destinationORsource;
		  }
		  break;
	  }
  }
  if(neighbor != NULL)
  {
	  memcpy(&msg->l2_nextORpreviousHop, neighbor, sizeof(iwsn_addr_t));
  }else{
	  memcpy(&msg->l2_nextORpreviousHop, &msg->l3_destinationORsource, sizeof(iwsn_addr_t));
  }

}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
