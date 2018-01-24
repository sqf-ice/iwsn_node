/**
\brief General IWSN 03-Network 'network' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
#ifdef HAVE_CONFIG
#include "config.h"    //this is set to the HAVE_AGGREGATE
#endif

#include "network.h"
#include "idmanager.h"
#include "res.h"
#include "packet.h"
#include "route.h"
#include "neighbors.h"
#include "monitor.h"
#include "forward.h"
#include "aggregate.h"
#include "cmd.h"
#include "queue.h"
//#include "applications.h"

#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#ifdef HAVE_USB
#include "usbAPI.h"
#endif

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

error_t prependNetHeader(QueueEntry_t*  msg,
                         bool           dam,
                         bool           sam,
                         bool           pr,
                         bool           fsr,
                         bool           ssr,
                         uint8_t        ttl,
                         uint16_t       seq,
                         uint16_t       graphId,
                         iwsn_addr_t*   src,                         
                         iwsn_addr_t*   dst,
                         iwsn_addr_t*   proxy,
                         iwsn_addr_t*   second,
                         iwsn_addr_t*   first);
void    retrieveNetHeader(QueueEntry_t* msg, net_header_iht *net_header);
bool    isValidPacket(net_header_iht *net_header);
bool    isTimeoutSeq(uint16_t seq);

//=========================== main ============================================

//=========================== public ==========================================

void net_init()
{
}

error_t net_send(QueueEntry_t* msg, uint8_t type, net_header_iht* net_header)
{
  asn_t*         asn;
  uint8_t        ttl;
  uint16_t       seq;
  uint16_t       graphId;
  
  iwsn_addr_t*   src;  
  
  bool           pr = false;
  iwsn_addr_t*   proxy;
  iwsn_addr_t*   second;
  iwsn_addr_t*   first;
  
  uint8_t srCnt   = msg->payload[2];
                           
  msg->owner = COMPONENT_NETWORK; 
  
  if (type == NET_SEND_UPPER_TYPE)
  {
    ttl = NETWORK_MAX_TTL;
    asn = ieee802154e_getCurASN();
    seq = asn->bytes0and1;
    graphId = 0;  // temp
    src = idmanager_getMyID(ADDR_16B);
    second = NULL;
    first = NULL;
    
    route_findNexthopForMsg(msg); 
  }
  else if(type == NET_SEND_LOWER_TYPE)		//rev.2
  {
	ttl = NETWORK_MAX_TTL;
	asn = ieee802154e_getCurASN();
	seq = asn->bytes0and1;
	graphId = 0;  // temp
	src = idmanager_getMyID(ADDR_16B);
	proxy = NULL;
	second = NULL;
	first = NULL;

	route_findNexthopForMsg(msg);
  }
  else if (type == NET_SEND_FORWARD_TYPE)
  {
    ttl     = net_header->ttl - 1;
    seq     = net_header->seq;
    graphId = net_header->graphId;
    src     = &net_header->src;
    proxy = NULL;
    second = NULL;
    first = NULL;
    
    if (srCnt > 0)
    {
      msg->l3_destinationORsource.type = ADDR_16B;
      memcpy(&msg->l3_destinationORsource.addr_16b[0],  &msg->payload[3], 2);
      route_findNexthopForMsg(msg);
      memcpy(&msg->l3_destinationORsource, &net_header->dst, sizeof(iwsn_addr_t));
    }
    else
    {
      route_findNexthopForMsg(msg);
    }
  }
  
  if (prependNetHeader(msg,
                       NET_ADDR_MODE_SHORT, 
                       NET_ADDR_MODE_SHORT,
                       pr,
                       NET_ROUTE_NO_ROUTE,
                       NET_ROUTE_NO_ROUTE,
                       ttl,
                       seq,
                       graphId,
                       src,
                       &msg->l3_destinationORsource,
                       proxy,
                       second,
                       first) == E_FAIL)
  {
    return E_FAIL;
  }
  
#ifdef HAVE_AGGREGATE  
  if (msg->l3_pduSpecifier == NET_TYPE_DATA_AGGREGATE)
  {
    return aggregate_send(msg);
  }
#endif
  return res_send(msg);
}

void net_sendDone(QueueEntry_t* msg, error_t error)
{
  msg->owner = COMPONENT_NETWORK;

  switch (msg->creator)
  {
	case COMPONENT_NOTIFICATION:
		queue_freePacketBuffer(msg);
	break;
	default:
		queue_freePacketBuffer(msg);
	break;
  }
}

void net_receive(QueueEntry_t* msg)
{
  net_header_iht net_header;
  uint8_t        myRole = idmanager_getMyRole();
  
  msg->owner = COMPONENT_NETWORK;
  retrieveNetHeader(msg, &net_header);
  
  if (isValidPacket(&net_header) == false)
  {
    queue_freePacketBuffer(msg);
    return;
  }
  
  if (idmanager_isMyAddress(&net_header.dst) == true && net_header.proxyRouteExisted == 0)
  {
    packet_tossHeader(msg, net_header.header_length);
    switch (net_header.pdu_specifier)
    {
      case NET_TYPE_CMD_CONTROL:
        cmd_receive(msg);
        break;
      case NET_TYPE_NODE_NOTIFY:
        monitor_reveive(msg);
        break;
      default:
        queue_freePacketBuffer(msg);
        break;
    }
  }
  else
  {
    packet_tossHeader(msg, net_header.header_length);
    if (idmanager_isRouter(myRole) == true)
    {
      forward_receive(msg, net_header);
      return;
    }
    else
    {
      queue_freePacketBuffer(msg);
      return;
    }
  }
}

error_t net_forward(QueueEntry_t* msg, net_header_iht net_header)
{
  uint8_t       myRole  = idmanager_getMyRole();
  iwsn_addr_t*  myAddr  = idmanager_getMyID(ADDR_16B);
  iwsn_addr_t*  manager = neighbors_getManagerAddr();
  
  msg->owner = COMPONENT_FORWARD;
  route_findNexthopForMsg(msg);
  
  if (idmanager_isGateway(myRole) == true &&
      idmanager_isSameAddress(manager, &msg->l2_nextORpreviousHop) == true)
  {
    packet_tossHeader(msg, net_header.header_length);
    packet_reserveHeaderSize(msg, 2);
    memcpy(&(msg->payload[0]), &myAddr->addr_16b[0], 2);
#ifdef HAVE_AGGREGATE
    if (msg->l3_pduSpecifier == NET_TYPE_DATA_AGGREGATE)
    {
      packet_tossFooter(msg, aggregate_getLeftLength(msg));
    }
#endif
#ifdef HAVE_USB
#if SET_USB_FUNC == USB_ETHERNET
    usbeth_send( &(msg->payload[0]), msg->length);
#endif
#endif
    queue_freePacketBuffer(msg);
    return E_SUCCESS;
  }
  
#ifdef HAVE_AGGREGATE  
  if (msg->l3_pduSpecifier == NET_TYPE_DATA_AGGREGATE)
  {
    return aggregate_send(msg);
  }
#endif
  
  return res_send(msg);
}

error_t net_prependNetHeader(QueueEntry_t*  msg)
{
  bool pr = false, fsr = false, ssr = false;
  uint8_t srCnt = 0;
  asn_t* asn = ieee802154e_getCurASN();
  //iwsn_addr_t* src = idmanager_getMyID(ADDR_16B);
  iwsn_addr_t* src = neighbors_getManagerAddr();
  iwsn_addr_t dst;
  iwsn_addr_t proxy, second, first;
  uint8_t type;
  
  type = *(uint8_t *)(&msg->payload[0]);
//  subType = *(uint8_t *)(&msg->payload[1]);
  
  srCnt = *(uint8_t *)(&msg->payload[2]);
  dst.type = ADDR_16B;
  memcpy(&dst.addr_16b[0], &msg->payload[3 + srCnt * 2], 2);
  
  msg->l3_pduSpecifier = type;
  
  return prependNetHeader(msg, NET_ADDR_MODE_SHORT, NET_ADDR_MODE_SHORT,
                          pr,
                          fsr,
                          ssr,
                          NETWORK_MAX_TTL,
                          asn->bytes0and1,
                          0,
                          src,
                          &dst,
                          &proxy,
                          &second,
                          &first);
}

error_t net_prependNetHeaderForForward(QueueEntry_t*  msg, net_header_iht net_header)
{
  return prependNetHeader(msg,
                          net_header.dstAddrMode,
                          net_header.srcAddrMode,
                          net_header.proxyRouteExisted,
                          net_header.firstRouteExisted,
                          net_header.secondRouteExisted,
                          (net_header.ttl - 1),
                          net_header.seq,
                          net_header.graphId,
                          &net_header.src,
                          &net_header.dst,
                          &net_header.proxy_route,
                          &net_header.second_route,
                          &net_header.first_route);
}

//=========================== private =========================================

error_t prependNetHeader(QueueEntry_t*  msg,
                         bool           dam,
                         bool           sam,
                         bool           pr,
                         bool           fsr,
                         bool           ssr,
                         uint8_t        ttl,
                         uint16_t       seq,
                         uint16_t       graphId,
                         iwsn_addr_t*   src,                         
                         iwsn_addr_t*   dst,
                         iwsn_addr_t*   proxy,
                         iwsn_addr_t*   second,
                         iwsn_addr_t*   first)
{
  uint8_t temp_8b;
  msg->l3_payload = msg->payload;    // rev.2 for monitor in the res task_resNotifSendDone
  
  // first source route
  if (fsr == true)
  {
    packet_writeAddress(msg, first, LITTLE_ENDIAN);
  }
  
  // second source route
  if (ssr == true)
  {
    packet_writeAddress(msg, second, LITTLE_ENDIAN);
  }
  
  // proxy route
  if (pr == true)
  {
    packet_writeAddress(msg, proxy, LITTLE_ENDIAN);
  }
  
  // network PDU specifier
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = msg->l3_pduSpecifier;
  
  // source address
  packet_writeAddress(msg, src, LITTLE_ENDIAN);
  
  // destination address
  packet_writeAddress(msg, dst, LITTLE_ENDIAN);
  
  // graph ID
  packet_reserveHeaderSize(msg, sizeof(uint16_t));
  memcpy((uint8_t *)(msg->payload), &graphId, sizeof(uint16_t));
  
  // packet seq
  packet_reserveHeaderSize(msg, sizeof(uint16_t));
  memcpy((uint8_t *)(msg->payload), &seq, sizeof(uint16_t));
  
  // TTL
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = ttl;
  
  // control
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  temp_8b  = 0;
  temp_8b |= fsr ? (1 << NET_CTL_FISRT_ROUTE) : 0;
  temp_8b |= ssr ? (1 << NET_CTL_SECOND_ROUTE) : 0;
  temp_8b |= pr ? (1 << NET_CTL_PROXY_ROUTE) : 0;
  temp_8b |= sam ? (1 << NET_CTL_SRC_ADDR_MODE) : 0;
  temp_8b |= dam ? (1 << NET_CTL_DST_ADDR_MODE) : 0;
  *((uint8_t*)(msg->payload)) = temp_8b;
  
  return E_SUCCESS;
}

void retrieveNetHeader(QueueEntry_t* msg, net_header_iht *net_header)
{
  uint8_t         temp_8b;
  uint8_t         temp[2];
  
  net_header->header_length = 0;
  // network control
  if (net_header->header_length > msg->length)
  {
    return;
  }
  temp_8b = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->dstAddrMode         = (temp_8b >> NET_CTL_DST_ADDR_MODE) & 0x01;
  net_header->srcAddrMode         = (temp_8b >> NET_CTL_SRC_ADDR_MODE) & 0x01;
  net_header->proxyRouteExisted   = (temp_8b >> NET_CTL_PROXY_ROUTE)   & 0x01;
  net_header->secondRouteExisted  = (temp_8b >> NET_CTL_SECOND_ROUTE)  & 0x01;
  net_header->firstRouteExisted   = (temp_8b >> NET_CTL_FISRT_ROUTE)   & 0x01;
  
  if (net_header->dstAddrMode == 0)
  {
    net_header->dst.type = ADDR_16B;
  }
  else
  {
    net_header->dst.type = ADDR_64B;
  }
  
  if (net_header->srcAddrMode == 0)
  {
    net_header->src.type = ADDR_16B;
  }
  else
  {
    net_header->src.type = ADDR_64B;
  }
  net_header->header_length += 1;
  
  // network ttl
  if (net_header->header_length > msg->length)
  {
    return;
  }
  net_header->ttl = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  
  // network seq
  if (net_header->header_length > msg->length)
  {
    return;
  }
  temp[0] = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  temp[1] = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  net_header->seq = temp[1] * 256 + temp[0];
  
  // network graph ID
  if (net_header->header_length > msg->length)
  {
    return;
  }
  temp[0] = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  temp[1] = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  net_header->graphId = temp[1] * 256 + temp[0];
  
  // network dst address
  if (net_header->header_length > msg->length)
  {
    return;
  }
  if (net_header->dst.type == ADDR_16B)
  {
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_16B,
                       &net_header->dst,
                       LITTLE_ENDIAN);
    net_header->header_length += 2;
  }
  else
  {
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_64B,
                       &net_header->dst,
                       LITTLE_ENDIAN);
    net_header->header_length += 8;
  }
  
  // network src address
  if (net_header->header_length > msg->length)
  {
    return;
  }
  if (net_header->src.type == ADDR_16B)
  {
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_16B,
                       &net_header->src,
                       LITTLE_ENDIAN);
    net_header->header_length += 2;
  }
  else
  {
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_64B,
                       &net_header->src,
                       LITTLE_ENDIAN);
    net_header->header_length += 8;
  }
  
  // network PDU specifier
  if (net_header->header_length > msg->length)
  {
    return;
  }
  net_header->pdu_specifier  = *((uint8_t *)(msg->payload) + net_header->header_length);
  net_header->header_length += 1;
  
  // network proxy route
  if (net_header->header_length > msg->length)
  {
    return;
  }
  if (net_header->proxyRouteExisted != 0)
  {
    net_header->proxy_route.type = ADDR_16B;
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_16B,
                       &net_header->proxy_route,
                       LITTLE_ENDIAN);
    net_header->header_length += 2;
  }
  
  // network second src route
  if (net_header->header_length > msg->length)
  {
    return;
  }
  if (net_header->secondRouteExisted != 0)
  {
    net_header->second_route.type = ADDR_16B;
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_16B,
                       &net_header->second_route,
                       LITTLE_ENDIAN);
    net_header->header_length += 2;
  }
  
  // network first src route
  if (net_header->header_length > msg->length)
  {
    return;
  }
  if (net_header->firstRouteExisted != 0)
  {
    net_header->first_route.type = ADDR_16B;
    packet_readAddress(((uint8_t*)(msg->payload) + net_header->header_length),
                       ADDR_16B,
                       &net_header->first_route,
                       LITTLE_ENDIAN);
    net_header->header_length += 2;
  }  
}

port_INLINE bool isValidPacket(net_header_iht *net_header)
{
  if (net_header->ttl == 0 || isTimeoutSeq(net_header->seq) == true)
  {
    return false;
  }
  
  return true;
}

port_INLINE bool isTimeoutSeq(uint16_t seq)
{
  asn_t* asn = ieee802154e_getCurASN();
  
  if ((uint16_t)(asn->bytes0and1 - seq) > MAX_PACKET_TIME)
  {
    return true;
  }
  
  return false;
}

//=========================== interrupt handlers ==============================
