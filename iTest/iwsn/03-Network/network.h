/**
\brief General IWSN 03-Network 'network' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __NETWORK_H
#define __NETWORK_H

//=========================== include =========================================

#include "iwsn.h"
#include "queue.h"

//=========================== define ==========================================

#define NETWORK_MAX_TTL     16
#define MAX_PACKET_TIME   1000

enum Net_send_type_enums {
  NET_SEND_UPPER_TYPE     = 0,
  NET_SEND_LOWER_TYPE	  = 1,
  NET_SEND_FORWARD_TYPE   = 2,
  NET_SEND_PROXY_TYPE     = 3,
};

enum Net_ctl_enums {
  NET_CTL_FISRT_ROUTE     = 0,
  NET_CTL_SECOND_ROUTE    = 1,
  NET_CTL_PROXY_ROUTE     = 2,
  NET_CTL_SRC_ADDR_MODE   = 6,
  NET_CTL_DST_ADDR_MODE   = 7,
};

enum Net_ctl_route_enums {
  NET_ROUTE_NO_ROUTE      = 0,
  NET_ROUTE_YES_ROUTE     = 1,
};

enum Net_ctl_addr_mode_enums {
  NET_ADDR_MODE_SHORT     = 0,
  NET_ADDR_MODE_EXT       = 1,
};

enum Net_pdu_specifier_enums {
  NET_TYPE_NODE_NOTIFY      = 0,
  NET_TYPE_MONITOR_STATUS   = 1,
  NET_TYPE_CAN_SAMPLE       = 0x32,
  NET_TYPE_DATA_AGGREGATE   = 0x20,
  NET_TYPE_TRACE_SAMPLE     = 21,
  NET_TYPE_CMD_CONTROL      = 8,
  NET_TYPE_USB_CONNECT		= 20,
};

enum cmd_control_enums {
  CMD_CONTROL_UNICAST       = 0,
  CMD_CONTROL_MULTICAST     = 1,
  CMD_CONTROL_BROADCAST     = 2,
};

//=========================== typedef =========================================

typedef struct {
  uint8_t     firstRouteExisted   :1,
              secondRouteExisted  :1,
              proxyRouteExisted   :1,
              reserved            :3,
              srcAddrMode         :1,
              dstAddrMode         :1;
  uint8_t     ttl;
  uint16_t    seq;
  uint16_t    graphId;
  iwsn_addr_t dst;
  iwsn_addr_t src;
  uint8_t     pdu_specifier;
  iwsn_addr_t proxy_route;
  iwsn_addr_t first_route;
  iwsn_addr_t second_route;
  uint8_t     header_length;
} net_header_iht;

//=========================== variables =======================================

//=========================== prototypes ======================================

void    net_init();
error_t net_send(QueueEntry_t *msg, uint8_t type, net_header_iht* net_header);
void    net_sendDone(QueueEntry_t* msg, error_t error);
void    net_receive(QueueEntry_t* msg);

error_t net_forward(QueueEntry_t* msg, net_header_iht net_header);

error_t net_prependNetHeader(QueueEntry_t*  msg);
error_t net_prependNetHeaderForForward(QueueEntry_t*  msg, net_header_iht net_header);

#endif
