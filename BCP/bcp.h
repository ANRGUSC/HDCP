// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details


#ifndef __BCP_H__
#define __BCP_H__

#include <stdbool.h>

#include "net/rime/unicast.h"
#include "net/rime/broadcast.h"
#include "lib/list.h"

#include "packetstack.h"
#include "routingtable.h"

#define BCP_ATTRIBUTES  { PACKETBUF_ADDR_ERECEIVER,     PACKETBUF_ADDRSIZE }, \
            { PACKETBUF_ATTR_PACKET_ID,   PACKETBUF_ATTR_BIT * 16 }, \
            { PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_BIT * 3 }, \
                            BROADCAST_ATTRIBUTES

#define PACKETBUF_ATTR_PACKET_TYPE_BEACON    5

// Forward declaration
struct bcp_conn;

struct bcp_callbacks {
  void (*recv)(struct bcp_conn *c,struct data_hdr hdr);
};

struct bcp_conn {

  // Broadcast connection for data packets and beacons
  struct broadcast_conn broadcast_conn;

  // Unicast connection for ACKS
  struct unicast_conn unicast_conn;

  // Timer for retransmissions
  struct ctimer retransmission_timer;

  // Timer for triggering a send data packet task
  struct ctimer send_timer;

  // Timer for beaconing
  struct ctimer beacon_timer;

  // Timer for link estimation
  struct timer delay_timer;

  LIST_STRUCT(send_stack_list);
  struct packetstack send_stack;

  // Routing table of neighbors
  LIST_STRUCT(routing_table_list);
  struct routingtable routing_table;

  // Callbacks for the end user
  const struct bcp_callbacks *cb;

  // The packet currently being sent
  struct packetstack_item *current_packet;
  struct packetstack_item *packet_not_acked;

  uint16_t tx_count;
  uint16_t virtual_queue_size;
  uint32_t prev_etx;
  bool sending;
  bool is_sink;

  uint32_t active_neighbor_backpressure;
  rimeaddr_t active_neighbor;   // Current active neighbor we are forwarding to

  // Time the current packet was sent
  clock_time_t send_time;
};

void bcp_open(struct bcp_conn *c, uint16_t channel,
              const struct bcp_callbacks *callbacks);

void bcp_close(struct bcp_conn *c);

int bcp_send(struct bcp_conn *c);

void bcp_set_sink(struct bcp_conn *c, const rimeaddr_t *addr);

#endif /* __BCP_H__ */
/** @} */
/** @} */
