/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh, 
 * Bhaskar Krishnamachari 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

/**
 * @file        bcp.h
 * @brief       Main library for BCP routing
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 *   
 */

#ifndef __bcp_H__
#define __bcp_H__

#include <stdbool.h>

#include "net/rime/unicast.h"
#include "net/rime/broadcast.h"
#include "lib/list.h"

#include "packetstack.h"
#include "routingtable.h"

#define bcp_ATTRIBUTES  { PACKETBUF_ADDR_ERECEIVER,     PACKETBUF_ADDRSIZE }, \
            { PACKETBUF_ATTR_PACKET_ID,   PACKETBUF_ATTR_BIT * 16 }, \
            { PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_BIT * 3 }, \
            { PACKETBUF_ATTR_MAX_REXMIT,  PACKETBUF_ATTR_BIT * 5 }, \
            { PACKETBUF_ATTR_NUM_REXMIT, PACKETBUF_ATTR_BIT * 5},\
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
  uint8_t max_rexmits,transmissions;
  uint16_t virtual_queue_size;
  uint32_t prev_etx;
  bool sending;
  bool is_sink;

  uint32_t active_neighbor_backpressure;
  rimeaddr_t active_neighbor;   // Current active neighbor we are forwarding to

  // Time the current packet was sent
  clock_time_t send_time;
};

/**
 * @brief open a new bcp connection
 * @param c         pointer to the connection structure
 * @param channel   the channel number
 * @param callbacks callback functions
 */
void bcp_open(struct bcp_conn *c, uint16_t channel,
              const struct bcp_callbacks *callbacks);

/**
 * @brief close the bcp connections
 * @param c [pointer to the connection structure]
 */
void bcp_close(struct bcp_conn *c);

/**
 * Send a bcp packet
 * @param  c [pointer to the connection structure]
 * @return   success status
 */
int bcp_send(struct bcp_conn *c);

/**
 * @brief if the node's address is @p addr, set the node as a sink node
 * @param c    [pointer to the connection structure]
 * @param addr the address of the sink node
 */
void bcp_set_sink(struct bcp_conn *c, const rimeaddr_t *addr);

#endif /* __bcp_H__ */
/** @} */
/** @} */
