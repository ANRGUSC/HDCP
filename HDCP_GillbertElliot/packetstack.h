// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#ifndef __PACKETSTACK_H__
#define __PACKETSTACK_H__

#include "lib/list.h"
#include "lib/memb.h"

#include "net/packetbuf.h"
#include "net/queuebuf.h"

struct packetstack 
{
  list_t *list;
  struct memb *memb;
};

struct data_hdr 
{
  //uint32_t path;
  //uint32_t bcp_delay;			// Delay experienced by this packet
  uint16_t bcp_backpressure;	// Node backpressure measurement for neighbors
  uint16_t tx_count;			// Total transmission count experienced by the packet
  uint32_t timestamp;			//timestamp
  uint16_t delay;                       //delay
  rimeaddr_t origin;
  uint16_t hop_count;			// End-to-end hop count experienced by the packet
  uint16_t origin_seq_no;
  //uint16_t pkt_type;				// PKT_NORMAL | PKT_NULL
  //uint16_t node_tx_count;// Incremented every tx by this node, to determine bursts for STLE
  //rimeaddr_t burst_notify_addr;	// In the event of a burst link available detect, set neighbor addr, else set self addr
  // uint16_t nh_backpressure; // Next hop Backpressure, used by STLE, overheard by neighbors
};

struct packetstack_item 
{
  struct packetstack_item *next;
  struct queuebuf *buf;
  struct packetstack *stack;
  void *ptr;

  // BCP Header
  struct data_hdr hdr;
};

#define PACKETSTACK(name, size) LIST(name##_list); \
                                MEMB(name##_memb, struct packetstack_item, size); \
				static struct packetstack name = { &name##_list, \
								   &name##_memb }

void packetstack_init(struct packetstack *s);

struct packetstack_item *packetstack_top(struct packetstack *s);
struct packetstack_item *packetstack_bottom(struct packetstack *s);
struct packetstack_item *packetstack_push_packetbuf(struct packetstack *s,
                                                    void *ptr, uint16_t *p);

void packetstack_pop(struct packetstack *s);

void packetstack_remove(struct packetstack *s, struct packetstack_item *i);

int packetstack_len(struct packetstack *s);

struct queuebuf *packetstack_queuebuf(struct packetstack_item *i);

#endif /* __PACKETSTACK_H__ */

/** @} */
/** @} */
