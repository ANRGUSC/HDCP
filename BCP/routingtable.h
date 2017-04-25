// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#ifndef __ROUTINGTABLE_H__
#define __ROUTINGTABLE_H__

#include <stdbool.h>
#include <string.h>
#include "lib/list.h"
#include "lib/memb.h"
#include "net/rime.h"

#define ROUTING_TABLE_SIZE  0x24  // Max Neighbor Count
#define LINK_LOSS_ALPHA   90  // Decay parameter. 90 = 90% weight of previous link loss Estimate
#define LINK_LOSS_V       2  // V Value used to weight link losses in Lyapunov Calculation
#define LINK_EST_ALPHA    9   // Decay parameter. 9 = 90% weight of previous rate Estimation
#define RT_SUCCESS 0
#define RT_FAILURE  (!(RT_SUCCESS))
#define MAX_LIFE 500
struct routingtable {
  list_t *list;
  struct memb *memb;
};

struct routingtable_item {
  struct routingtable_item *next;
  uint16_t backpressure;
  uint16_t link_packet_tx_time; // Exponential moving average in 100US units
  //uint16_t link_etx; // Exponential moving average of ETX (in 100ths of expected transmissions)
  uint16_t lifetime; // Used to detect bursts of 3 successful receptions from a neighbor
  //uint16_t fail_count; // Used to detect bursts of 3 successful receptions from a neighbor
  uint32_t success_to_fail;
  uint32_t fail_to_success;  //Indicates the number of state 0 to state 1 transitions
  uint32_t fail_to_fail;
  uint32_t success_to_success;
  bool last_state;  //Indicated the last state of the channel: Good or Bad
  bool is_bursty_now; // Indicates whether the neighbor was used before in tranmitting the packet in the buffer
  //uint16_t used;
  rimeaddr_t neighbor;
};

/**
 * [routingtable_init description]
 * @param t [description]
 */
void routingtable_init(struct routingtable *t);
/**
 * [routingtable_update_link_success description]
 * @param t        [description]
 * @param neighbor [description]
 * @param tx_count [description]
 */
void routingtable_update_link_success(struct routingtable *t,
                                      const rimeaddr_t *neighbor,
                                      uint16_t tx_count);
/**
 * [routingtable_update_link_rate description]
 * @param t                   [description]
 * @param neighbor            [description]
 * @param link_packet_tx_time [description]
 */
void routingtable_update_link_rate(struct routingtable *t,
                                   const rimeaddr_t *neighbor,
                                   uint16_t link_packet_tx_time);
/**
 * [routing_table_update_entry description]
 * @param  t                [description]
 * @param  neighbor         [description]
 * @param  rcv_backpressure [description]
 * @return                  [description]
 */
int routing_table_update_entry(struct routingtable *t,
                               const rimeaddr_t *neighbor,
                               uint16_t rcv_backpressure);
/**
 * [routingtable_update_link_status description]
 * @param t        [description]
 * @param neighbor [description]
 * @param success  [description]
 */
void routingtable_update_link_status(struct routingtable *t,const rimeaddr_t *neighbor,bool success);
/**
 * [routingtable_len description]
 * @param  t [description]
 * @return   [description]
 */
int routingtable_len(struct routingtable *t);
/**
 * [routingtable_update_routing description]
 * @param  t                [description]
 * @param  rcv_backpressure [description]
 * @param  act_neighbor     [description]
 * @param  act_neighbor_bp  [description]
 * @param  tx               [description]
 * @param  prev_etx         [description]
 * @return                  [description]
 */
int routingtable_update_routing(struct routingtable *t,
                                uint16_t rcv_backpressure,
                                rimeaddr_t *act_neighbor,
                                uint32_t *act_neighbor_bp,
                                uint16_t tx,
                                uint32_t *prev_etx);

#endif /* __ROUTINGTABLE_H__ */

/** @} */
/** @} */
