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
  uint16_t link_etx; // Exponential moving average of ETX (in 100ths of expected transmissions)
  uint16_t lifetime; // Used to detect bursts of 3 successful receptions from a neighbor
  //uint16_t fail_count; // Used to detect bursts of 3 successful receptions from a neighbor
  // uint32_t success_to_fail;
  // uint32_t fail_to_success;  //Indicates the number of state 0 to state 1 transitions
  // uint32_t fail_to_fail;
  // uint32_t success_to_success;
  uint8_t last_count;  //Indicated the last state of the channel: Good or Bad
  bool is_bursty_now; // Indicates whether the neighbor was used before in tranmitting the packet in the buffer
  //uint16_t used;
  rimeaddr_t neighbor;
};

/**
 * @brief       initializes the routing table
 * @param       pointer to the table
 */
void routingtable_init(struct routingtable *t);

/**
 * @brief             update the link ETX
 * @param t           pointer to the routing table
 * @param neighbor    the other end of the link
 * @param tx_count    the last retransmission count
 */
void routingtable_update_link_success(struct routingtable *t,
                                      const rimeaddr_t *neighbor,
                                      uint16_t tx_count);

/**
 * @brief  Update the queue occupancy information of the neighbor
 * @param  t                pointer to the routing table
 * @param  neighbor         the neighbor id to be updated
 * @param  rcv_backpressure the queue length of the neighbor
 * @return                  status
 */
int routing_table_update_entry(struct routingtable *t,
                               const rimeaddr_t *neighbor,
                               uint16_t rcv_backpressure);


/**
 * @brief  returns the current number of neighbors
 * @param  t pointer to the routing table
 * @return   the number of neighbors
 */
int routingtable_len(struct routingtable *t);


/**
 * Updates the link weights and calculates the best neighbor to forward packets
 * @param  t                   pointer to the routing table
 * @param  localBackpressure_p the local queu occupancy
 * @param  act_neighbor        returns the best neighbors addess
 * @param  act_neighbor_bp     returns the backpressure of the best neighbor
 * @param  tx                  number of transmissions
 * @param  prev_etx            the ETX of the previously used link in case of switching
 * @return                     success status
 */
int routingtable_update_routing(struct routingtable *t,
                                uint16_t localBackpressure_p,
                                rimeaddr_t *act_neighbor,
                                uint32_t *act_neighbor_bp,
                                uint16_t tx,
                                uint32_t *prev_etx);

#endif /* __ROUTINGTABLE_H__ */

/** @} */
/** @} */
