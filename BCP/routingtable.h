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
 * @file        routingtable.h
 * @brief       Helper library for HDCP routing
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 * 
 */

#ifndef __ROUTINGTABLE_H__
#define __ROUTINGTABLE_H__

#include <stdbool.h>
#include <string.h>
#include "lib/list.h"
#include "lib/memb.h"
#include "net/rime.h"

#define ROUTING_TABLE_SIZE      0x24  // Max Neighbor Count
#define LINK_LOSS_ALPHA         90  // Decay parameter. 90 = 90% weight of previous link loss Estimate
#define LINK_LOSS_V             2  // V Value used to weight link losses in Lyapunov Calculation
#define LINK_EST_ALPHA          9   // Decay parameter. 9 = 90% weight of previous rate Estimation
#define RT_SUCCESS              0
#define RT_FAILURE              (!(RT_SUCCESS))
#define MAX_LIFE                500

struct routingtable {
  list_t *list;
  struct memb *memb;
};

struct routingtable_item {
  struct routingtable_item    *next;
  uint16_t                    backpressure;
  uint16_t                    link_packet_tx_time; // Exponential moving average in 100US units
  //uint16_t                  link_etx; // Exponential moving average of ETX (in 100ths of expected transmissions)
  uint16_t                    lifetime; // Used to detect bursts of 3 successful receptions from a neighbor
  //uint16_t                  fail_count; // Used to detect bursts of 3 successful receptions from a neighbor
  uint32_t                    success_to_fail;
  uint32_t                    fail_to_success;  //Indicates the number of state 0 to state 1 transitions
  uint32_t                    fail_to_fail;
  uint32_t                    success_to_success;
  bool                        last_state;  //Indicated the last state of the channel: Good or Bad
  bool                        is_bursty_now; // Indicates whether the neighbor was used before in tranmitting the packet in the buffer
  //uint16_t                   used;
  rimeaddr_t                  neighbor;
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
 * @brief  Update the link rate
 * @param t                   pointer to the routing table
 * @param neighbor            the other end of the link
 * @param link_packet_tx_time the transmission time
 */
void routingtable_update_link_rate(struct routingtable *t,
                                   const rimeaddr_t *neighbor,
                                   uint16_t link_packet_tx_time);
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
 * Updates the Gillbert Elliot MC model 
 * @param  t                pointer to the routing table
 * @param  neighbor         the neighbor id to be updated
 * @param  success          whether the packet transmission was success
 */
void routingtable_update_link_status( struct routingtable *t,
                                      const rimeaddr_t *neighbor,
                                      bool success);
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
                                uint16_t rcv_backpressure,
                                rimeaddr_t *act_neighbor,
                                uint32_t *act_neighbor_bp,
                                uint16_t tx,
                                uint32_t *prev_etx);

#endif /* __ROUTINGTABLE_H__ */

/** @} */
/** @} */
