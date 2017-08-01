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
 * @file        routingtable.c
 * @brief       Helper library for HDCP routing
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 * 
 */

#include "routingtable.h"

#define DEBUG         0
#define OWNDEBUG      0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTF_RTABLE(X) dbg_print_rtable(X)
#else
#define PRINTF(...)
#define PRINTF_RTABLE(Y)
#endif
#if OWNDEBUG
  #include <stdio.h>
  #define PRINTF_LVL2(...) printf(__VA_ARGS__)
#else
  #define PRINTF_LVL2(...)
#endif

#define SWITCHING     0
#define ORIGINAL      1       // Original HD algorithm without modifications
#define COUNTTH       100
#define ETX_THSLD     800
#define queue_filter  0

static uint16_t flag_g = 1; /* This flag is used for link switching */

// Forward Declarations
void dbg_print_rtitem(struct routingtable_item *rtitem);
void dbg_print_rtable(struct routingtable *t);

/**
 * @brief       initializes the routing table
 * @param       pointer to the table
 */
void routingtable_init(struct routingtable *t)
{
  list_init(*t->list);
  memb_init(t->memb);
}
/*---------------------------------------------------------------------------*/
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
                                 uint32_t *prev_etx)
{
  uint16_t         compareIdx           = 0;
  int32_t          compareWeight        = 0;
  int32_t          maxWeight            = -1;
  int32_t          maxWeight1           = -1;
  uint16_t         NeighborBackpressure = 0;
  uint16_t         link_ETX             = 0;
  uint16_t         etx_best             = 0;
  uint16_t         phi                  = 0;
  uint16_t         phi1                 = 0;
  int16_t          queue_differ         = 0;
  int16_t          f_ij                 = 0;
  static uint16_t  mu_ij                = 100;
  uint16_t         beta                 = 100;
 
  struct routingtable_item *i;
  struct routingtable_item *maxWeightPtr = NULL;
  
  if (routingtable_len(t) == 0) 
  {
    PRINTF(" routingtable: however, routingtable: routingtable_len returned: %d - there are no neighbors\n", routingtable_len(t));// There are no neighbors
    return RT_FAILURE;
  }

  for (i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {
    if (tx == 1){
      i->is_bursty_now = false; // On the first transmission set all neighbors to be available
      flag_g = 1;
    }

    if (flag_g == 2){
      i->is_bursty_now=false; // When no good neighbor is available with switching, set all neighbors to be available again
    }

    #if SWITCHING
      if (i->is_bursty_now){ //If a neighbor is previously tried in transmitting this packet, skip that
        PRINTF("routingtable: Moving to next neighbor from %d:%d\n",i->neighbor.u8[0],i->neighbor.u8[1]);
        if(i->last_count > 1)
          continue;
      }
    #endif /* SWITCHING */

    if (i->lifetime == 0)
      continue;
   
    NeighborBackpressure = i->backpressure;

    //---------------Calculate the ETX based on the Markov chain transitions-----------------//
    if(i->last_state)
      link_ETX = LINK_LOSS_V * ((i->success_to_success + 
                                i->success_to_fail) * 100) / (i->success_to_success);
    else
      link_ETX = LINK_LOSS_V * ((i->fail_to_success +
                                i->fail_to_fail) * 100) / (i->fail_to_success);
        
    queue_differ = (int16_t) (localBackpressure_p - NeighborBackpressure); 
    phi = (100 - beta) + beta * 100 / link_ETX; // 100 is used for interger arithmetic as contiki doesn't allow floats

    if (ORIGINAL)
    {
      if ((int16_t)queue_differ > 0){
        phi1 = phi * queue_differ;
        f_ij = (phi1 < mu_ij) ? mu_ij : phi1;
      }
      else
        f_ij = 0;
    }
    else
      f_ij = mu_ij;
    
    PRINTF_LVL2("routingtable: Phi = %d \n",phi);
    PRINTF_LVL2("routingtable: f_ij = %d \n",f_ij);

    //------Calculate the weight of the link------------------//
    if(ORIGINAL)
    {
      compareWeight = (2 * phi * queue_differ - f_ij);   
      if ( (int16_t)compareWeight <= 0 || f_ij == 0 )
        continue;
      compareWeight = (compareWeight * f_ij) / 100;
    }
    else
      compareWeight = 2*(phi* queue_differ)-(f_ij);

    PRINTF_LVL2("routingtable: ENTRY: ");
    PRINTF_LVL2("routingtable: ETX = %u ",link_ETX);
    PRINTF_LVL2("routingtable: Neighbor_BP = %u ",NeighborBackpressure);
    PRINTF_LVL2("routingtable: Local_BP = %u ",localBackpressure_p);
    PRINTF_LVL2("queue_diff = %d ",queue_diff);
    PRINTF_LVL2("routingtable: Weight = %ld ",compareWeight);
    PRINTF_LVL2("routingtable: Neighbor %d:%d \n",(i->neighbor).u8[0],(i->neighbor).u8[1]);

    if (queue_differ <= 0)
      continue;

    if (queue_filter && NeighborBackpressure >= 25)
      continue;

    i->lifetime = i->lifetime-1;

    if ((compareWeight > maxWeight1))//&&link_ETX<ETX_THSLD)//&&(link_ETX<ETX_THSLD))
    {
      maxWeight1 = compareWeight;

      #if SWITCHING
        if ((*prev_etx + 100 < link_ETX) && (tx != 1)){  
          PRINTF("routingtable: Moving to the next neighbor. This neighbor's ETX is %u\n",link_ETX);
          continue;
        }
      #endif /* SWITCHING */

      maxWeightPtr = i;           
      maxWeight = compareWeight;
      etx_best = link_ETX;
    }
    compareIdx++;
  }

  if (maxWeight >= 100) // Lower bound on the weights of an outgoing links 
  {
    // There is a neighbor we should be transmitting to
    rimeaddr_copy(act_neighbor, &(maxWeightPtr->neighbor));
    PRINTF_LVL2("routingtable: Neighbor address %d:%d with backpressure %d\n",
                                act_neighbor->u8[0],act_neighbor->u8[1],
                                            maxWeightPtr->backpressure);
        
    *act_neighbor_bp = maxWeightPtr->backpressure;
    maxWeightPtr->is_bursty_now = true; // Set the link as ``used recently''
    //PRINTF("etx_best=%lu\n",etx_best);
    
    #if SWITCHING
      *prev_etx = (etx_best < *prev_etx) * etx_best;
    #endif /* SWITCHING */

    return RT_SUCCESS;
  }
  else // There exists no neighbor that we want to transmit to at this time
  {
    flag_g = 2;
    *prev_etx = 1000;
    PRINTF_LVL2("routingtable: no neighbor with positive weight exist\n");
    return RT_FAILURE;
  } 
}



/**
 * Updates the Gillbert Elliot MC model 
 * @param  t                pointer to the routing table
 * @param  neighbor         the neighbor id to be updated
 * @param  success          whether the packet transmission was success
 */
void routingtable_update_link_status(struct routingtable *t, 
                                          const rimeaddr_t *neighbor,
                                          bool success)
{
  struct routingtable_item *i;

  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {
    if(rimeaddr_cmp(&(i->neighbor), neighbor)) 
      break;
  }

  if(i == NULL)// No match in routing table 
    return;
  
  PRINTF_LVL2("Before: ");
  PRINTF_LVL2("state:%u ",i->last_state);
  PRINTF_LVL2("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,
                        i->fail_to_success,i->success_to_fail,i->fail_to_fail);

  if(success)
  {
    if(i->last_state)
      i->success_to_success++;
    else
      i->fail_to_success++;
  }
  else
  {
    if(i->last_state)
      i->success_to_fail++;
    else
      i->fail_to_fail++;
  }

  i->last_state = success;

  PRINTF_LVL2("After: ");
  PRINTF_LVL2("state:%u ",i->last_state);
  PRINTF_LVL2("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,
                      i->fail_to_success,i->success_to_fail,i->fail_to_fail);
}

/*---------------------------------------------------------------------------*/
/**
 * @brief  Update the queue occupancy information of the neighbor
 * @param  t                pointer to the routing table
 * @param  neighbor         the neighbor id to be updated
 * @param  rcv_backpressure the queue length of the neighbor
 * @return                  status
 */
int routing_table_update_entry(struct routingtable *t,
                               const rimeaddr_t *neighbor,
                               uint16_t rcv_backpressure)
{

  struct routingtable_item *i;
  PRINTF_LVL2("Neighbor Update %d:%d\n",neighbor->u8[0],neighbor->u8[1]);

  // Check for entry
  for (i = list_head(*t->list); i != NULL; i = list_item_next(i))
  {
    if(rimeaddr_cmp(&(i->neighbor), neighbor))
      break;
  }

  // No match in routing table, add it
  if(i == NULL) 
  {
    if(routingtable_len(t) >= ROUTING_TABLE_SIZE)
    {
      //PRINTF("routingtable: routing_table_update_entry: Routing Table Full\n");
      return -1;
    }
    // Allocate Memory for new entry
    i = memb_alloc(t->memb);
    //Failed to allocate memory
    if(i == NULL) 
      return -1;
    // Set default attributes
    i->next                 = NULL;
    i->backpressure         = rcv_backpressure;
    i->link_packet_tx_time  = 128;  // TODO: why was this -1 previously?????

    //Set Parameters for Switching and ETX Calculation
    i->is_bursty_now        = false;
    i->success_to_fail      = 0;
    i->fail_to_success      = 1;
    i->fail_to_fail         = 0;
    i->success_to_success   = 1;
    i->last_state           = true;
    i->lifetime             = MAX_LIFE;
    //Copy the neighbor address
    rimeaddr_copy(&(i->neighbor), neighbor);
    // Add the item to the table
    list_add(*t->list, i);
  }
  else   //Entry found, update it
  {  
    i->backpressure = rcv_backpressure;
    //PRINTF("BP:%u\n",i->backpressure);
    i->lifetime=MAX_LIFE;
  }
  // dbg_print_rtable(t);
  // Return success
  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief  returns the current number of neighbors
 * @param  t    pointer to the routing table
 * @return      the number of neighbors
 */
int routingtable_len(struct routingtable *t)
{
  return list_length(*t->list);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief prints a routing table entry
 * @param rtitem      pointer to a routing table entry
 */
void dbg_print_rtitem(struct routingtable_item *rtitem)
{
  PRINTF("routingtable: backpressure: %d\n", rtitem->backpressure);
  PRINTF("routingtable: link_packet_tx_time: %u\n", rtitem->link_packet_tx_time);
//  PRINTF("link_etx: %lu\n", rtitem->link_etx);
  PRINTF("routingtable: is_bursty_now: %d\n", rtitem->is_bursty_now);
  PRINTF("routingtable: neighbor: %d.%d\n", rtitem->neighbor.u8[0], rtitem->neighbor.u8[1]);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief prints the entire routing table
 * @param t           pointer to the routing table
 */
void dbg_print_rtable(struct routingtable *t)
{
  struct routingtable_item *i;
  uint16_t numItems;
  uint16_t count = 0;

  numItems = routingtable_len(t);

  PRINTF("------------------------------------------------------------\n");
  PRINTF("Routing Table Contents: %d entries found\n", numItems);
  PRINTF("------------------------------------------------------------\n");
  for (i = list_head(*t->list); i != NULL; i = list_item_next(i)) {
    PRINTF("------------------------------------------------------------\n");
    PRINTF("Routing table item: %d\n", count);
    PRINTF("------------------------------------------------------------\n");
    dbg_print_rtitem(i);
    count++;
  }
  PRINTF("------------------------------------------------------------\n");
}
/*---------------------------------------------------------------------------*/
