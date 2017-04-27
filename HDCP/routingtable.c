// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details


#include "routingtable.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTF_RTABLE(X) dbg_print_rtable(X)
#else
#define PRINTF(...)
#define PRINTF_RTABLE(Y)
#endif
#define OWNDEBUG      0
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
  uint16_t         compareIdx = 0;
  int32_t          compareWeight = 0;
  int32_t          maxWeight = -1;
  int32_t          maxWeight1 = -1;
  uint16_t         NeighborBackpressure = 0;
  uint16_t         link_ETX = 0;
  uint16_t         etx_best = 0;
  uint16_t         phi = 0;
  uint16_t         phi1 = 0;
  int16_t          queue_differ = 0;
  int16_t          f_ij = 0;
  static uint16_t  mu_ij = 100;
  uint16_t         beta = 100;
 
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
        if(i->last_count>1)
          continue;
      }
    #endif /* SWITCHING */

    if (i->lifetime == 0)
      continue;
    
    NeighborBackpressure = i->backpressure;
    link_ETX = LINK_LOSS_V * i->link_etx;  
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
    
    if(OWNDEBUG)
    {
      PRINTF("routingtable: Phi = %d \n",phi);
      PRINTF("routingtable: f_ij = %d \n",f_ij);
    }

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

    if(OWNDEBUG)
    {
      PRINTF("routingtable: ENTRY: ");
      PRINTF("routingtable: ETX = %u ",link_ETX);
      PRINTF("routingtable: Neighbor_BP = %u ",NeighborBackpressure);
      PRINTF("routingtable: Local_BP = %u ",localBackpressure_p);
      //PRINTF("queue_diff = %d ",queue_diff);
      PRINTF("routingtable: Weight = %ld ",compareWeight);
      PRINTF("routingtable: Neighbor %d:%d \n",(i->neighbor).u8[0],(i->neighbor).u8[1]);
    }
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
    if(OWNDEBUG)
      PRINTF("routingtable: Neighbor address %d:%d with backpressure %d\n",
                                act_neighbor->u8[0],act_neighbor->u8[1],
                                            maxWeightPtr->backpressure);
        
    maxWeightPtr->last_count = maxWeightPtr->last_count + 1; // Number of retries on this link
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
    PRINTF("routingtable: no neighbor with positive weight exist\n");
    return RT_FAILURE;
  }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief             update the link ETX
 * @param t           pointer to the routing table
 * @param neighbor    the other end of the link
 * @param tx_count    the last retransmission count
 */
void routingtable_update_link_success (struct routingtable *t , 
                                          const rimeaddr_t *neighbor ,
                                              uint16_t tx_count)
{
  struct routingtable_item *i;
  for (i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {
    if (i == NULL){// No match in routing table 
      PRINTF("but routingtable entry not found\n");
      return;
    }

    if(i->last_count > 0) // if number of tries on this link is non zero
      i->link_etx = ((i->link_etx * LINK_LOSS_ALPHA) 
                        + (i->last_count * 100 * (100 - LINK_LOSS_ALPHA))) / 100;

    i->last_count = 0; // reset the retransmission count after the update

  }
  PRINTF("routingtable: routingtable_update_link_success: new link_etx = %lu\n", i -> link_etx/100);
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
  if(OWNDEBUG)
    PRINTF("Neighbor Update %d:%d\n",neighbor->u8[0],neighbor->u8[1]);

  // Check for entry
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i))
  {
    if(rimeaddr_cmp(&(i->neighbor), neighbor))
      break;
  }

  
  if(i == NULL) // No matching entry in routing table, add the new neighbor
  {
    if(routingtable_len(t) >= ROUTING_TABLE_SIZE)
      return -1;

    i = memb_alloc(t->memb);     // Allocate Memory for new entry
    if (i == NULL) //Failed to allocate memory 
      return -1;

    // Set default attributes
    i->next = NULL;
    i->backpressure = rcv_backpressure;
    i->link_packet_tx_time = 128;  // TODO: why was this -1 previously?????
    i->link_etx = 100;
    i->last_count = 0;
    
    //Set Parameters for Switching and ETX Calculation
    i->is_bursty_now = false;
    i->lifetime = MAX_LIFE;

    rimeaddr_copy(&(i->neighbor), neighbor); //Copy the neighbor address
    list_add(*t->list, i);  // Add the item to the table
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
 * @brief prints a routing table entry
 * @param rtitem      pointer to a routing table entry
 */
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
