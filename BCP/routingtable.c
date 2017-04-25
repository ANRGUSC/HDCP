// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#include "routingtable.h"
#include <stdio.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTF_RTABLE(X) dbg_print_rtable(X)
#else
#define PRINTF(...)
#define PRINTF_RTABLE(Y)
#endif
//bool timersafe=false;
#define PRO 0
#define OWNDEBUG 0
#define ETX_THSLD 800
#define SWITCHING 0

static uint16_t flag_g=1;
static uint16_t print_routing=1000;
// Forward Declarations
void dbg_print_rtitem(struct routingtable_item *rtitem);
void dbg_print_rtable(struct routingtable *t);

/**
 * [routingtable_init description]
 * @param t [description]
 */
void routingtable_init(struct routingtable *t)
{
  list_init(*t->list);
  memb_init(t->memb);
}

/**
 * [routingtable_update_routing description]
 * @param  t                   [description]
 * @param  localBackpressure_p [description]
 * @param  act_neighbor        [description]
 * @param  act_neighbor_bp     [description]
 * @param  tx                  [description]
 * @param  prev_etx            [description]
 * @return                     [description]
 */
int routingtable_update_routing(struct routingtable *t,
                                uint16_t localBackpressure_p,
                                rimeaddr_t *act_neighbor,
                                uint32_t *act_neighbor_bp,
                                uint16_t tx,
                                uint32_t *prev_etx)
{
  //PRINTF("routingtable_update_routing called\n");
  uint16_t compareIdx = 0;
  int32_t compareWeight = 0;
  uint16_t maxWeightIdx = 0;
  int32_t maxWeight = -1;
  int32_t maxWeight1 = -1;
  int16_t queue_diff=-1;
   rimeaddr_t *neigh_iter;
  rimeaddr_t bestNeighbor;
  bestNeighbor.u8[0] = 0;
  bestNeighbor.u8[1] = 0;
  bool burstNeighborFound = 0;
  uint16_t bestBurstNeighborIdx = 0;
  uint16_t bestBurstNeighborBackpressure = 0;
  uint16_t longLocalBackpressure = localBackpressure_p;
  uint16_t longNeighborBackpressure = 0;
  uint16_t longLinkETX = 0;
  uint16_t longLinkRate = 0;
  uint16_t timespent=0;
  uint16_t etx_best=0;
  struct routingtable_item *i;
  struct routingtable_item *bestBurstNeighborPtr = NULL;
  struct routingtable_item *maxWeightPtr = NULL;
  
  if(routingtable_len(t) == 0) 
  {
    PRINTF(" however, routingtable: routingtable_len returned: %d - there are no neighbors\n", routingtable_len(t));// There are no neighbors
    return RT_FAILURE;
  }

  if(print_routing==0)   
    print_routing=1000;
  //print_routing=print_routing-1;
  

  // should be atomic START---------------------------------------------
  if(print_routing==0)   
    PRINTF("Routing Table::\n");
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
	{

    if(tx==1) // On the first transmission set all neighbors to be available
    {
      i->is_bursty_now=false;
      flag_g=1;
    }

    if(flag_g==2) // When no good neighbor is available with switching, set all neighbors to be available again
    {
      i->is_bursty_now=false;
    }

    if(i->is_bursty_now&&SWITCHING) //If a neighbor is previously tried in transmitting this packet, skip that
    {
      //PRINTF("Moving to next neighbor from %d:%d\n",i->neighbor.u8[0],i->neighbor.u8[1]);
      continue;
    }
    // timespent=(clock_time()-i->lifetime)/128;
    // //PRINTF("%d\n",timespent);
    if(i->lifetime==0)
    {  
      //PRINTF("EXPIRED\n");
      continue;
    }
    // Convert link transmit time to packets / second
    longNeighborBackpressure = i->backpressure;
    //PRINTF("Neighbor Backpressure = %d\n",longNeighborBackpressure);
  
    if(i->last_state)//||tx==1)
      longLinkETX=LINK_LOSS_V*((i->success_to_success+i->success_to_fail)*100)/(i->success_to_success);
    else
      longLinkETX=LINK_LOSS_V*((i->fail_to_success+i->fail_to_fail)*100)/(i->fail_to_success);
    //longLinkETX = LINK_LOSS_V * i->link_etx;
    //PRINTF("LinkETX = %d ",longLinkETX);
    //longLinkRate = 128*100/(i->link_packet_tx_time); // modified***Packets per second
    //PRINTF("LinkRate = %d ",longLinkRate);
    //PRINTF("Local Backpressure = %d\n",longLocalBackpressure);
    if(OWNDEBUG)
    {
      PRINTF("Neighbor link_packet_tx_time = %d\n",i->link_packet_tx_time);
      PRINTF("Neighbor longLinkRate = %d\n",longLinkRate);
    }
    rimeaddr_copy(neigh_iter, &(i->neighbor));
    //s if(OWNDEBUG)
    //PRINTF("Neighbor %d:%d ",(&(i->neighbor))->u8[0],(&(i->neighbor))->u8[1]);
    // timersafe=true;
    queue_diff=longLocalBackpressure-longNeighborBackpressure;
    
    
    if(queue_diff>=0)
      compareWeight = ((int32_t)queue_diff*100-longLinkETX);
    else
      compareWeight=0;

    // * longLinkRate;
    // timersafe=false;
    //PRINTF("Backpressure %d   \n",longLocalBackpressure-longNeighborBackpressure);
    if(print_routing==0)    
    {
      PRINTF("ENTRY: ");
      PRINTF("ETX= %u ",longLinkETX);
      PRINTF("Neighbor_BP= %u ",longNeighborBackpressure);
      PRINTF("Local_BP= %u ",longLocalBackpressure);
      //PRINTF("queue_diff = %d ",queue_diff);
      PRINTF("Weight= %ld ",compareWeight);
      PRINTF("Neighbor %d:%d \n",(i->neighbor).u8[0],(i->neighbor).u8[1]);
    }
    if(queue_diff<0||(queue_diff<(longLinkETX/100))||longNeighborBackpressure>=25)
      continue;

    i->lifetime=i->lifetime-1;

    if((compareWeight > maxWeight1))//&&longLinkETX<ETX_THSLD)//&&(longLinkETX<ETX_THSLD))
    {
      maxWeight1=compareWeight;
      if((*prev_etx+100<longLinkETX)&&(tx!=1)&&SWITCHING)
      {  
        PRINTF("Moving to next neighbor1 %u\n",longLinkETX);
        continue;
      }
      maxWeightIdx = compareIdx;
      maxWeightPtr = i;
      maxWeight = compareWeight;
      etx_best = longLinkETX;
		}
    compareIdx++;

	}


  if(maxWeight>0) //PG was >=0
	{
    //PRINTF("wrong\n");
    // There is a neighbor we should be transmitting to
    rimeaddr_copy(act_neighbor, &(maxWeightPtr->neighbor));
    *act_neighbor_bp = maxWeightPtr->backpressure;
    maxWeightPtr->is_bursty_now=true;
    //PRINTF("etx_best=%lu\n",etx_best);
    if(SWITCHING&&tx==1)
    {
      if(etx_best<(*prev_etx))
        *prev_etx=etx_best;
    }
  
    return RT_SUCCESS;
  }
  else
  {
    flag_g=2;
    *prev_etx=1000;
    // There exists no neighbor that we want to transmit to at this time
    //PRINTF(" neighbor maxWeight < 0.\n");
    return RT_FAILURE;
  }
}


/*---------------------------------------------------------------------------*/
// void routingtable_update_link_success(struct routingtable *t,const rimeaddr_t *neighbor,uint16_t tx_count)
// {
//   //PRINTF("routingtable_update_link_success called\n ");
//   struct routingtable_item *i;
//   for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
//   {
//     if(rimeaddr_cmp(&(i->neighbor), neighbor)) 
//       break;
//   }

//   if(i == NULL)// No match in routing table 
//   {
//     //PRINTF("but routingtable entry not found\n");
//     return;
//   }

//   i->link_etx = ((i->link_etx * LINK_LOSS_ALPHA) + (tx_count *100* (100 - LINK_LOSS_ALPHA))) / 100;
//   //PRINTF("and routingtable_update_link_success: link_etx = %lu\n", i -> link_etx/100);
// }

/**
 * [routingtable_update_link_status description]
 * @param t        [description]
 * @param neighbor [description]
 * @param success  [description]
 */
void routingtable_update_link_status(struct routingtable *t,const rimeaddr_t *neighbor,bool success)
{
//PRINTF("routingtable_update_link_success called\n ");
  struct routingtable_item *i;

  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {
    if(rimeaddr_cmp(&(i->neighbor), neighbor)) 
      break;
  }

  if(i == NULL)// No match in routing table 
  {
    //PRINTF("but routingtable entry not found\n");
    return;
  }

  if(OWNDEBUG)
  {
    PRINTF("Before: ");
    PRINTF("state:%u ",i->last_state);
    PRINTF("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,i->fail_to_success,i->success_to_fail,i->fail_to_fail);
  }

  if(success)
  {
      //i->success_count++;
      
      if(i->last_state)
        i->success_to_success++;
      else
        i->fail_to_success++;
  }
 else
  {
      //i->fail_count++;
      if(i->last_state)
        i->success_to_fail++;
      else
        i->fail_to_fail++;

  }
  i->last_state=success;
  if(OWNDEBUG)
  {
    PRINTF("After: ");
    PRINTF("state:%u ",i->last_state);
    PRINTF("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,i->fail_to_success,i->success_to_fail,i->fail_to_fail);
  }
}
/*---------------------------------------------------------------------------*/
/*--------WE DONOT USE THE FOLLLOWING FUNCTION----------------------------*/
/**
 * [routingtable_update_link_rate description]
 * @param t                   [description]
 * @param neighbor            [description]
 * @param link_packet_tx_time [description]
 */
void routingtable_update_link_rate(struct routingtable *t,
                                   const rimeaddr_t *neighbor,
                                   uint16_t link_packet_tx_time)
{

  uint16_t new_link_packet_tx_time;
  struct routingtable_item *i;

  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) {
    if(rimeaddr_cmp(&(i->neighbor), neighbor))
      break;
  }

  // No match in routing table
  if(i == NULL) {
    //PRINTF("routingtable_update_link_rate: entry not found\n");
    return;
  }

  // Floor the delay value to 1 MS! Should be impossibly fast, but just in case.
  // Otherwise backpressure will be ignored for fast links.
  
  new_link_packet_tx_time = (link_packet_tx_time == 0) ? 1 : link_packet_tx_time;
  
  // Update the estimated packet transmission time for the link.
  // Use exponential weighted avg.
  if(OWNDEBUG)
    PRINTF("packet tx time=%d,%d, %d\n",i->link_packet_tx_time,LINK_EST_ALPHA,new_link_packet_tx_time);
  
  i->link_packet_tx_time =((LINK_EST_ALPHA * (uint32_t)(i->link_packet_tx_time))+ (10 - LINK_EST_ALPHA) * (uint32_t)new_link_packet_tx_time) / 10;
  
  if(OWNDEBUG)
    PRINTF("packet tx time=%d\n",i->link_packet_tx_time);
}

/*---------------------------------------------------------------------------*/
/**
 * [routing_table_update_entry description]
 * @param  t                [description]
 * @param  neighbor         [description]
 * @param  rcv_backpressure [description]
 * @return                  [description]
 */
int routing_table_update_entry(struct routingtable *t,
                               const rimeaddr_t *neighbor,
                               uint16_t rcv_backpressure)
{

  struct routingtable_item *i;
  if(OWNDEBUG)
    PRINTF("Neighbor Update %d:%d\n",neighbor->u8[0],neighbor->u8[1]);
  
  // Routing table is full, return error
  //if(routingtable_len(t) >= ROUTING_TABLE_SIZE)
  //{
    //PRINTF("routingtable: routing_table_update_entry: Routing Table Full\n");
  //  return -1;
  //}

  // Check for entry
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i))
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
    {
      return -1;
    }

    // Set default attributes
    i->next = NULL;
    i->backpressure = rcv_backpressure;
    i->link_packet_tx_time = 128;  // TODO: why was this -1 previously?????
//    i->link_etx = 100;
    
    //Set Parameters for Switching and ETX Calculation
    i->is_bursty_now = false;
    //i->success_count = 1;
    //i->fail_count = 0;
    i->success_to_fail = 0;
    i->fail_to_success = 1;
    i->fail_to_fail = 0;
    i->success_to_success = 1;
    i->last_state=true;
    i->lifetime=MAX_LIFE;
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
/**
 * [routingtable_len description]
 * @param  t [description]
 * @return   [description]
 */
int routingtable_len(struct routingtable *t)
{
  return list_length(*t->list);
}
/**
 * [dbg_print_rtitem description]
 * @param rtitem [description]
 */
void dbg_print_rtitem(struct routingtable_item *rtitem)
{
  PRINTF("backpressure: %d\n", rtitem->backpressure);
  PRINTF("link_packet_tx_time: %u\n", rtitem->link_packet_tx_time);
//  PRINTF("link_etx: %lu\n", rtitem->link_etx);
  PRINTF("is_bursty_now: %d\n", rtitem->is_bursty_now);
  PRINTF("neighbor: %d.%d\n", rtitem->neighbor.u8[0], rtitem->neighbor.u8[1]);
}
/**
 * [dbg_print_rtable description]
 * @param t [description]
 */
void dbg_print_rtable(struct routingtable *t)
{
  struct routingtable_item *i;
  uint16_t numItems = 0;
  uint16_t count = 0;

  numItems = routingtable_len(t);

  PRINTF("------------------------------------------------------------\n");
  PRINTF("Routing Table Contents: %d entries found\n", numItems);
  PRINTF("------------------------------------------------------------\n");
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) {
    PRINTF("------------------------------------------------------------\n");
    PRINTF("Routing table item: %d\n", count);
    PRINTF("------------------------------------------------------------\n");
    dbg_print_rtitem(i);
    count++;
  }
  PRINTF("------------------------------------------------------------\n");
}
/*---------------------------------------------------------------------------*/
