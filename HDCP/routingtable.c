// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details


#include "routingtable.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTF_RTABLE(X) dbg_print_rtable(X)
#else
#define PRINTF(...)
#define PRINTF_RTABLE(Y)
#endif
#define OWNDEBUG 0
#define SWITCHING 0
#define ORIGINAL 1
#define COUNTTH 100
#define ETX_THSLD 800
#define queue_filter 0
static uint16_t flag_g=1;
//bool timersafe=false;
// Forward Declarations
void dbg_print_rtitem(struct routingtable_item *rtitem);
void dbg_print_rtable(struct routingtable *t);

/*---------------------------------------------------------------------------*/
void routingtable_init(struct routingtable *t)
{
  list_init(*t->list);
  memb_init(t->memb);
}
/*---------------------------------------------------------------------------*/
 int routingtable_update_routing(struct routingtable *t,
                                 uint16_t localBackpressure_p,
                                 rimeaddr_t *act_neighbor,
                                 uint32_t *act_neighbor_bp,
                                 uint16_t tx,
                                 uint32_t *prev_etx)
 {
  //----------VERIABLES INITIALIZATION------------------------//
   
   //PRINTF("routingtable_update_routing called\n");
   //PRINTF("FLAG %u\n",flag_g++);
   uint16_t compareIdx = 0;
   int32_t compareWeight = 0;
   //int32_t aa=0;
   uint16_t maxWeightIdx = 0;
   int32_t maxWeight = -1;
   int32_t maxWeight1 = -1;
   rimeaddr_t bestNeighbor;
   bestNeighbor.u8[0] = 0;
   bestNeighbor.u8[1] = 0;
   //rimeaddr_t *neigh_iter;
   //bool burstNeighborFound = 0;
   //uint16_t bestBurstNeighborIdx = 0;
   //uint16_t bestBurstNeighborBackpressure = 0;
   uint16_t longLocalBackpressure = localBackpressure_p;
   uint16_t longNeighborBackpressure = 0;
   uint16_t longLinkETX = 0;
   uint16_t etx_best=0;
   //uint16_t new_etx = 0;
   uint16_t longLinkRate = 0;
   uint16_t phi=0;
   uint16_t phi1=0;
   int16_t queue_differ=0;
   int16_t f_ij=0;
   int16_t best_f_ij=0;
   static uint16_t mu_ij=100;


   uint16_t beta=100;
   

   struct routingtable_item *i;
   //struct routingtable_item *bestBurstNeighborPtr = NULL;
   struct routingtable_item *maxWeightPtr = NULL;

   //-----------------------------------------------------------------------//

 

  
  //PRINTF("ENTRY: %d",*burst_count);
  if(routingtable_len(t) == 0) 
  {
    PRINTF(" however, routingtable: routingtable_len returned: %d - there are no neighbors\n", routingtable_len(t));// There are no neighbors
    return RT_FAILURE;
  }
  //PRINTF("Routing table:\n");
  // should be atomic START---------------------------------------------
  //timersafe=true;
  
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {
    //neigh_count++;
    // if(random_rand()%100<30)
    // {
    //   //PRINTF("Probably rejected\n");
    //   continue;
    // }  
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
      if(i->last_count>1)
        continue;
    }
    if(i->lifetime==0)
    {  
      //PRINTF("EXPIRED\n");
      continue;
    }
    
    // Convert link transmit time to packets / second
    longNeighborBackpressure = i->backpressure;
    
    //longLinkETX = LINK_LOSS_V * i->link_etx;
    
    //---------------Calculate the ETX based on the Markov chain transitions-----------------//
    // if(i->last_state)
    //   longLinkETX=LINK_LOSS_V*((i->success_to_success+i->success_to_fail)*100)/(i->success_to_success);
    // else
    //   longLinkETX=LINK_LOSS_V*((i->fail_to_success+i->fail_to_fail)*100)/(i->fail_to_success);
        
    longLinkETX = LINK_LOSS_V * i->link_etx;

    
    //longLinkRate = 100*128/(i->link_packet_tx_time); // modified***Packets per second
    
    queue_differ=(int16_t)(longLocalBackpressure-longNeighborBackpressure);

    // if(OWNDEBUG)
    // {
    //   PRINTF("Neighbor Backpressure = %u\n",longNeighborBackpressure);
    //   PRINTF("Neighbor longLinkETX = %u\n",longLinkETX);
    //   PRINTF("Neighbor %d:%d ",(&(i->neighbor))->u8[0],(&(i->neighbor))->u8[1]);
    //   PRINTF("LinkETX = %u ",longLinkETX);
    //   PRINTF("Neighbor link_packet_tx_time = %u\n",i->link_packet_tx_time);
    //   PRINTF("Neighbor longLinkRate = %lu\n",longLinkRate);
    //   PRINTF("Local Backpressure = %u\n",longLocalBackpressure);
    //   PRINTF("queue_diff = %d ",queue_differ);
    // }
    //128 ticks in cooja implies 1 sec
    //PRINTF("Neigh = %d \n",neigh_count);
    phi=(100-beta)+beta*100/(longLinkETX);
 
    // mu_ij=100;
       
    if(ORIGINAL)
    {
      if((int16_t)queue_differ>0)
      {
        //queue_differ=(int32_t)(longLocalBackpressure-longNeighborBackpressure);
        phi1=phi*queue_differ;
        // f_ij=phi1;
        if(phi1<mu_ij)
          f_ij=mu_ij;
        else
          f_ij=phi1;

      //PRINTF("f_ij = %d \n",f_ij);
      //PRINTF("queue_diff = %d \n",queue_differ);
      }
      else
      {
        //PRINTF(" phi_in: %u  mu_ij= %u  rate=%u \n", phi,mu_ij, longLinkRate);
        f_ij=0;//mu_ij;
      }
    }
    else
    {
      f_ij=mu_ij;
    }
    if(OWNDEBUG)
    {
      PRINTF("Phi = %d \n",phi);
      PRINTF("f_ij = %d \n",f_ij);
    }

    //------Calculate the weight of the link------------------//
    //
    if(ORIGINAL)
    {
      compareWeight = (2*(phi* queue_differ)-(f_ij));
      //PRINTF("Weight= %d \n",(int16_t)compareWeight);    
      if((int16_t)compareWeight<=0||f_ij==0)
        continue;
      compareWeight =((compareWeight/10) *f_ij)/10;
     // PRINTF("Weight= %ld \n",compareWeight);    
    }
    else
    {
      compareWeight = 2*(phi* queue_differ)-(f_ij);
    }

    if(OWNDEBUG)
    {
      PRINTF("ENTRY: ");
      PRINTF("ETX= %u ",longLinkETX);
      PRINTF("Neighbor_BP= %u ",longNeighborBackpressure);
      PRINTF("Local_BP= %u ",longLocalBackpressure);
      //PRINTF("queue_diff = %d ",queue_diff);
      PRINTF("Weight= %ld ",compareWeight);
      PRINTF("Neighbor %d:%d \n",(i->neighbor).u8[0],(i->neighbor).u8[1]);
    }
    if(queue_differ<=0)
      continue;

    if(queue_filter&&longNeighborBackpressure>=25)
      continue;

    i->lifetime=i->lifetime-1;

    if((compareWeight > maxWeight1))//&&longLinkETX<ETX_THSLD)//&&(longLinkETX<ETX_THSLD))
    {
      maxWeight1=compareWeight;
      if((*prev_etx+100<longLinkETX)&&(tx!=1)&&SWITCHING)
      {  
        //PRINTF("Moving to next neighbor1 %u\n",longLinkETX);
        continue;
      }
      maxWeightIdx = compareIdx;
      maxWeightPtr = i;           
      maxWeight = compareWeight;
      best_f_ij=f_ij;
      etx_best = longLinkETX;
    }
    compareIdx++;
  }
  //timersafe=false;
  //PRINTF("%ld\n",maxWeight);
  if(maxWeight>100)
  {

    // There is a neighbor we should be transmitting to
    //PRINTF("max weight %d\n",maxWeight);
    rimeaddr_copy(act_neighbor, &(maxWeightPtr->neighbor));
    if(OWNDEBUG)
      PRINTF("Neighbor %d:%d with backpressure %d\n",act_neighbor->u8[0],act_neighbor->u8[1],maxWeightPtr->backpressure);
        
    maxWeightPtr->last_count=maxWeightPtr->last_count+1;

    *act_neighbor_bp = maxWeightPtr->backpressure;
    maxWeightPtr->is_bursty_now=true;
    // *burst_count=best_f_ij;
    //PRINTF("etx_best=%lu\n",etx_best);
    if(SWITCHING)
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

//*---------------------------------------------------------------------------*/
void routingtable_update_link_success(struct routingtable *t,const rimeaddr_t *neighbor,uint16_t tx_count)
{
  //PRINTF("routingtable_update_link_success called\n ");
  struct routingtable_item *i;
  for(i = list_head(*t->list); i != NULL; i = list_item_next(i)) 
  {

     if(i == NULL)// No match in routing table 
      {
      //PRINTF("but routingtable entry not found\n");
        return;
      }
     // PRINTF("last_count= %u ",i->last_count);
    if(i->last_count>0) 
      i->link_etx = ((i->link_etx * LINK_LOSS_ALPHA) + (i->last_count *100* (100 - LINK_LOSS_ALPHA))) / 100;

    i->last_count=0;

  }
  
  //PRINTF("and routingtable_update_link_success: link_etx = %lu\n", i -> link_etx/100);
}

// // void

// void routingtable_update_link_status(struct routingtable *t,const rimeaddr_t *neighbor,bool success)
// {
// //PRINTF("routingtable_update_link_success called\n ");
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

//   if(OWNDEBUG)
//   {
//     PRINTF("Before: ");
//     PRINTF("state:%u ",i->last_state);
//     PRINTF("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,i->fail_to_success,i->success_to_fail,i->fail_to_fail);
//   }

//   if(success)
//   {
//       //i->success_count++;
      
//       if(i->last_state)
//       {  
//         i->success_to_success++;
//         // if(i->success_to_success>=COUNTTH)
//         // {
//         //   i->success_to_success=i->success_to_success/(COUNTTH/10);
//         //   i->success_to_fail=i->success_to_fail/(COUNTTH/10);
//         //   if(i->success_to_fail==0)
//         //     i->success_to_fail=1;
//         // }
//       }
//       else
//       { 
//         i->fail_to_success++;
//         // if(i->fail_to_success>=COUNTTH)
//         // {
//         //   i->fail_to_success=i->fail_to_success/(COUNTTH/10);
//         //   i->fail_to_fail=i->fail_to_fail/(COUNTTH/10);
//         //   if(i->fail_to_fail==0)
//         //     i->fail_to_fail=1;
//         // }
//       }
//   }
//  else
//   {
//       //i->fail_count++;
//       if(i->last_state)
//       {
//         i->success_to_fail++;
//         // if(i->success_to_fail>=COUNTTH)
//         // {
//         //   i->success_to_success=i->success_to_success/(COUNTTH/10);
//         //   i->success_to_fail=i->success_to_fail/(COUNTTH/10);
//         //   if(i->success_to_success==0)
//         //     i->success_to_success=1;
//         // }
//       }
//       else
//       {
//         i->fail_to_fail++;
//         // if(i->fail_to_fail>=COUNTTH)
//         // {
//         //   i->fail_to_success=i->fail_to_success/(COUNTTH/10);
//         //   i->fail_to_fail=i->fail_to_fail/(COUNTTH/10);
//         //   if(i->fail_to_success==0)
//         //     i->fail_to_success=1;
//         // }
//       }

//   }
//   i->last_state=success;
//   if(OWNDEBUG)
//   {
//     PRINTF("After: ");
//     PRINTF("state:%u ",i->last_state);
//     PRINTF("ss=%lu; fs=%lu; sf=%lu; ff=%lu \n",i->success_to_success,i->fail_to_success,i->success_to_fail,i->fail_to_fail);
//   }
// }
/*---------------------------------------------------------------------------*/
/*--------WE DONOT USE THE FOLLLOWING FUNCTION----------------------------*/
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

/*---------------------------------------------------------------------------*/
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
     i->link_etx = 100;
    i->last_count=0;
    //Set Parameters for Switching and ETX Calculation
    i->is_bursty_now = false;
    //i->success_count = 1;
    //i->fail_count = 0;
    // i->success_to_fail = 0;
    // i->fail_to_success = 1;
    // i->fail_to_fail = 0;
    // i->success_to_success = 1;
    // i->last_state=true;
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
/*---------------------------------------------------------------------------*/
int routingtable_len(struct routingtable *t)
{
  return list_length(*t->list);
}
/*---------------------------------------------------------------------------*/
void dbg_print_rtitem(struct routingtable_item *rtitem)
{
  PRINTF("backpressure: %d\n", rtitem->backpressure);
  PRINTF("link_packet_tx_time: %u\n", rtitem->link_packet_tx_time);
//  PRINTF("link_etx: %lu\n", rtitem->link_etx);
  PRINTF("is_bursty_now: %d\n", rtitem->is_bursty_now);
  PRINTF("neighbor: %d.%d\n", rtitem->neighbor.u8[0], rtitem->neighbor.u8[1]);
}
/*---------------------------------------------------------------------------*/
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
