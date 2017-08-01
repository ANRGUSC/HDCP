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
 * @file        bcp.c
 * @brief       Main library for BCP routing
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 *   
 */

#include <stddef.h>  //For offsetof
#include "net/rime.h"
#include "net/netstack.h"
#include "bcp.h"
#include <string.h>
#include <stdio.h>
#include "lib/random.h"

#define VQ 						1 //Set 1 to use Virtual Queue
#define FIFO 					0
#define MAX_ACK_REXMITS			5  //retransmission no more than 5 times
#define COUNTED 				0  // Count the first try
#define SEND_TIME_DELAY			(CLOCK_SECOND/100)	// 50 ms
#define DELAY_TIME				CLOCK_SECOND * 60
#define BEACON_TIME 			CLOCK_SECOND * 5 	// 5 seconds //(random_rand() % CLOCK_SECOND * 60) //
#define FAST_BEACON_TIME 		CLOCK_SECOND * 2	// 500ms
#define MAX_SENDING_STACK 		25 //change it here to enable flexible stack size
#define MIN_TIME  				(CLOCK_SECOND/10)
#define REXMIT_TIME 			(CLOCK_SECOND/10)
#define RETRY_TIME				(CLOCK_SECOND/50)
#define MAX_HOPLIM   			10
#define MAX_MAC_REXMITS         5
#define MAX_REXMITS             31
#define MAX_ACK_MAC_REXMITS     5
#define NUM_RECENT_PACKETS 		30

#define DEBUG 		1
#define OWNDEBUG 	0
// #define OWNDEBUG 	0
#if DEBUG
	#include <stdio.h>
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif

#if OWNDEBUG
	#include <stdio.h>
	#define PRINTF_LVL2(...) printf(__VA_ARGS__)
#else
	#define PRINTF_LVL2(...)
#endif

bool tx_flag 			= 0;
static uint16_t s_no 	= 0;
static const struct packetbuf_attrlist attributes[] = {
	bcp_ATTRIBUTES
	PACKETBUF_ATTR_LAST };

struct recent_packet {
  	struct bcp_conn *conn;
	rimeaddr_t originator;
	uint16_t eseqno;
	uint16_t hop_count;
};

static struct 	recent_packet recent_packets[NUM_RECENT_PACKETS];
static uint8_t 	recent_packet_ptr;	

/**
 * @brief Update the recent packet list with the current packet
 * @param tc [pointer to the bcp connection]
 */
static void add_packet_to_recent_packets(struct bcp_conn *tc)
{
	struct data_hdr hdr;
	memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));
    recent_packets[recent_packet_ptr].eseqno =hdr.origin_seq_no;
	rimeaddr_copy(&recent_packets[recent_packet_ptr].originator,&(hdr.origin));
	recent_packets[recent_packet_ptr].conn = tc;
	recent_packets[recent_packet_ptr].hop_count = hdr.hop_count;
	recent_packet_ptr = (recent_packet_ptr + 1) % NUM_RECENT_PACKETS;
}


// Forward Declarations 
static void 	send_packet_bcp(void *ptr);
static void 	send_beacon(void *ptr);
static void 	send_ack(struct bcp_conn *bc, const rimeaddr_t *to, const rimeaddr_t *from, uint16_t seq_no);
static void 	retransmit_callback(void *ptr);
static struct 	packetstack_item *push_data_packet(struct bcp_conn *c);

/*---------------------------------------------------------------------------*/
MEMB(send_stack_memb, struct packetstack_item, MAX_SENDING_STACK);
MEMB(recent_pkt, struct packetstack_item, 1);
MEMB(routing_table_memb, struct routingtable_item, ROUTING_TABLE_SIZE);
/*---------------------------------------------------------------------------*/
struct beacon_msg {uint16_t backpressure;};
/*---------------------------------------------------------------------------*/
struct ack_msg {
	uint16_t id;
	rimeaddr_t origin;
};
/*---------------------------------------------------------------------------*/

/**
 * @brief callback function for bcp broadcast packet receive 
 * @param c    		pointer to the connection structure
 * @param from 		rime address of the packet source
 */
static void recv_from_broadcast(struct broadcast_conn *c, const rimeaddr_t *from)
{
	if(from->u8[1]!=0) 	//This is done to fix the problem with contiki
		return;			// where some broadcast have weirds source addresses.
						// In our experiment the address are in the format of (a:0)
	
	rimeaddr_t 		addr_null;  // Null address
	rimeaddr_t 		addr_broadcast1;  // Broadcast address
	rimeaddr_t 		addr_final_destination;
	
	addr_null.u8[0] 				= 0;
	addr_null.u8[1] 				= 0;
	addr_broadcast1.u8[0] 			= 255;
	addr_broadcast1.u8[1] 			= 255;
	addr_final_destination.u8[0] 	= 0;
	addr_final_destination.u8[1] 	= 0;
	int 		k;
	bool 		dup_flag			= false;
	bool 		hop_flag			= false;
	uint16_t	sn;
	
	static struct data_hdr 			hdr;
	static struct data_hdr 			*buf_hdr;
	static struct packetstack_item 	*i = NULL;
	struct bcp_conn *bc = (struct bcp_conn *)((char *)c - offsetof(struct bcp_conn, broadcast_conn));
	
	buf_hdr = packetbuf_dataptr();
	if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_DATA) // Check for data packets
	{
		memcpy(&hdr, buf_hdr, sizeof(struct data_hdr));
		if (hdr.origin.u8[1] != 0 || hdr.origin.u8[0] == 0) //This is done to fix the problem with contiki where some broadcast have wqeirds source addresses. In our experiment the address are in the format of (a:0)
			return;
		//PRINTF("bcp: From %d:%d\n",hdr.origin.u8[0],hdr.origin.u8[1]);
		hdr.timestamp=clock_time();
	}
	rimeaddr_copy(&addr_final_destination, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
	
	if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_DATA)
		PRINTF_LVL2("bcp: Esender %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],
									(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);

	if (rimeaddr_cmp(&addr_broadcast1,&hdr.origin))
		return;

	if ((packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_DATA)
						&&rimeaddr_cmp(&addr_final_destination,&rimeaddr_node_addr)) // Report the results
	{
		
		PRINTF_LVL2("bcp: From %d:%d\n",from->u8[0],from->u8[1]);
		PRINTF_LVL2("bcp: packet data length = %d\n",packetbuf_datalen());

		if (packetbuf_datalen() == 2)	// NULL packets		
		{      
			if ((bc->is_sink) == false)
			{
				PRINTF("bcp: NULL packet arrives (filtered by sink)\n");
				if(VQ)
					bc->virtual_queue_size++;
				routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure); 
			}
		} 
		else 
		{  
			for(k = 0; k < NUM_RECENT_PACKETS; k++) 
			{
  				if(recent_packets[k].eseqno == hdr.origin_seq_no &&
  						rimeaddr_cmp(&recent_packets[k].originator,&hdr.origin))
  			 	{
  			 		dup_flag=true; //Set the duplicate flag
		 			if(!bc->is_sink && recent_packets[k].hop_count != hdr.hop_count) 	//If not sink, same packet can come back after leaving the node. Need to handle correctly. 
		 				hop_flag = true;									//Need to also see if the same packet is already in the memory. If yes then discard. This part is not implemented.
     			}
  			}
  			
  			sn=hdr.origin_seq_no;
  			//PRINTF("bcp: seqno=%u\n",sn);
  			if(!dup_flag) //if not duplicate packets
  			{
  				add_packet_to_recent_packets(bc);
  				bool ttl_flag = 0;
  				if (hdr.hop_count >= MAX_HOPLIM)
  					ttl_flag=1; // bcp: max hop count reached

				if (!bc->is_sink) // If not sink 
				{
					//PRINTF("bcp: Received a forwarded packet from %d.%d to %d.%d (origin: %d.%d)\n", from -> u8[0], from -> u8[1], packetbuf_addr(PACKETBUF_ADDR_ERECEIVER) -> u8[0], packetbuf_addr(PACKETBUF_ADDR_ERECEIVER) -> u8[1], hdr.origin.u8[0],hdr.origin.u8[1]);
					if (!ttl_flag)
					{
						i = push_data_packet(bc); //Push onto our own send stack.i couldn't be null
						if (i == NULL)
							return;
						memcpy(&i->hdr, &hdr, sizeof(struct data_hdr)); 
					}
					send_ack(bc, from, &hdr.origin,sn); // Only send an ack if our stack isnt full
					routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);// Update the table
					PRINTF("bcp: P_received_forwarded_packet_at:%7lu   x %u from %d.%d with_hop_count %d timestamp %lu and delay %u via %d:%d\n",
									clock_time(), hdr.origin_seq_no, hdr.origin.u8[0],
									hdr.origin.u8[1], hdr.hop_count, hdr.timestamp,
									hdr.delay, from->u8[0],from->u8[1]);

					if (ctimer_expired(&bc->send_timer) && (!bc->sending)) // Reset the send data timer
					{
						clock_time_t time = SEND_TIME_DELAY;
						ctimer_set(&bc->send_timer, time, send_packet_bcp, bc);
					}
					// if(hdr.bcp_backpressure>10)
					// 	PRINTF_LVL2("bcp: Backpressure Error1 %d\n",hdr.bcp_backpressure);
				} 
				else 
				{
					send_ack(bc, from,&hdr.origin,sn);
					PRINTF("bcp: P_Sink_received_packet_at:%7lu   x %u     from %d.%d ",
						clock_time(), hdr.origin_seq_no, hdr.origin.u8[0], hdr.origin.u8[1]);
					PRINTF("with_hop_count %u timestamp %lu and delay %u etx %u\n",
					  	hdr.hop_count, hdr.timestamp, hdr.delay,hdr.tx_count);
				}
			}
			else // dup_flag
			{
				if (!hop_flag) 
					send_ack(bc, from,&hdr.origin,sn);
				routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);
				if (!(bc->is_sink)) 
					PRINTF("bcp: Duplicate_packet_at:%7lu x %u from %d.%d with_hop_count %d timestamp %lu and delay %u \n",
							clock_time(), hdr.origin_seq_no, hdr.origin.u8[0],
							hdr.origin.u8[1], hdr.hop_count, hdr.timestamp,
							hdr.delay);	
			}
		}
	}
	else if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_BEACON)
	{
		if (rimeaddr_cmp(from, &rimeaddr_node_addr)) // avoid self looping
			return;

		if (!rimeaddr_cmp((packetbuf_addr(PACKETBUF_ADDR_ERECEIVER)),&addr_null))
			return;
		
		static struct beacon_msg beacon;
		memcpy(&beacon, packetbuf_dataptr(), sizeof(struct beacon_msg));

		PRINTF_LVL2("bcp: Beacon backpressure = %d\n",beacon.backpressure);  
  		routing_table_update_entry(&bc->routing_table, from, beacon.backpressure);
		// if (beacon.backpressure > 10)
  		// PRINTF("bcp: Backpressure Error2 %d\n",beacon.backpressure);	
	} 
	else 
	{
		if (rimeaddr_cmp(from,&rimeaddr_node_addr))
			return;
		
		if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_DATA)
		{	
			routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);
			
			if (ctimer_expired(&(bc->send_timer)) && (!bc->sending)) // Reset the send data timer
			{
				clock_time_t time = SEND_TIME_DELAY;
				ctimer_set(&bc->send_timer, time, send_packet_bcp, bc);
			}
  		}
	}
}
/*---------------------------------------------------------------------------*/
/**
 * @brief This the callback function for bcp broadcast send
 * @param c             pointer to the connection structure
 * @param status        status of the packets send
 * @param transmissions number of transmission attemps
 */
static void sent_from_broadcast(struct broadcast_conn *c, int status, int transmissions)
{
	//PRINTF("bcp: sent_from_broadcast called %d,\n",transmissions);	
	// Cast the broadcast connection as a bcp connection
	struct bcp_conn 	*bcp_conn = (struct bcp_conn *)((char *)c - 
								offsetof(struct bcp_conn, broadcast_conn));
	if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_BEACON) 
	{
		// This was a beacon. Set sending to false and reset the beacon timer
		if(ctimer_expired(&bcp_conn->beacon_timer)) // Reset the beacon timer
		{
			clock_time_t time1 = (bcp_conn->is_sink) ? FAST_BEACON_TIME : BEACON_TIME;
			ctimer_set(&bcp_conn->beacon_timer, time1, send_beacon, bcp_conn);
		}
		bcp_conn->sending = false;
	}	
	else //This was a data message. Set up retransmission timer in case it does not get there
	{
		bcp_conn->transmissions += transmissions;
		clock_time_t time1 = MIN_TIME + (random_rand() % REXMIT_TIME);// Retransmission time
		ctimer_stop(&bcp_conn->retransmission_timer); // Start the retransmission timer
		ctimer_set(&bcp_conn->retransmission_timer, time1, retransmit_callback, bcp_conn);
	}
	//PRINTF("bcp: sent_from_broadcast called1\n");	
}
/*---------------------------------------------------------------------------*/
/**
 * @brief Unicast receive callback function 
 * @param c    		pointer to the connection structure
 * @param from 		the address of the unicast sender
 */
static void recv_from_unicast(struct unicast_conn *c, const rimeaddr_t *from)
{
	struct ack_msg 		hdr;
	struct bcp_conn 	*bcp_conn = (struct bcp_conn *)((char *)c -
	 									offsetof(struct bcp_conn, unicast_conn));// Cast the unicast connection as a bcp connection
	memcpy(&hdr, packetbuf_dataptr(), sizeof(struct ack_msg));
	PRINTF("bcp: recv_from_unicast called seq: %d actual %d \n",
								(bcp_conn->current_packet->hdr).origin_seq_no,
									packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));	
	//PRINTF("bcp: ack delay= %lu \n",(clock_time() - (bcp_conn->current_packet->hdr).timestamp));
	struct packetstack_item 	*i;
	
	if ((bcp_conn->current_packet->hdr).origin_seq_no == packetbuf_attr(PACKETBUF_ATTR_PACKET_ID)) //check 1. TODO: Add source check
	{
		i = bcp_conn->current_packet;
		if (i != NULL) 
		{
			if (!rimeaddr_cmp(&((bcp_conn->current_packet->hdr).origin),&hdr.origin))
				return;

			ctimer_stop(&bcp_conn->retransmission_timer); // Stop the retrasnsmission
			// uint32_t link_estimate_time = DELAY_TIME - timer_remaining(&bcp_conn->delay_timer); //estimate transmission time
			//PRINTF("bcp: link_estimate_time= %d\n",link_estimate_time);//also in ticks
			//PRINTF("bcp: tx_count= %d(in unicast where it will be used)\n",bcp_conn->tx_count);

			routingtable_update_link_status(&bcp_conn->routing_table, from, true); //Update link status
			bcp_conn->tx_count = 0;
			if (bcp_conn->current_packet)
				packetstack_remove(&bcp_conn->send_stack,bcp_conn->current_packet); // remove the last sent packet
			bcp_conn->current_packet = NULL;
			bcp_conn->sending = false;// Reset bcp connection for next packet to send

			if (ctimer_expired(&bcp_conn->send_timer)) // Reset the send data timer
			{
				clock_time_t time1 = SEND_TIME_DELAY;
				ctimer_set(&bcp_conn->send_timer, time1, send_packet_bcp, bcp_conn);// Reset the send data timer
			}
		}
	}
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_callbacks = {recv_from_broadcast,sent_from_broadcast };
static const struct unicast_callbacks unicast_callbacks = { recv_from_unicast };
/*---------------------------------------------------------------------------*/
/**
 * @brief pushes a packet to be sent in the sending queue
 * @param  c    	pointer to the connection structure
 * @return   		pointer to the queue
 */
struct packetstack_item *push_data_packet(struct bcp_conn *c)
{
	struct packetstack_item *i;
	uint16_t p = 0;
	// Set the packet type as data
	i = packetstack_push_packetbuf(&c->send_stack, c, &p);// Push the packet onto the send stack
	if(VQ)
	{
		if (p == 1 ) 
			c->virtual_queue_size++;// Successfully pushed the packet, return 1
	}
	if (i != NULL) 
		return i;
	
	PRINTF("bcp: ERROR: Send stack is full\n");// Unable to put the data packet on the send stack
	return NULL;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief sends the next bcp packet in the queue
 * @param ptr     	pointer to the connection structure
 */
static void send_packet_bcp(void *ptr)
{
	struct bcp_conn 	*c 	= ptr;

	if (c->sending == true)
	    return;   // If we are already sending, just return
	
	ctimer_stop(&c->send_timer); // Stop the send timer

	//ctimer_stop(&c->beacon_timer);
	struct queuebuf 			*q;// Queue Buffer struct
	struct packetstack_item 	*i;// Item in the packet stack
	
	#if FIFO
		i = packetstack_bottom(&c->send_stack); // Grab the item on the front of the queue
	#else
    	i = packetstack_top(&c->send_stack); // Grab the item on the back of the queue
    #endif /* FIFO */
    	
	if (i == NULL)  
	{
		PRINTF("bcp: %d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n",
							 rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		if(ctimer_expired(&c->beacon_timer))
			send_beacon(c);
		if(VQ)
		{
			if(c->virtual_queue_size > 0) 
				c->virtual_queue_size--;// Floor at 0
		}
		clock_time_t time1 = SEND_TIME_DELAY;
		ctimer_set(&c->send_timer, time1, send_packet_bcp, c);
		return;
	}

	// Send the packet
	q = packetstack_queuebuf(i); // get the packet's buffer
	
	if (q != NULL) 
	{
		c->sending = true; // Set the global sending flag
		tx_flag = 1;  // Set the local sending flag. Used for??
		
		queuebuf_to_packetbuf(q);// Place the queued packet into the packetbuf.
		uint16_t bp = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;

		PRINTF_LVL2("bcp: packetstack_length = %d\n",packetstack_len(&(c->send_stack)) );
		PRINTF_LVL2("bcp: virtual_queue_size = %d\n",c->virtual_queue_size);

		i->hdr.bcp_backpressure = bp; //Set the packet header backpressure info
		
		if(i->hdr.bcp_backpressure>100) // Sanity Filter
			PRINTF_LVL2("bcp: Packet BP ERROR: %d",i->hdr.bcp_backpressure);
		
		if (i->hdr.timestamp != 0)
		{
			i->hdr.delay = i->hdr.delay + (clock_time() - i->hdr.timestamp);	
			i->hdr.timestamp=clock_time();
		}
		//i->hdr.tx_count=i->hdr.tx_count+1;	//PG
		
		#if COUNTED
			c->tx_count++;
			if(c->tx_count <= 1) // Start the link rate timer if this is not a retransmit
			{
				clock_time_t time1 = DELAY_TIME;
				timer_set(&c->delay_timer, time1);
			}
		#endif /* COUNTED */	
        
		c->current_packet = i; //current not acked packet
		
		int return_val = routingtable_update_routing(&c->routing_table, bp,
							 &c->active_neighbor, &c->active_neighbor_backpressure,
							 1, &c->prev_etx); // Get the best neighbor

		if(return_val == RT_FAILURE) // No good neighbor
		{
			clock_time_t time1 = RETRY_TIME;// Retry time
			//ctimer_stop(&c->send_timer);// Start the retransmission timer
			ctimer_set(&c->send_timer, time1, send_packet_bcp, c); //Try sending again
			c->sending = false; //reset the send flag
			if (ctimer_expired(&c->beacon_timer)) // if beacon timer have expired send beacon immediately
				send_beacon(c);
			tx_flag = 0;
			return;//quit sending
		}

		#if !COUNTED
			c->tx_count++;
			if (c->tx_count <= 1) // Start the link rate timer if this is not a retransmit
			{
				clock_time_t time1 = DELAY_TIME;
				timer_set(&c->delay_timer, time1);
			}
		#endif

		c->transmissions = 0;
  		c->max_rexmits = packetbuf_attr(PACKETBUF_ATTR_MAX_REXMIT);
	    packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 1);

	    /*
	    	MAC Retransmission Setup
	     */
      	int max_mac_rexmits;
 	    max_mac_rexmits = c->max_rexmits > MAX_MAC_REXMITS? MAX_MAC_REXMITS : c->max_rexmits;
  		packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, max_mac_rexmits);
  		packetbuf_set_attr(PACKETBUF_ATTR_NUM_REXMIT,packetbuf_attr(PACKETBUF_ATTR_NUM_REXMIT)+1);

  		// Increase transmission and hop count of the packet
		i->hdr.tx_count = i->hdr.tx_count + 1;
		i->hdr.hop_count = i->hdr.hop_count + 1;

		memcpy(packetbuf_dataptr(), &(i->hdr), sizeof(struct data_hdr));
		struct data_hdr hdr;
		memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));

		packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &c->active_neighbor);
		PRINTF_LVL2("bcp: active_neighbor %d:%d\n",c->active_neighbor.u8[0],c->active_neighbor.u8[1]);
		
        if (c->packet_not_acked->buf != NULL)
        	queuebuf_free(c->packet_not_acked->buf);
        
        memset(c->packet_not_acked, 0, sizeof(struct packetstack_item));
        memcpy(&(c->packet_not_acked->hdr), &hdr, sizeof(struct data_hdr));
        c->packet_not_acked->buf = queuebuf_new_from_packetbuf();
        c->packet_not_acked->ptr = c;
		PRINTF_LVL2("bcp: ERC %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],
								(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);
		
		broadcast_send(&c->broadcast_conn); // Send the bcp packet via broadcast
		c->send_time = clock_time();// Set the first send time
			
		if (rimeaddr_node_addr.u8[0] != hdr.origin.u8[0])
			PRINTF("bcp: P_Forwarding_packet_at_time: %7lu  X %u  ",  c -> send_time, hdr.origin_seq_no);
		else
			PRINTF("bcp: P_Sending_packet_at_time: %7lu   X %u  ", c -> send_time, hdr.origin_seq_no);
		
		PRINTF("from %d.%d with queue: %d  virtualqueue: %u timestamp %lu and delay %u ",
							 hdr.origin.u8[0], hdr.origin.u8[1],packetstack_len(&(c->send_stack)),
							  c->virtual_queue_size, hdr.timestamp, hdr.delay);
		PRINTF("bp %u ",bp);
	    PRINTF("neighbor-bp %lu\n",c->active_neighbor_backpressure);
	    
	    tx_flag = 0;
		return;
	}
}


/*---------------------------------------------------------------------------*/
/**
 * @brief this function sends the periodic beacons for backpressure update
 * @param ptr   		pointer to the connection structure
 */
static void send_beacon(void *ptr)
{
	PRINTF_LVL2("bcp: send_beacon called %d\n",rimeaddr_node_addr.u8[0]);	
	rimeaddr_t 				addr_null;  // Broadcast address
	struct bcp_conn 		*c = ptr;
	struct beacon_msg 		beacon;
		
	addr_null.u8[0] = 0;
	addr_null.u8[1] = 0;
	
	if(c->sending == false) // If the link is free
		c->sending = true;
	else 					// If the link is busy
		return;

	packetbuf_clear();
	packetbuf_set_datalen(sizeof(struct beacon_msg));
	//beacon = packetbuf_dataptr();
	memset(&beacon, 0, sizeof(beacon));
	beacon.backpressure = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;// Store the local backpressure level
	memcpy(packetbuf_dataptr(), &beacon, sizeof(struct beacon_msg));
	packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,PACKETBUF_ATTR_PACKET_TYPE_BEACON);
	packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &addr_null);
	// Set the packet type
	PRINTF_LVL2("bcp: send beacon flag %d\n",beacon.backpressure);
	broadcast_send(&c->broadcast_conn);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief The retransmission function
 * @param ptr  		pointer to the connection structure
 */
static void send_restrans(void *ptr)
{
	struct bcp_conn *c = ptr;

	if (c->sending == false) 
		return;   // If we are not already sending the actual packet, just a safety filter
	
	struct queuebuf 		*q;	// Queue Buffer struct
	struct packetstack_item *i;	// Item in the packet stack
	i = c->packet_not_acked; 	// Grab the last non acked packet
	ctimer_stop(&c->retransmission_timer); //Stop the retrasnsmission timer

	if (i == NULL) // No packet to transmit
	{
		PRINTF_LVL2("bcp: %d.%d: Nothing to send. False retransmission \n",
							 rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		c->sending = false;
		ctimer_reset(&c->beacon_timer);// Start beaconing
		clock_time_t time1 = SEND_TIME_DELAY;
		ctimer_set(&c->send_timer, time1, send_packet_bcp, c);
	    if (ctimer_expired(&c->beacon_timer))
	    	send_beacon(c);
		return;
	}

	q = packetstack_queuebuf(i); // get the packetbuffer of the packet

	if (q != NULL) 
	{
		tx_flag = 1;
		queuebuf_to_packetbuf(q);// Place the queued packet into the packetbuf.
		uint16_t bp = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;
		i->hdr.bcp_backpressure = bp; // Set the packet's backpressure field
		
		if (i->hdr.timestamp != 0) 
		{
			 i->hdr.delay = i->hdr.delay + (clock_time() - i->hdr.timestamp);	
			 i->hdr.timestamp = clock_time();	
		}

		// Get the best neighbor
		int return_val = routingtable_update_routing(&c->routing_table, bp,
								&c->active_neighbor,&c->active_neighbor_backpressure,
									c->tx_count+1,&c->prev_etx);
		if(return_val == RT_FAILURE) // No neighbor exists
		{
			clock_time_t time1 = RETRY_TIME;// Retry time
			ctimer_set(&c->retransmission_timer, time1, send_restrans, c);
			if (ctimer_expired(&c->beacon_timer))
				send_beacon(c);
			tx_flag = 0;
			return; //quit sending
		}

		#if !COUNTED
			c->tx_count++;
		#endif /* !COUNTED */
		packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 1);
	  	
	  	int max_mac_rexmits;
  		max_mac_rexmits = c->max_rexmits - c->transmissions > MAX_MAC_REXMITS ?
  							MAX_MAC_REXMITS : c->max_rexmits - c->transmissions;
  		packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, max_mac_rexmits);
  		packetbuf_set_attr(PACKETBUF_ATTR_NUM_REXMIT,packetbuf_attr(PACKETBUF_ATTR_NUM_REXMIT)
  												+ c->transmissions);
		if (i->hdr.hop_count == 0) //if new packet
			i->hdr.hop_count++;
		
		i->hdr.tx_count = i->hdr.tx_count + 1; // Increase retransmission count
		memcpy(packetbuf_dataptr(), &(i->hdr), sizeof(struct data_hdr));

		packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &c->active_neighbor);
		struct data_hdr hdr;
		memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));
	
		PRINTF_LVL2("bcp: active_neighbor %d:%d\n",c->active_neighbor.u8[0],c->active_neighbor.u8[1]);
	   
		if (c->packet_not_acked->buf != NULL)
        	queuebuf_free(c->packet_not_acked->buf);
        
        memset(c->packet_not_acked, 0, sizeof(struct packetstack_item));
        memcpy(&(c->packet_not_acked->hdr), &hdr, sizeof(struct data_hdr));
        c->packet_not_acked->buf = queuebuf_new_from_packetbuf();
        c->packet_not_acked->ptr = c;
		PRINTF_LVL2("bcp: ERC %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);
		
		broadcast_send(&c->broadcast_conn); // Send the Packet
		
		if (!(rimeaddr_cmp(&rimeaddr_node_addr, &hdr.origin))) // Not original sender of the packet
		{
			c->send_time = clock_time();// Send time and transmission count
			PRINTF("bcp: P_Forwarding_packet_at_time(RE): %7lu  X %u  ",  c -> send_time, hdr.origin_seq_no);
		}
		else // original sender of the packet
		{
			c->send_time = clock_time();// Send time and transmission count
			PRINTF("bcp: P_Sending_packet_at_time(RE): %7lu   X %u  ", c -> send_time, hdr.origin_seq_no);
		}
		PRINTF("from %d.%d with queue: %d  virtualqueue: %u timestamp %lu and delay %u ",
									 hdr.origin.u8[0], hdr.origin.u8[1],
									 	packetstack_len(&(c->send_stack)),
									 		c->virtual_queue_size,
									 			hdr.timestamp, hdr.delay);
		PRINTF("bp %u ",bp);
	    PRINTF("neighbor-bp %lu\n",c->active_neighbor_backpressure);
		
		tx_flag = 0;
		return;
	}
	//PRINTF("bcp: After retransmission\n");
	//PRINTF("bcp: %d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
}

/**
 * @brief Callback function for retransmission after the timer finished
 * @param ptr 		pointer to the connection structure
 */
static void retransmit_callback(void *ptr)
{
	PRINTF_LVL2("bcp: retransmit_callback\n");	
	struct bcp_conn *c = ptr;
	
	routingtable_update_link_status(&c->routing_table, &c->active_neighbor, false);

	if (c->tx_count >= MAX_ACK_REXMITS) 
	{
		PRINTF("bcp: P_PACKET TIMED OUT, DROPPING %d\n",(c->current_packet->hdr).origin_seq_no);
		c->tx_count = 0;
		if (c->current_packet)
			packetstack_remove(&c->send_stack,c->current_packet);

		c->current_packet = NULL;
		c->sending = false;
		if(ctimer_expired(&(c->send_timer))) // Reset the send data timer
			send_packet_bcp(c);
	} 
	else
	{
		#if COUNTED
	 		c->tx_count++;
	 	#endif /* COUNTED */
	 	send_restrans(c);
	}
}

/*---------------------------------------------------------------------------*/
/**
 * @brief open a new bcp connection
 * @param c         pointer to the connection structure
 * @param channel   the channel number
 * @param callbacks callback functions
 */
void bcp_open(struct bcp_conn *c, uint16_t channel, const struct bcp_callbacks *callbacks)
{
	PRINTF_LVL2("bcp: bcp_open called\n");
	c->cb 						= callbacks;// Assign the callbacks to the connection struct
	
	LIST_STRUCT_INIT(c, send_stack_list);// Initialize the send stack LIST
	LIST_STRUCT_INIT(c, routing_table_list);// Initialize the routing table of neighbors LIST
	
	c->send_stack.list 			= &(c->send_stack_list);// Initialize send stack
	c->send_stack.memb 			= &send_stack_memb;
	c->routing_table.list 		= &(c->routing_table_list);// Initialize routing table
	c->routing_table.memb 		= &routing_table_memb;
	c->packet_not_acked 		= memb_alloc(&recent_pkt);
	c->packet_not_acked->buf 	= NULL;
	c->virtual_queue_size 		= 0;
	c->prev_etx 				= 1000;// Init virtual queue
	
	rimeaddr_t 					addr;// Init active neighbor
	addr.u8[0] 					= 0;
	addr.u8[1] 					= 0;

	rimeaddr_copy(&c->active_neighbor, &addr);
	broadcast_open(&c->broadcast_conn, channel, &broadcast_callbacks);// Open the broadcast connection for data packets and beacons
	channel_set_attributes(channel, attributes);
	unicast_open(&c->unicast_conn, channel + 1, &unicast_callbacks);// Open the unicast connection for ACKs (channel + 1)
	channel_set_attributes(channel + 1, attributes);
	send_beacon(c); // Start Beaconing
}
/*---------------------------------------------------------------------------*/
/**
 * @brief close the bcp connections
 * @param c [pointer to the connection structure]
 */
void bcp_close(struct bcp_conn *c)
{
	broadcast_close(&c->broadcast_conn);// Close the broadcast connection
	unicast_close(&c->unicast_conn);// Close the unicast connection
	ctimer_stop(&c->send_timer);// Stop the timers
	ctimer_stop(&c->beacon_timer);
	while(packetstack_top(&c->send_stack) != NULL) // Empty packet stack
		packetstack_pop(&c->send_stack);
}
/*---------------------------------------------------------------------------*/
/**
 * Send a bcp packet
 * @param  c [pointer to the connection structure]
 * @return   success status
 */
int bcp_send(struct bcp_conn *c)
{

	//PRINTF("bcp: bcp_send called\n");
	struct packetstack_item *i;
	
	packetbuf_clear();
	packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_PACKET_TYPE_DATA);
	packetbuf_set_attr(PACKETBUF_ATTR_NUM_REXMIT, 0);
    packetbuf_set_attr(PACKETBUF_ATTR_MAX_REXMIT, MAX_REXMITS);
    packetbuf_hdralloc(sizeof(struct data_hdr));// Allocate space for the header.

	i = push_data_packet(c);
	if (i != NULL) 
	{
		rimeaddr_copy(&(i->hdr.origin), &rimeaddr_node_addr);// Set the origin of the packet
		s_no++;
		i->hdr.origin_seq_no 	= s_no;
		i->hdr.hop_count 		= 0;
		i->hdr.timestamp 		= clock_time();
		i->hdr.delay 			= 0;
		i->hdr.tx_count 		= 0;
		
		if (ctimer_expired(&c->send_timer) && (!c->sending)) // Reset the send data timer
		{
			clock_time_t time1 = SEND_TIME_DELAY;
			ctimer_set(&c->send_timer, time1, send_packet_bcp, c);
		}
		return 1;
	}	
	if (ctimer_expired(&c->send_timer) && (!c->sending))// Reset the send data timer
	{
		clock_time_t time1 = SEND_TIME_DELAY;
	 	ctimer_set(&c->send_timer, time1, send_packet_bcp, c);
 	}
 	return 0;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief Sends back acknowledgement of a bcp packet
 * @param bc      pointer to the connection structure
 * @param to      reciever's address
 * @param from    sender's address
 * @param seq_no  sequence number to acknowledge
 */
static void send_ack(struct bcp_conn *bc, const rimeaddr_t *to, const rimeaddr_t *from, uint16_t seq_no)
{
	//PRINTF("bcp: send_ack called\n");
	struct ack_msg ack;

	packetbuf_clear();
	packetbuf_set_datalen(sizeof(struct ack_msg));
	
	memset(&ack, 0, sizeof(struct ack_msg));
	rimeaddr_copy(&(ack.origin), from);
	ack.id = seq_no;
	memcpy(packetbuf_dataptr(),&ack,sizeof(struct ack_msg));
	
	packetbuf_set_attr(PACKETBUF_ATTR_PACKET_ID, seq_no);
	packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_PACKET_TYPE_ACK);
	packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 0);
	packetbuf_set_attr(PACKETBUF_ATTR_ERELIABLE, 0);
	packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, MAX_ACK_MAC_REXMITS);

	unicast_send(&bc->unicast_conn, to); // Send the Ack Via Unicast
}
/*---------------------------------------------------------------------------*/
/**
 * @brief if the node's address is @p addr, set the node as a sink node
 * @param c    [pointer to the connection structure]
 * @param addr the address of the sink node
 */
void bcp_set_sink(struct bcp_conn *c, const rimeaddr_t *addr)
{
	if(rimeaddr_cmp(addr, &rimeaddr_node_addr)) // This node IS a sink node
	{  
		//PRINTF("bcp: bcp_set_sink: %d.%d set as sink node.\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		c->is_sink = true;
		c->virtual_queue_size = 0;// As soon as we are root, should have zero virtual queue
	} 
	else //This node is not a sink node
		c->is_sink = false;
}
/*---------------------------------------------------------------------------*/


