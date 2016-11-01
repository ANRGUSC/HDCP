// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#include <stddef.h>  //For offsetof
#include "net/rime.h"
#include "net/netstack.h"
#include "hdcp.h"
#include <string.h>
#include <stdio.h>
#include "lib/random.h"
static uint16_t s_no = 0;
static const struct packetbuf_attrlist attributes[] = {
	BCP_ATTRIBUTES
	PACKETBUF_ATTR_LAST };
#define VQ 1 //Set 1 to use Virtual Queue
#define FIFO 0
#define MAX_ACK_REXMITS			5  //retransmission no more than 5 times
#define COUNTED 0
#define SEND_TIME_DELAY	(CLOCK_SECOND/100)	// 50 ms
#define DELAY_TIME			CLOCK_SECOND * 60
#define BEACON_TIME 		CLOCK_SECOND * 5 	// 5 seconds //(random_rand() % CLOCK_SECOND * 60) //
#define FAST_BEACON_TIME 	CLOCK_SECOND * 2	// 500ms
#define MAX_SENDING_STACK 		25 //change it here to enable flexible stack size
#define MIN_TIME  (CLOCK_SECOND/10)
#define REXMIT_TIME (CLOCK_SECOND/5)
#define RETRY_TIME	(CLOCK_SECOND/20)
#define MAX_HOPLIM   10


#define DEBUG 1
#if DEBUG

#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
#define OWNDEBUG 0
#define OWNDEBUG1 0
#define NUM_RECENT_PACKETS 30
bool tx_flag=0;
bool tx_flag1=0;
bool first_send=0;
	struct recent_packet 
	{
	  	struct bcp_conn *conn;
  		rimeaddr_t originator;
  		uint16_t eseqno;
  		uint16_t hop_count;
	};

	static struct recent_packet recent_packets[NUM_RECENT_PACKETS];
	static uint8_t recent_packet_ptr;	
	static void add_packet_to_recent_packets(struct bcp_conn *tc)
	{
		struct data_hdr hdr;
  		memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));
 		// PRINTF("Inside %d\n",hdr.origin_seq_no );
	    recent_packets[recent_packet_ptr].eseqno =hdr.origin_seq_no;
    	rimeaddr_copy(&recent_packets[recent_packet_ptr].originator,&(hdr.origin));
    	recent_packets[recent_packet_ptr].conn = tc;
    	recent_packets[recent_packet_ptr].hop_count = hdr.hop_count;
    	recent_packet_ptr = (recent_packet_ptr + 1) % NUM_RECENT_PACKETS;
	}


	// Forward Declarations 
	static void send_packet_pg(void *ptr);
	static void send_beacon(void *ptr);
	static void send_ack(struct bcp_conn *bc, const rimeaddr_t *to, const rimeaddr_t *from, uint16_t seq_no);
	static void retransmit_callback(void *ptr);
	static struct packetstack_item *push_data_packet(struct bcp_conn *c);

	/*---------------------------------------------------------------------------*/
	MEMB(send_stack_memb, struct packetstack_item, MAX_SENDING_STACK);
	MEMB(recent_pkt, struct packetstack_item, 1);
	MEMB(routing_table_memb, struct routingtable_item, ROUTING_TABLE_SIZE);
	/*---------------------------------------------------------------------------*/
	struct beacon_msg {uint16_t backpressure;};
	/*---------------------------------------------------------------------------*/
	struct ack_msg 
	{
		uint16_t id;
		rimeaddr_t origin;
	};
	/*---------------------------------------------------------------------------*/
	

	static void recv_from_broadcast(struct broadcast_conn *c, const rimeaddr_t *from)
	{
		//PRINTF("recv_from_broadcast called\n");
		//if(OWNDEBUG1)
		if(from->u8[1]!=0) //This is done to fix the problem with contiki where some broadcast have wqeirds source addresses. In our experiment the address are in the format of (a:0)
			return;
		//PRINTF("From %d:%d\n",from->u8[0],from->u8[1]);	
		

		rimeaddr_t addr_broadcast;  // Broadcast address
		addr_broadcast.u8[0] = 0;
		addr_broadcast.u8[1] = 0;
		rimeaddr_t addr_broadcast1;  // Broadcast address
		addr_broadcast1.u8[0] = 255;
		addr_broadcast1.u8[1] = 255;
		rimeaddr_t sentToNode;
		sentToNode.u8[0] = 0;
		sentToNode.u8[1] = 0;
		int k;
		bool pflag=false;
		bool hop_flag=false;
		uint16_t sn;
		
		struct bcp_conn *bc = (struct bcp_conn *)((char *)c- offsetof(struct bcp_conn, broadcast_conn));
		static struct data_hdr hdr;
		static struct data_hdr *buf_hdr;
		static struct packetstack_item *i;

		buf_hdr=packetbuf_dataptr();
		i=NULL;
		if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==PACKETBUF_ATTR_PACKET_TYPE_DATA)
		{
			memcpy(&hdr, buf_hdr, sizeof(struct data_hdr));
			if(hdr.origin.u8[1]!=0||hdr.origin.u8[0]==0) //This is done to fix the problem with contiki where some broadcast have wqeirds source addresses. In our experiment the address are in the format of (a:0)
				return;
			//PRINTF("From %d:%d\n",hdr.origin.u8[0],hdr.origin.u8[1]);

			hdr.timestamp=clock_time();
		}
		rimeaddr_copy(&sentToNode, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
		
		if(OWNDEBUG1)
		{
			if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==PACKETBUF_ATTR_PACKET_TYPE_DATA)
				PRINTF("Esender %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);
		}
		
		if(rimeaddr_cmp(&addr_broadcast1,&hdr.origin))
			return;


		if((packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==PACKETBUF_ATTR_PACKET_TYPE_DATA)&&rimeaddr_cmp(&sentToNode,&rimeaddr_node_addr)) // Report the results
		{
			
			if(OWNDEBUG1)
				PRINTF("From %d:%d\n",from->u8[0],from->u8[1]);
			
			//PRINTF("packet data length = %d\n",packetbuf_datalen());
			if (packetbuf_datalen()==2)			
			{      
				if((bc->is_sink)==false)
				{
					PRINTF("NULL packet arrives (filtered by sink)\n");
					if(VQ)
						bc->virtual_queue_size++;
					
					//PRINTF("From4 %d:%d\n",from->u8[0],from->u8[1]);
					routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure); 
				}
			} 
			else 
			{  
				for(k = 0; k < NUM_RECENT_PACKETS; k++) 
				{
      				if(recent_packets[k].eseqno == hdr.origin_seq_no && rimeaddr_cmp(&recent_packets[k].originator,&hdr.origin))
      			 	{
      			 		if(bc->is_sink) //If Sink, discard any duplicate packet
      			 			pflag=true;
      			 		else
      			 		{
      			 			if(recent_packets[k].hop_count == hdr.hop_count) 	//If not sink, same packet can come back after leaving the node. Need to handle correctly. 
      			 				pflag=true;	
      			 			else
      			 			{	
      			 				pflag=true;
      			 				if(!(bc->is_sink)) 	
      			 					hop_flag=true;									//Need to also see if the same packet is already in the memory. If yes then discard. This part is not implemented.
							}
							//PRINTF("GOT IT\n");
						}	
	     			}
      				//PRINTF("la la la %d from %u.%u\n",recent_packets[k].eseqno,recent_packets[k].originator.u8[0],recent_packets[k].originator.u8[1]);
      			}
      			
      			sn=hdr.origin_seq_no;
      			//PRINTF("seqno=%u\n",sn);
      			if(!pflag)
      			{
      				add_packet_to_recent_packets(bc);
      				bool ttl_flag=0;
      				if(hdr.hop_count>=MAX_HOPLIM)
      				{	
      					ttl_flag=1;
      					PRINTF("max hop count reached!!\n");
      				}
      				if(hdr.origin_seq_no>2000)
      					return;
					if(!(bc->is_sink)) 
					{


						//PRINTF("Received a forwarded packet from %d.%d to %d.%d (origin: %d.%d)\n", from -> u8[0], from -> u8[1], packetbuf_addr(PACKETBUF_ADDR_ERECEIVER) -> u8[0], packetbuf_addr(PACKETBUF_ADDR_ERECEIVER) -> u8[1], hdr.origin.u8[0],hdr.origin.u8[1]);
						if(!ttl_flag)
						{
							i = push_data_packet(bc);//Push onto our own send stack.i couldn't be null
							if(i==NULL)
							{
								return;
							}
							memcpy(&i->hdr,&hdr,sizeof(struct data_hdr)); //PG CORRECTED//
						//i->hdr = hdr;// Set the header of the packet that was pushed onto the stack
						//PRINTF("Delay %d ",i->hdr.delay);
									
							
						}
						send_ack(bc, from,&hdr.origin,sn);// Only send an ack if our stack isnt full
						routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);// Update the table
						PRINTF("P_received_forwarded_packet_at:%7lu   x %u from %d.%d with_hop_count %d timestamp %lu and delay %u  ", clock_time(), hdr.origin_seq_no, hdr.origin.u8[0], hdr.origin.u8[1], hdr.hop_count, hdr.timestamp,hdr.delay);
						PRINTF("From %d:%d\n",from->u8[0],from->u8[1]);

						if((ctimer_expired(&(bc->send_timer)))&&(!bc->sending)) // Reset the send data timer
						{
							clock_time_t time = SEND_TIME_DELAY;
							ctimer_set(&bc->send_timer, time, send_packet_pg, bc);
						}
						if(OWNDEBUG)
						{	
							if(hdr.bcp_backpressure>10)
      							PRINTF("Backpressure Error1 %d\n",hdr.bcp_backpressure);
      					}
					} 
					else 
					{
						//add_packet_to_recent_packets(bc);
						//PRINTF("with_hop_count %u ",hdr.hop_count);
						//						
						send_ack(bc, from,&hdr.origin,sn);
						PRINTF("P_Sink_received_packet_at:%7lu   x %u     from %d.%d ",clock_time(), hdr.origin_seq_no, hdr.origin.u8[0], hdr.origin.u8[1]);
						PRINTF("with_hop_count %u timestamp %lu and delay %u etx %u\n",  hdr.hop_count, hdr.timestamp, hdr.delay,hdr.tx_count);
					}
				}
				else
				{
					//PRINTF("From %d:%d\n",from->u8[0],from->u8[1]);
					if(!hop_flag)
						send_ack(bc, from,&hdr.origin,sn);
										//PRINTF("From %d.%d\n",from->u8[0],from->u8[1]);
					routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);
					if(!(bc->is_sink)) 
						PRINTF("Duplicate_packet_at:%7lu   x %u     from %d.%d with_hop_count %d timestamp %lu and delay %u \n", clock_time(), hdr.origin_seq_no, hdr.origin.u8[0], hdr.origin.u8[1], hdr.hop_count, hdr.timestamp,hdr.delay);	

				}
			}
		}
		else if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==PACKETBUF_ATTR_PACKET_TYPE_BEACON)
		//else if(rimeaddr_cmp((packetbuf_addr(PACKETBUF_ADDR_ERECEIVER)),&addr_broadcast)) 
		{
			if(rimeaddr_cmp(from,&rimeaddr_node_addr))
				return;

			if(!rimeaddr_cmp((packetbuf_addr(PACKETBUF_ADDR_ERECEIVER)),&addr_broadcast))
				return;
			//if(addr_broadcast1)
			//PRINTF("beacon data length = %d\n",packetbuf_datalen());  
			static struct beacon_msg beacon;
			memcpy(&beacon, packetbuf_dataptr(), sizeof(struct beacon_msg));
			if(OWNDEBUG)
			{	if(beacon.backpressure>10)
      						PRINTF("Backpressure Error2 %d\n",beacon.backpressure);
      		}
      		//PRINTF("From2 %d.%d\n",from->u8[0],from->u8[1]);
			routing_table_update_entry(&bc->routing_table, from, beacon.backpressure);
			//PRINTF("Beacon backpressure = %d\n",beacon.backpressure);  
		} 
		else 
		{
			if(rimeaddr_cmp(from,&rimeaddr_node_addr))
				return;
			if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==PACKETBUF_ATTR_PACKET_TYPE_DATA)
			{	
				//PRINTF("yes, %u\n",);
				routing_table_update_entry(&bc->routing_table, from, hdr.bcp_backpressure);
				//PRINTF("From3 %d:%d,  %u\n",from->u8[0],from->u8[1],hdr.bcp_backpressure);

				if((ctimer_expired(&(bc->send_timer)))&&(!bc->sending)) // Reset the send data timer
				{
					clock_time_t time = SEND_TIME_DELAY;
					ctimer_set(&bc->send_timer, time, send_packet_pg, bc);
				}
				if(OWNDEBUG)
				{	
					if(hdr.bcp_backpressure>10)
      						PRINTF("Backpressure Error3 %d\n",hdr.bcp_backpressure);
      			}
      		}
		}
	
		//Invoke the callback for BCP recv if this node was the intended 'hop' or is the sink
		/*if(rimeaddr_cmp(&sentToNode, &rimeaddr_node_addr)) {
		if(bc->cb->recv != NULL) {
		bc->cb->recv(bc,hdr);
		} else {
		PRINTF("BCP: Error, BCP receive callback was not set.\n");
		}
		}*/
		LAST: 

		if(OWNDEBUG)
		{
      		PRINTF("\n");
      	}
	}
	/*---------------------------------------------------------------------------*/
	static void sent_from_broadcast(struct broadcast_conn *c, int status, int transmissions)
	{
		//PRINTF("sent_from_broadcast called\n");	
		// Cast the broadcast connection as a BCP connection
		struct bcp_conn *bcp_conn = (struct bcp_conn *)((char *)c-offsetof(struct bcp_conn, broadcast_conn));
		if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == PACKETBUF_ATTR_PACKET_TYPE_BEACON) 
		{
			//PRINTF("1\n");	
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
			//PRINTF("2\n");	
			//PRINTF("RESET\n");
			clock_time_t time1 = MIN_TIME+(random_rand()%(REXMIT_TIME));// Retransmission time
			// if(first_send)
			// {
			//  	time1=time1*16;
			//  	first_send=0;
			// }
			//PRINTF("TIMER=%lu \n",time1);
			//clock_time_t time1 = MIN_TIME+(random_rand()%((bcp_conn->tx_count+1)*REXMIT_TIME));// Retransmission time
			
			ctimer_stop(&bcp_conn->retransmission_timer);// Start the retransmission timer
			ctimer_set(&bcp_conn->retransmission_timer, time1, retransmit_callback, bcp_conn);
			//clock_time_t time = SEND_TIME_DELAY;
			////PG
			//clock_time_t time2 = SEND_TIME_DELAY;
			//ctimer_set(&bcp_conn->send_timer, time2, send_packet_pg, bcp_conn);
 			
 			//ctimer_reset(&bcp_conn->send_timer);
 			//ctimer_restart(&bcp_conn->beacon_timer);

		}
		//PRINTF("sent_from_broadcast called1\n");	
	}
	/*---------------------------------------------------------------------------*/
	static void recv_from_unicast(struct unicast_conn *c, const rimeaddr_t *from)
	{
		
		struct ack_msg hdr;
		struct bcp_conn *bcp_conn = (struct bcp_conn *)((char *)c- offsetof(struct bcp_conn, unicast_conn));// Cast the unicast connection as a BCP connection
		memcpy(&hdr, packetbuf_dataptr(), sizeof(struct ack_msg));
	//	if(OWNDEBUG1)
			//PRINTF("CLock=%lu \n",SEND_TIME_DELAY);
		PRINTF("recv_from_unicast called seq: %d   actual %d \n",(bcp_conn->current_packet->hdr).origin_seq_no,packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));	
		//PRINTF("ack delay= %lu \n",(clock_time() - (bcp_conn->current_packet->hdr).timestamp));
		struct packetstack_item *i;
		//if(bcp_conn->current_packet==NULL)
		//{
		//	PRINTF("Nothing to Ack\n");
		//	return;
		//}
		
		if((bcp_conn->current_packet->hdr).origin_seq_no==packetbuf_attr(PACKETBUF_ATTR_PACKET_ID)) //check 1. TODO: Add source check
		{
			//PRINTF("INSIDE\n");
			
			i = bcp_conn->current_packet;
			if(i != NULL) 
			{
				if(!rimeaddr_cmp(&((bcp_conn->current_packet->hdr).origin),&hdr.origin))
					return;

				ctimer_stop(&bcp_conn->retransmission_timer);
				// if(timer_expired(&bcp_conn->delay_timer) != 0) 
				// {
				// 	//PRINTF("hdcp: ERROR: delay_timer expired.\n");
				// }
				//DR("DELAY_TIME= %d\n",DELAY_TIME);//it equals 15360 (ticks)
				uint32_t link_estimate_time = DELAY_TIME- timer_remaining(&bcp_conn->delay_timer);
				//PRINTF("link_estimate_time= %d\n",link_estimate_time);//also in ticks
				//PRINTF("tx_count= %d(in unicast where it will be used)\n",bcp_conn->tx_count);
				routingtable_update_link_rate(&bcp_conn->routing_table, from,link_estimate_time);
				//routingtable_update_link_success(&bcp_conn->routing_table, from, bcp_conn->tx_count);
				routingtable_update_link_status(&bcp_conn->routing_table, from, true);
				bcp_conn->tx_count = 0;
				if(bcp_conn->current_packet)
					packetstack_remove(&bcp_conn->send_stack,bcp_conn->current_packet);
				bcp_conn->current_packet = NULL;
				bcp_conn->sending = false;// Reset BCP connection for next packet to send

				//PG CORRECTED//
				if(ctimer_expired(&bcp_conn->send_timer)) // Reset the send data timer
				{
					clock_time_t time1 = SEND_TIME_DELAY;
					ctimer_set(&bcp_conn->send_timer, time1, send_packet_pg, bcp_conn);// Reset the send data timer
				}
				//PG CORRECTED//
			}
		}
	}
	/*---------------------------------------------------------------------------*/
	static const struct broadcast_callbacks broadcast_callbacks = {recv_from_broadcast,sent_from_broadcast };
	static const struct unicast_callbacks unicast_callbacks = { recv_from_unicast };
	/*---------------------------------------------------------------------------*/
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
		//PRINTF("ERROR: Send stack is full\n");// Unable to put the data packet on the send stack
		return NULL;
	}
	/*---------------------------------------------------------------------------*/
	static void send_packet_pg(void *ptr)
	{
		//PRINTF("send_packet_pg called\n");
		rimeaddr_t addr_broadcast;  // Broadcast address
		addr_broadcast.u8[0] = 0;
		addr_broadcast.u8[1] = 0;
		struct bcp_conn *c = ptr;
		struct beacon_msg beacon;
		//static rimeaddr_t addr;
		//addr.u8[0]=9;
		//addr.u8[1]=9;
		
		if(c->sending==true)
		{
			//PG
			//clock_time_t time1 = SEND_TIME_DELAY;
 			//ctimer_set(&c->send_timer, time1, send_packet_pg, c);
 			//PG
		    return;   // If we are already sending, just return
		}

		//tx_flag=1;
		//ctimer_stop(&c->retransmission_timer);
		ctimer_stop(&c->send_timer);

		//ctimer_stop(&c->beacon_timer);
		struct queuebuf *q;// Queue Buffer struct
		struct packetstack_item *i;// Item in the packet stack
		if(!FIFO)
        	i = packetstack_top(&c->send_stack);// Grab the item on top of the stack
        else
        	i= packetstack_bottom(&c->send_stack);
       
		//	i = packetstack_top(&c->send_stack);// Grab the item on top of the stack
		if(i == NULL) 
		{
			//PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
			if(ctimer_expired(&c->beacon_timer))
			{
				send_beacon(c);
				//ctimer_reset(&c->beacon_timer);// Start beaconing
			}
			if(VQ)
			{
				if(c->virtual_queue_size > 0) 
					c->virtual_queue_size--;// Floor at 0
			}
			uint16_t bp = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;
			//PRINTF("packetstack_length = %d\n",packetstack_len(&(c->send_stack)) );
			//PRINTF("virtual_queue_size = %d\n",c->virtual_queue_size);
			

			//int return_val = routingtable_update_routing(&c->routing_table, bp, &c->active_neighbor,&c->active_neighbor_backpressure,1,&c->prev_etx);
			
			//if(bp)
			{
				clock_time_t time1 = SEND_TIME_DELAY;
 				ctimer_set(&c->send_timer, time1, send_packet_pg, c);
			}
			//tx_flag=0;
			//send_beacon(c);
			return;
		}
		// if(tx_flag1)
		// 	PRINTF("Collision1. BOooMMM\n");
		q = packetstack_queuebuf(i);//q=i->buf
		
		if(q != NULL) 
		{
			c->sending = true;
			tx_flag=1;
			//tx_flag=0;
			
			queuebuf_to_packetbuf(q);// Place the queued packet into the packetbuf.
			uint16_t bp = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;
			//PRINTF("packetstack_length = %d\n",packetstack_len(&(c->send_stack)) );
			//PRINTF("virtual_queue_size = %d\n",c->virtual_queue_size);

			i->hdr.bcp_backpressure = bp;//PG
			if(OWNDEBUG)
			{
				if(i->hdr.bcp_backpressure>100)
					printf("Packet BP ERROR: %d",i->hdr.bcp_backpressure);
			}
			//PG
			if (i->hdr.timestamp != 0)
			{
				i->hdr.delay = i->hdr.delay + (clock_time() - i->hdr.timestamp);	
				i->hdr.timestamp=clock_time();
			}
			//i->hdr.tx_count=i->hdr.tx_count+1;	//PG
			
			if(COUNTED)
			{

				c->tx_count++;
				if(c->tx_count <= 1) // Start the link rate timer if this is not a retransmit
				{
					clock_time_t time1 = DELAY_TIME;
					timer_set(&c->delay_timer, time1);
				}
			}	
            
			// c->sending = true; Set sending to true

			//PRINTF("dd\n");
			c->current_packet = i;//&c->packet_not_acked;
			//memcpy(&c->packet_not_acked,i,sizeof(struct packetstack_item));

			// Set the currently sending packet to 'i'
			//ctimer_stop(&c->beacon_timer);// Stop the beaconing timer

			
			int return_val = routingtable_update_routing(&c->routing_table, bp, &c->active_neighbor,&c->active_neighbor_backpressure,1,&c->prev_etx);
			//PRINTF("etx=%u\n",c->prev_etx);
			if(return_val == RT_FAILURE) // Get the best neighbor
			{
				//PG
				/* TODO */
				clock_time_t time1 = RETRY_TIME;// Retransmission time
				//ctimer_stop(&c->send_timer);// Start the retransmission timer
				ctimer_set(&c->send_timer, time1, send_packet_pg, c); //Try sending again
				c->sending = false;
				//PG
				if(ctimer_expired(&c->beacon_timer))
					send_beacon(c);
				tx_flag=0;
				return;//quit sending
			}
			if(!COUNTED)
			{
				c->tx_count++;
				if(c->tx_count <= 1) // Start the link rate timer if this is not a retransmit
				{
					clock_time_t time1 = DELAY_TIME;
					timer_set(&c->delay_timer, time1);
				}
			}
	
			i->hdr.tx_count=i->hdr.tx_count+1;
			i->hdr.hop_count=i->hdr.hop_count+1;
			//i->hdr.path=(i->hdr.path)*10+rimeaddr_node_addr.u8[0];
			//PRINTF("%d\n",i->hdr.tx_count);
			memcpy(packetbuf_dataptr(), &(i->hdr), sizeof(struct data_hdr));
			struct data_hdr hdr;
			memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));

			packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &c->active_neighbor);
			//if(OWNDEBUG1)
				PRINTF("active_neighbor %d:%d\n",c->active_neighbor.u8[0],c->active_neighbor.u8[1]);
			//PG
	
			
			
            if(c->packet_not_acked->buf!=NULL)
            	queuebuf_free(c->packet_not_acked->buf);
            
            memset(c->packet_not_acked,0,sizeof(struct packetstack_item));
            memcpy(&(c->packet_not_acked->hdr),&hdr,sizeof(struct data_hdr));
            c->packet_not_acked->buf=queuebuf_new_from_packetbuf();
            c->packet_not_acked->ptr=c;
   			// PRINTF("from %d.%d with queue: %d  virtualqueue: %u timestamp %lu and delay %u ", c->packet_not_acked->hdr.origin.u8[0], c->packet_not_acked->hdr.origin.u8[1],packetstack_len(&(c->send_stack)) , c->virtual_queue_size, c->packet_not_acked->hdr.timestamp, c->packet_not_acked->hdr.delay);
			// PRINTF("bp %u ",bp);
		 	// PRINTF("neighbor-bp %lu\n",c->active_neighbor_backpressure);
			if(OWNDEBUG1)
				PRINTF("ERC %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);

			//tx_flag=1;
			broadcast_send(&c->broadcast_conn);
			
			if (rimeaddr_node_addr.u8[0]!=hdr.origin.u8[0])
			{
				c->send_time = clock_time();// Send time and transmission count
				PRINTF("P_Forwarding_packet_at_time: %7lu  X %u  ",  c -> send_time, hdr.origin_seq_no);
			}
			else
			{
				c->send_time = clock_time();// Send time and transmission count
				PRINTF("P_Sending_packet_at_time: %7lu   X %u  ", c -> send_time, hdr.origin_seq_no);
			}
			//memcpy(&c->packet_not_acked,i,sizeof(struct packetstack_item));

			PRINTF("from %d.%d with queue: %d  virtualqueue: %u timestamp %lu and delay %u ", hdr.origin.u8[0], hdr.origin.u8[1],packetstack_len(&(c->send_stack)) , c->virtual_queue_size, hdr.timestamp, hdr.delay);
			PRINTF("bp %u ",bp);
		    PRINTF("neighbor-bp %lu\n",c->active_neighbor_backpressure);
		    tx_flag=0;
		    first_send=1;
			//PRINTF("seqno=%u\n",c->current_packet->hdr.origin_seq_no);
			return;
		}
	}


	/*---------------------------------------------------------------------------*/
	static void send_beacon(void *ptr)
	{
		//PRINTF("send_beacon called %d\n",rimeaddr_node_addr.u8[0]);	
		rimeaddr_t addr_broadcast;  // Broadcast address
		addr_broadcast.u8[0] = 0;
		addr_broadcast.u8[1] = 0;
		struct bcp_conn *c = ptr;
		struct beacon_msg beacon;
		if(c->sending == false) 
			c->sending = true;
		else 
			return;
		packetbuf_clear();
		packetbuf_set_datalen(sizeof(struct beacon_msg));
		//beacon = packetbuf_dataptr();
		memset(&beacon, 0, sizeof(beacon));
		beacon.backpressure = packetstack_len(&(c->send_stack))+ c->virtual_queue_size;// Store the local backpressure level
		memcpy(packetbuf_dataptr(), &beacon, sizeof(struct beacon_msg));
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,PACKETBUF_ATTR_PACKET_TYPE_BEACON);
		packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &addr_broadcast);
		// Set the packet type
		if(OWNDEBUG)
			PRINTF("send beacon flag %d\n",beacon.backpressure);
		broadcast_send(&c->broadcast_conn);
		//PRINTF("send beacon flag 2\n");
	}
	/*---------------------------------------------------------------------------*/
	
	static void send_restrans(void *ptr)
	{
		//PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
			
		//PRINTF("send_re-packet_pg called\n");
		rimeaddr_t addr_broadcast;  // Broadcast address
		addr_broadcast.u8[0] = 0;
		addr_broadcast.u8[1] = 0;
		struct bcp_conn *c = ptr;
		//struct beacon_msg *beacon;
		//static rimeaddr_t addr;
		//addr.u8[0]=9;
		//addr.u8[1]=9;
		if(c->sending==false) 
			return;   // If we are not already sending the actual packet, just a safety condition
		
		struct queuebuf *q;// Queue Buffer struct
		struct packetstack_item *i;// Item in the packet stack
		i = c->packet_not_acked;// Grab the item on top of the stack
		// PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		ctimer_stop(&c->retransmission_timer);
		//ctimer_stop(&c->send_timer);
		//ctimer_stop(&c->beacon_timer);

		if(i == NULL) 
		{
			//PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
			c->sending=false;
			ctimer_reset(&c->beacon_timer);// Start beaconing
			clock_time_t time1 = SEND_TIME_DELAY;
			//PRINTF("bcp_send called\n");
		    ctimer_set(&c->send_timer, time1, send_packet_pg, c);
		    if(ctimer_expired(&c->beacon_timer))
		    	send_beacon(c);
			//ctimer_reset(&c->send_timer);
			return;
		}
		if(tx_flag1)
			PRINTF("Collision1. BOooMMM\n");
		q = packetstack_queuebuf(i);//q=i->buf

		if(q != NULL) 
		{
			tx_flag=1;
			queuebuf_to_packetbuf(q);// Place the queued packet into the packetbuf.
			//PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);

			uint16_t bp = packetstack_len(&(c->send_stack)) + c->virtual_queue_size;
			
			i->hdr.bcp_backpressure = bp;//packetstack_len(&(c->send_stack)) + c->virtual_queue_size;//PG
			
			//PG
			if (i->hdr.timestamp != 0) 
			{
				 i->hdr.delay = i->hdr.delay + (clock_time() - i->hdr.timestamp);	
				 i->hdr.timestamp=clock_time();	
			}
			//PG
			
			int return_val = routingtable_update_routing(&c->routing_table, bp, &c->active_neighbor,&c->active_neighbor_backpressure,c->tx_count+1,&c->prev_etx);
			
			if(return_val == RT_FAILURE) // Get the best neighbor
			{
				clock_time_t time1 = RETRY_TIME;// Retransmission time
				//ctimer_stop(&c->retransmission_timer);// Start the retransmission timer
				ctimer_set(&c->retransmission_timer, time1, send_restrans, c);
				//ctimer_stop(&c->retransmission_timer);
		        //clock_time_t time2 = SEND_TIME_DELAY;
				//PRINTF("bcp_send called\n");
				//ctimer_set(&c->send_timer, time2, send_packet_pg, c);
				if(ctimer_expired(&c->beacon_timer))
					send_beacon(c);
				tx_flag=0;
				return;//quit sending
			}
			if(!COUNTED)
				c->tx_count++;
			if(i->hdr.hop_count==0)
			{
				//i->hdr.path=(i->hdr.path)*10+rimeaddr_node_addr.u8[0];
				i->hdr.hop_count++;
			}
			i->hdr.tx_count=i->hdr.tx_count+1;
			memcpy(packetbuf_dataptr(), &(i->hdr), sizeof(struct data_hdr));
			//c->tx_count++;
			//c->sending = true;// Set sending to true
			//c->current_packet = i;// Set the currently sending packet to 'i'
			//ctimer_stop(&c->beacon_timer);// Stop the beaconing timer

			//PG
			
			packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &c->active_neighbor);
			
			struct data_hdr hdr;
			memcpy(&hdr, packetbuf_dataptr(), sizeof(struct data_hdr));
		
			//if(OWNDEBUG1)
			PRINTF("active_neighbor %d:%d\n",c->active_neighbor.u8[0],c->active_neighbor.u8[1]);

			
		   
			if(c->packet_not_acked->buf!=NULL)
            	queuebuf_free(c->packet_not_acked->buf);
            
            memset(c->packet_not_acked,0,sizeof(struct packetstack_item));
            memcpy(&(c->packet_not_acked->hdr),&hdr,sizeof(struct data_hdr));
            c->packet_not_acked->buf=queuebuf_new_from_packetbuf();
            c->packet_not_acked->ptr=c;

			if(OWNDEBUG1)
				PRINTF("ERC %d:%d\n",(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[0],(packetbuf_addr(PACKETBUF_ADDR_ERECEIVER))->u8[1]);
		   	//PRINTF("Before retransmission\n");
			//packetstack_pop(&c->send_stack);
			//memcpy(&c->current_packet,i,sizeof(struct packetstack_item));
			//PRINTF("seqno1=%u\n",i->hdr.origin_seq_no);
			
			broadcast_send(&c->broadcast_conn);
			if (!(rimeaddr_cmp(&rimeaddr_node_addr,&hdr.origin)))
			{
				c->send_time = clock_time();// Send time and transmission count
				PRINTF("P_Forwarding_packet_at_time(RE): %7lu  X %u  ",  c -> send_time, hdr.origin_seq_no);
				//PRINTF("from %d.%d with queue: %d  virtualqueue: %d timestamp %u and delay %u \n", hdr.origin.u8[0], hdr.origin.u8[1],packetstack_len(&(c->send_stack))+1 , c->virtual_queue_size, hdr.timestamp, hdr.delay);
				// PRINTF("%d\n",hdr.tx_count);
			}
			else
			{
				c->send_time = clock_time();// Send time and transmission count
				PRINTF("P_Sending_packet_at_time(RE): %7lu   X %u  ", c -> send_time, hdr.origin_seq_no);
				//PRINTF("from %d.%d with queue: %d  virtualqueue: %d timestamp %u and delay %u \n", hdr.origin.u8[0], hdr.origin.u8[1],packetstack_len(&(c->send_stack))+1 , c->virtual_queue_size, hdr.timestamp, hdr.delay);
				// PRINTF("%d\n",hdr.tx_count);
			}
			PRINTF("from %d.%d with queue: %d  virtualqueue: %u timestamp %lu and delay %u ", hdr.origin.u8[0], hdr.origin.u8[1],packetstack_len(&(c->send_stack)) , c->virtual_queue_size, hdr.timestamp, hdr.delay);
			PRINTF("bp %u ",bp);
		    PRINTF("neighbor-bp %lu\n",c->active_neighbor_backpressure);
			tx_flag=0;
			//PRINTF("seqno=%u\n",c->current_packet->hdr.origin_seq_no);
			return;
		}
		//PRINTF("After retransmission\n");
		//PRINTF("%d.%d: Nothing on send stack, so send a null packet and count down virtual queue\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
		
	}

	static void retransmit_callback(void *ptr)
	{
		//PG
		//PRINTF("retransmit_callback\n");	
		//
		struct bcp_conn *c = ptr;
		//c->sending = false;
		routingtable_update_link_status(&c->routing_table, &c->active_neighbor, false);
		if(c->tx_count >= MAX_ACK_REXMITS) 
		{
			PRINTF("P_PACKET TIMED OUT, DROPPING %d\n",(c->current_packet->hdr).origin_seq_no);
			//packetstack_remove(&c->send_stack, c->current_packet);// Drop the packet
			//uint32_t link_estimate_time = DELAY_TIME- timer_remaining(&c->delay_timer);
			//PRINTF("link_estimate_time= %d\n",link_estimate_time);//also in ticks
			//PRINTF("tx_count= %d(in unicast where it will be used)\n",bcp_conn->tx_count);
			//routingtable_update_link_rate(&c->routing_table, &c->active_neighbor,link_estimate_time);
		
			//routingtable_update_link_success(&c->routing_table, &c->active_neighbor, c->tx_count);
			c->tx_count = 0;
			if(c->current_packet)
				packetstack_remove(&c->send_stack,c->current_packet);

			c->current_packet = NULL;
			c->sending = false;

			if(ctimer_expired(&(c->send_timer))) // Reset the send data timer
			{
						send_packet_pg(c);
			}

		} 
		else
		{
			if(COUNTED)
		 		c->tx_count++;
		 	send_restrans(c);
		}
		//PG
	}

	/*---------------------------------------------------------------------------*/
	void bcp_open(struct bcp_conn *c, uint16_t channel,const struct bcp_callbacks *callbacks)
	{
		//PRINTF("bcp_open called\n");
		c->cb = callbacks;// Assign the callbacks to the connection struct
		//PRINTF("bcp_open flag 1\n");
		LIST_STRUCT_INIT(c, send_stack_list);// Initialize the send stack LIST
		//PRINTF("(in bcp_open)tx_count= %d\n",c->tx_count);
		LIST_STRUCT_INIT(c, routing_table_list);// Initialize the routing table of neighbors LIST
		c->send_stack.list = &(c->send_stack_list);// Initialize send stack
		c->send_stack.memb = &send_stack_memb;
		c->routing_table.list = &(c->routing_table_list);// Initialize routing table
		c->routing_table.memb = &routing_table_memb;
		c->packet_not_acked=memb_alloc(&recent_pkt);
		c->packet_not_acked->buf=NULL;
		c->virtual_queue_size = 0;
		c->prev_etx=1000;// Init virtual queue
		rimeaddr_t addr;// Init active neighbor
		addr.u8[0] = 0;
		addr.u8[1] = 0;
		rimeaddr_copy(&c->active_neighbor, &addr);
		//PRINTF("bcp_open flag 2\n");
		broadcast_open(&c->broadcast_conn, channel, &broadcast_callbacks);// Open the broadcast connection for data packets and beacons
		channel_set_attributes(channel, attributes);
		//PRINTF("bcp_open flag 3\n");
		unicast_open(&c->unicast_conn, channel + 1, &unicast_callbacks);// Open the unicast connection for ACKs (channel + 1)
		channel_set_attributes(channel + 1, attributes);
		//PRINTF("bcp_open flag 4\n");
		send_beacon(c);// Start Beaconing
		//PRINTF("bcp_open flag 5\n");
	}
	/*---------------------------------------------------------------------------*/
	void bcp_close(struct bcp_conn *c)
	{
		broadcast_close(&c->broadcast_conn);// Close the broadcast connection
		unicast_close(&c->unicast_conn);// Close the unicast connection
		ctimer_stop(&c->send_timer);// Stop the timers
		ctimer_stop(&c->beacon_timer);
		while(packetstack_top(&c->send_stack) != NULL) // Empty packet stack
		{
			packetstack_pop(&c->send_stack);
		}
	}
	/*---------------------------------------------------------------------------*/
	int bcp_send(struct bcp_conn *c)
	{

		//PRINTF("bcp_send called\n");
		struct packetstack_item *i;
		//PRINTF("my addr= %d\n",rimeaddr_node_addr);
		// static rimeaddr_t addr2,addr4;
		// addr2.u8[0] = 2;
		// addr2.u8[1] = 0;
		// addr4.u8[0] = 4;
		// addr4.u8[1] = 0;
		//if (rimeaddr_node_addr.u8[0]==addr2.u8[0]) goto JUMP;
		//if (rimeaddr_node_addr.u8[0]==addr4.u8[0]) goto JUMP;
		//packetbuf_hdralloc(sizeof(struct data_msg_hdr));
		packetbuf_clear();
		packetbuf_hdralloc(sizeof(struct data_hdr));// Allocate space for the header.
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_PACKET_TYPE_DATA);
		i = push_data_packet(c);
		if(i != NULL) 
		{
			rimeaddr_copy(&(i->hdr.origin), &rimeaddr_node_addr);// Set the origin of the packet
			s_no++;
			i->hdr.origin_seq_no = s_no;
			i->hdr.hop_count=0;//PG
			i->hdr.timestamp = clock_time();
			i->hdr.delay=0;
			i->hdr.tx_count=0;
			//i->hdr.path=0;
			//ctimer_stop(&c->beacon_timer);// We have data to send, stop beaconing
			if(ctimer_expired(&c->send_timer)&&(!c->sending)) // Reset the send data timer
			{
				clock_time_t time1 = SEND_TIME_DELAY;
				//PRINTF("bcp_send called\n");
				ctimer_set(&c->send_timer, time1, send_packet_pg, c);
				//PRINTF("bcp_send flag 2\n");
			}

			return 1;
		}	
		if(ctimer_expired(&c->send_timer)&&(!c->sending))// Reset the send data timer
		{
			clock_time_t time1 = SEND_TIME_DELAY;
		 	//PRINTF("bcp_send flag 3\n");
		 	ctimer_set(&c->send_timer, time1, send_packet_pg, c);
		 	//PRINTF("bcp_send flag 4\n");
	 	}
	 	return 0;
	}
	/*---------------------------------------------------------------------------*/
	static void send_ack(struct bcp_conn *bc, const rimeaddr_t *to, const rimeaddr_t *from, uint16_t seq_no)
	{
		// if(tx_flag)
		// 	PRINTF("Collision. BOooMMM\n");
		tx_flag1=1;
		//PRINTF("send_ack called\n");
		struct ack_msg ack;
		//PRINTF("fff:%d\n",seq_no);
		packetbuf_clear();
		packetbuf_set_datalen(sizeof(struct ack_msg));
		//ack = packetbuf_dataptr();
		memset(&ack, 0, sizeof(struct ack_msg));
		rimeaddr_copy(&(ack.origin), from);
		ack.id=seq_no;
		memcpy(packetbuf_dataptr(),&ack,sizeof(struct ack_msg));
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_ID, seq_no);
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE, PACKETBUF_ATTR_PACKET_TYPE_ACK);
		//PRINTF("send_ack flag 1\n");
		unicast_send(&bc->unicast_conn, to);
		tx_flag1=0;
		//PRINTF("send_ack flag 2\n");
	}
	/*---------------------------------------------------------------------------*/
	void bcp_set_sink(struct bcp_conn *c, const rimeaddr_t *addr)
	{
		if(rimeaddr_cmp(addr, &rimeaddr_node_addr)) // This node IS a sink node
		{  
			//PRINTF("bcp_set_sink: %d.%d set as sink node.\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
			c->is_sink = true;
			c->virtual_queue_size = 0;// As soon as we are root, should have zero virtual queue
		} 
		else //THIS node is not a sink node
		{ 
			c->is_sink = false;
		}
	}
	/*---------------------------------------------------------------------------*/


