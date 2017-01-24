// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#include "contiki.h"
#include "net/rime.h"
#include "net/rime/timesynch.h"
#include "bcp.h"
#include "dev/cc2420.h"
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static void recv_bcp(struct bcp_conn *c,struct data_hdr hdr)
{
//  PRINTF("DEBUG: Inside BCP callback.\n");
printf("Hurray! Reached Sink.Received packets.Origin = %d.%d Seq no = %d\n",hdr.origin.u8[0],hdr.origin.u8[1],hdr.origin_seq_no);
}

static const struct bcp_callbacks bcp_callbacks = { recv_bcp };
static struct bcp_conn bcp;

/*---------------------------------------------------------------------------*/
PROCESS(main_process, "Main process LIFO with virtual queue pps=0.25");
AUTOSTART_PROCESSES(&main_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(main_process, ev, data)
{

  PROCESS_BEGIN();
  // rime_init();
  timesynch_init();
  timesynch_set_authority_level(rimeaddr_node_addr.u8[0]);
  static struct etimer timer,timer1;
  static rimeaddr_t addr,addr1;
 int txpower;
  addr.u8[0] = 1;
  addr.u8[1] = 0;
  addr1.u8[0]=40;
  addr1.u8[1]=0;

  bcp_set_sink(&bcp, &addr);
  bcp_open(&bcp, 140, &bcp_callbacks);

  cc2420_set_txpower(15);
  // txpower = cc2420_get_txpower();
  etimer_set(&timer1, 120 * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&timer1));
  //PRINTF("2\n");
	if(rimeaddr_cmp(&addr, &rimeaddr_node_addr)) 
	{ 
    //this is a sink node
    while(1) 
		{
		  //PRINTF("3\n");
      PROCESS_WAIT_EVENT();
    }
  } 
	else 
	{
		//PRINTF("4\n");
    // etimer_set(&timer1, CLOCK_SECOND*60);
    // PROCESS_WAIT_EVENT();
    etimer_set(&timer, 1*CLOCK_SECOND);
    //   etimer_set(&timer1, 300*CLOCK_SECOND);
    while(1) 
		{
 	    PROCESS_WAIT_EVENT();
		  //PRINTF("5\n");
	
		  if(ev == PROCESS_EVENT_TIMER) 
			{
        // if(ctimer_expired(&timer1))
        //     break;
		    addr.u8[0] = 1;
 		    addr.u8[1] = 0;
        //PRINTF("6\n");
 		 //    if(!rimeaddr_cmp(&addr, &rimeaddr_node_addr)) 
				// {
				//   //PRINTF("7\n");            
				//   bcp_send(&bcp);
    //  		}
         if(rimeaddr_node_addr.u8[0]>28) 
         {
            //PRINTF("7\n");            
            bcp_send(&bcp);
         }
  			//PRINTF("8\n");
        etimer_reset(&timer);
  			//PRINTF("9\n");
      }
    }
  }
  PROCESS_END();
}
