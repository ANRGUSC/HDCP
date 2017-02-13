// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details


#include "contiki.h"
#include "net/rime.h"
#include "net/rime/timesynch.h"
#include "hdcp.h"
#include "dev/cc2420.h"
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static void recv_hdcp(struct hdcp_conn *c,struct data_hdr hdr)
{
//  PRINTF("DEBUG: Inside HDCP callback.\n");
printf("Hurray! Reached Sink.Received packets.Origin = %d.%d Seq no = %d\n",hdr.origin.u8[0],hdr.origin.u8[1],hdr.origin_seq_no);
}

static const struct hdcp_callbacks hdcp_callbacks = { recv_hdcp };
static struct hdcp_conn hdcp;

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
    static struct etimer timer,timer1,timer2;
    static rimeaddr_t addr,addr1;
    int txpower;
    addr.u8[0] = 1;
    addr.u8[1] = 0;
    addr1.u8[0]=40;
    addr1.u8[1]=0;

    hdcp_set_sink(&hdcp, &addr);
    hdcp_open(&hdcp, 140, &hdcp_callbacks);

    // cc2420_set_txpower(20);
    // txpower = cc2420_get_txpower();
    etimer_set(&timer1, 120 * CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&timer1));
    // cc2420_set_txpower(19);
    //PRINTF("2\n");
    if(rimeaddr_cmp(&addr, &rimeaddr_node_addr)) 
    { 
        // cc2420_set_txpower(11);
        // etimer_set(&timer, 2*CLOCK_SECOND);
        //this is a sink node
        while(1) 
        {
            //PRINTF("3\n");  
            PROCESS_WAIT_EVENT();
            // if(ev == PROCESS_EVENT_TIMER) 
            // {

            //     if(etimer_expired(&timer2))
            //     {
                    // etimer_set(&timer2, 300*CLOCK_SECOND);
            //         txpower = cc2420_get_txpower();
            //         if(txpower<27)
            //             txpower=txpower+5;
            //         cc2420_set_txpower(txpower);
                    // printf("txpower %d\n",txpower);

            //     }
            // }
         }
    } 
    else 
    {
        //PRINTF("4\n");
        // etimer_set(&timer1, CLOCK_SECOND*60);
        // PROCESS_WAIT_EVENT();
        etimer_set(&timer, 4*CLOCK_SECOND);
        //   etimer_set(&timer1, 300*CLOCK_SECOND);
        while(1) 
        {
            PROCESS_WAIT_EVENT();
            //PRINTF("5\n");
    
            if(ev == PROCESS_EVENT_TIMER) 
            {

                // if(etimer_expired(&timer2))
                // {
                //     etimer_set(&timer2, 300*CLOCK_SECOND);
                   // txpower = cc2420_get_txpower();
                //     if(txpower<27)
                //         txpower=txpower+5;
                //     cc2420_set_txpower(txpower);
                    //printf("txpower %d\n",txpower);

                // }
                // if(ctimer_expired(&timer1))
                //     break;
                addr.u8[0] = 1;
                addr.u8[1] = 0;
                //PRINTF("6\n");
                // if(!rimeaddr_cmp(&addr, &rimeaddr_node_addr)) 
                // {
                //     //PRINTF("7\n");            
                //     hdcp_send(&hdcp);
                // }
                if(rimeaddr_node_addr.u8[0]>28) 
                {
                    //PRINTF("7\n");            
                    hdcp_send(&hdcp);
                }
                //PRINTF("8\n");
                etimer_reset(&timer);
                //PRINTF("9\n");
            }
        }
    }
  PROCESS_END();
}
