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
 * @file        main.c
 * @brief       Example of using HDCP routing libraries
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 * 
 */


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

#define TX_POWER_CH 31 //Choose from {31, 27, 23, 19, 15, 11 ,7 ,3}

static void recv_hdcp(struct hdcp_conn *c,struct data_hdr hdr)
{
    printf("Hurray! Reached Sink.Received packets.Origin = %d.%d Seq no = %d\n",
                        hdr.origin.u8[0],hdr.origin.u8[1],hdr.origin_seq_no);
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
    timesynch_init();
    timesynch_set_authority_level(rimeaddr_node_addr.u8[0]);
    
    static struct etimer timer, timer1;
    static rimeaddr_t addr;
    
    addr.u8[0] = 1;
    addr.u8[1] = 0;

    hdcp_set_sink(&hdcp, &addr); // Set the node sink node
    hdcp_open(&hdcp, 140, &hdcp_callbacks); // Open a hdcp connection

    cc2420_set_txpower(TX_POWER_CH); // Set the channel Power

    etimer_set(&timer1, 120 * CLOCK_SECOND); // Initial Setup Delay of 2 minutes 
    PROCESS_WAIT_UNTIL(etimer_expired(&timer1));


    if (rimeaddr_cmp(&addr, &rimeaddr_node_addr))  // Check whether the node is sink
    { 
        while(1) 
            PROCESS_WAIT_EVENT(); // Just wait for packets
    } 
    else 
    {
        etimer_set(&timer, 4*CLOCK_SECOND); // Send a Packet every 4 second

        while(1) 
        {
            PROCESS_WAIT_EVENT();
   
            if(ev == PROCESS_EVENT_TIMER) 
            {
                hdcp_send(&hdcp);
                etimer_reset(&timer);
            }
        }
    }
    PROCESS_END(); // Should Not Reach This
}
