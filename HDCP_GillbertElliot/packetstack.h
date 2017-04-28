/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh
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
 * @file        packetstack.h
 * @brief       Main packetstack library for HDCP from Contiki
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#ifndef __PACKETSTACK_H__
#define __PACKETSTACK_H__

#include "lib/list.h"
#include "lib/memb.h"

#include "net/packetbuf.h"
#include "net/queuebuf.h"

struct packetstack 
{
  list_t *list;
  struct memb *memb;
};

struct data_hdr 
{
  uint16_t hdcp_backpressure;   // Node backpressure measurement for neighbors
  uint16_t tx_count;           // Total transmission count experienced by the packet
  uint32_t timestamp;         //timestamp
  uint16_t delay;                       //delay
  rimeaddr_t origin;
  uint16_t hop_count;     // End-to-end hop count experienced by the packet
  uint16_t origin_seq_no;
};

struct packetstack_item 
{
  struct packetstack_item *next;
  struct queuebuf *buf;
  struct packetstack *stack;
  void *ptr;

  // HDCP Header
  struct data_hdr hdr;
};

#define PACKETSTACK(name, size) LIST(name##_list); \
                                MEMB(name##_memb, struct packetstack_item, size); \
        static struct packetstack name = { &name##_list, \
                   &name##_memb }


/**
 * Intitalized a packet stack
 * @param s             pointer the stack
 */
void packetstack_init(struct packetstack *s);

/**
 * @brief retuitrn a pointer to the most recently pushed opject
 * @param s             pointer the stack
 * @return              pointer to the top object on the stack
 */
struct packetstack_item *packetstack_top(struct packetstack *s);

/**
 * @brief returns a pointer to oldest pushed opject
 * @param s             pointer the stack
 * @return              pointer to the last object on the stack
 */
struct packetstack_item *packetstack_bottom(struct packetstack *s);

/**
 * @brief  pushed a new object into the stack
 * @param  s             pointer the stack
 * @param  ptr           pointer to the object 
 * @param  p             return status
 * @return               pointer to the top object on the stack 
 */
struct packetstack_item *packetstack_push_packetbuf(struct packetstack *s,
                                                    void *ptr, uint16_t *p);

/**
 * @brief pops a packet from packet stack
 * @param s             pointer the stack
 */
void packetstack_pop(struct packetstack *s);

/**
 * @brief removes a packet from the packet stack
 * @param s             pointer the stack
 * @param i             pointer to the item to be removed
 */
void packetstack_remove(struct packetstack *s, struct packetstack_item *i);

/**
 * @brief Get the length of the packet stack
 * @param s             pointer the stack
 * @return              length of the stack
 */
int packetstack_len(struct packetstack *s);

/**
 * @brief returns the buffered object pointed by the stack object
 * @param  i            the stack object
 * @return              pointer to the buffered object
 */
struct queuebuf *packetstack_queuebuf(struct packetstack_item *i);

#endif /* __PACKETSTACK_H__ */

/** @} */
/** @} */
