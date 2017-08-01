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
 * @file        packetstack.c
 * @brief       Main packetstack library for HDCP from Contiki
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Bhaskar Krishnamachari  <bkrishna@usc.edu> 
 *  
 */


#include "packetstack.h"
#define FIFO 0
/*---------------------------------------------------------------------------*/
/**
 * Intitalized a packet stack
 * @param s             pointer the stack
 */
void packetstack_init(struct packetstack *s)
{
  list_init(*s->list);
  memb_init(s->memb);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief  pushed a new object into the stack
 * @param  s             pointer the stack
 * @param  ptr           pointer to the object 
 * @param  p             return status
 * @return               pointer to the top object on the stack 
 */
struct packetstack_item *
packetstack_push_packetbuf(struct packetstack *s, void *ptr, uint16_t *p)
{
  struct packetstack_item *i;
  // Allocate a memory block to hold the packet stack item
  i = memb_alloc(s->memb);
  ////PRINTF("%d mmm %d \n",s->memb->size,s->memb->num);
  if (i == NULL) 
  {
    if (!FIFO)
    {
      //PRINTF("P_QUEUE DROPPING\n");
      packetstack_remove(s,list_tail(*s->list));
      *p = 1;
      RE: i=memb_alloc(s->memb);
    }
    else
    {
      *p = 1;
      return NULL;
    }
  }

  // Allocate a queuebuf and copy the contents of the packetbuf into it
  i->buf = queuebuf_new_from_packetbuf();

  if(i->buf == NULL)
  {
    packetstack_remove(s,list_tail(*s->list));
    *p = 1;
    goto RE;
  }

  i->stack = s;
  i->ptr = ptr;

  // Add the item to the stack
  list_push(*s->list, i);
  
  return packetstack_top(s);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief retuitrn a pointer to the most recently pushed opject
 * @param s             pointer the stack
 * @return              pointer to the top object on the stack
 */
struct packetstack_item *
packetstack_top(struct packetstack *s)
{
  return list_head(*s->list);
}

/**
 * @brief returns a pointer to oldest pushed opject
 * @param s             pointer the stack
 * @return              pointer to the last object on the stack
 */
struct packetstack_item *
packetstack_bottom(struct packetstack *s)
{
  return list_tail(*s->list);
}
/**
 * @brief pops a packet from packet stack
 * @param s             pointer the stack
 */
void packetstack_pop(struct packetstack *s)
{
  struct packetstack_item *i;

  i = list_head(*s->list);
  if(i != NULL) 
  {
    list_pop(*s->list);
    queuebuf_free(i->buf);
    memb_free(s->memb, i);
  }
}
/**
 * @brief removes a packet from the packet stack
 * @param s             pointer the stack
 * @param i             pointer to the item to be removed
 */
void packetstack_remove(struct packetstack *s, struct packetstack_item *i)
{
  if(i != NULL) 
  {
    list_remove(*s->list, i);
    queuebuf_free(i->buf);
    memb_free(s->memb, i);
  }
}
/**
 * @brief Get the length of the packet stack
 * @param s             pointer the stack
 * @return              length of the stack
 */
int packetstack_len(struct packetstack *s)
{
  return list_length(*s->list);
}
/**
 * @brief returns the buffered object pointed by the stack object
 * @param  i            the stack object
 * @return              pointer to the buffered object
 */
struct queuebuf *
packetstack_queuebuf(struct packetstack_item *i)
{
  if(i != NULL) 
    return i->buf;
  else 
    return NULL;
}
