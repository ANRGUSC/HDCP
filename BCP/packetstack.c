// Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
// contributor: Pradipta Ghosh
// read license file in main directory for more details

#include "packetstack.h"
#define FIFO 0
/*---------------------------------------------------------------------------*/
/**
 * [packetstack_init description]
 * @param s [description]
 */
void packetstack_init(struct packetstack *s)
{
  list_init(*s->list);
  memb_init(s->memb);
}
/**
 * [packetstack_push_packetbuf description]
 * @param  s   [description]
 * @param  ptr [description]
 * @param  p   [description]
 * @return     [description]
 */
struct packetstack_item *
packetstack_push_packetbuf(struct packetstack *s, void *ptr, uint16_t *p)
{
  struct packetstack_item *i;
  // Allocate a memory block to hold the packet stack item
  i = memb_alloc(s->memb);
  ////PRINTF("%d mmm %d \n",s->memb->size,s->memb->num);
  if(i == NULL) 
  {
    if(!FIFO)
    {
      //PRINTF("P_QUEUE DROPPING\n");
      packetstack_remove(s,list_tail(*s->list));
      *p = 1;
      RE:i=memb_alloc(s->memb);
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
    //PRINTF("P_QUEUE DROPPING_1\n");
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
/**
 * [packetstack_top description]
 * @param  s [description]
 * @return   [description]
 */
struct packetstack_item *packetstack_top(struct packetstack *s)
{
  return list_head(*s->list);
}

/**
 * [packetstack_bottom description]
 * @param  s [description]
 * @return   [description]
 */
struct packetstack_item *packetstack_bottom(struct packetstack *s)
{
  return list_tail(*s->list);
}
/**
 * [packetstack_pop description]
 * @param s [description]
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
 * [packetstack_remove description]
 * @param s [description]
 * @param i [description]
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
 * [packetstack_len description]
 * @param  s [description]
 * @return   [description]
 */
int packetstack_len(struct packetstack *s)
{
  return list_length(*s->list);
}
/**
 * [packetstack_queuebuf description]
 * @param  i [description]
 * @return   [description]
 */
struct queuebuf *packetstack_queuebuf(struct packetstack_item *i)
{
  if(i != NULL) 
  {
    return i->buf;
  } 
  else 
  {
    return NULL;
  }
}
