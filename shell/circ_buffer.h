/*
 * circ_buffer.h
 *
 * Created: 12/8/2016 4:04:57 PM
 *  Author: hkim
 */ 


#ifndef CIRC_BUFFER_H_
#define CIRC_BUFFER_H_

#include <stdio.h>
#include <stdint.h>
#include "app_common.h"

typedef struct __circ_buffer CircBuffer;

typedef void (*circ_buf_enter_critical)(CircBuffer* cb);
typedef void (*circ_buf_leave_critical)(CircBuffer* cb);

struct __circ_buffer
{
  volatile uint8_t*     buffer;
  volatile uint16_t     capacity;
  volatile uint16_t     num_bytes;
  volatile uint16_t     begin;
  volatile uint16_t     end;

  circ_buf_enter_critical   enter_critical;
  circ_buf_leave_critical   leave_critical;
} ;

extern void circ_buffer_init(CircBuffer* cb, volatile uint8_t* buffer, uint16_t capacity,
    circ_buf_enter_critical enter_critical,
    circ_buf_leave_critical leave_critical);

extern uint8_t circ_buffer_enqueue(CircBuffer* cb, uint8_t* data, uint16_t size, uint8_t from_isr);
extern uint8_t circ_buffer_dequeue(CircBuffer* cb, uint8_t* data, uint16_t size, uint8_t from_isr);
extern uint8_t circ_buffer_first(CircBuffer* cb, uint8_t* data);
extern uint8_t circ_buffer_last(CircBuffer* cb, uint8_t* data);

static inline uint8_t
circ_buffer_is_empty(CircBuffer* cb, uint8_t from_isr)
{
  uint8_t ret;

  if(from_isr)
  {
    return cb->num_bytes == 0 ? true : false;
  }

  if(cb->enter_critical != NULL)
  {
    cb->enter_critical(cb);
  }

  ret = cb->num_bytes == 0 ? true : false;

  if(cb->leave_critical != NULL)
  {
    cb->leave_critical(cb);
  }

  return ret;
}

static inline uint8_t
circ_buffer_is_full(CircBuffer* cb, uint8_t from_isr)
{
  uint8_t ret;

  if(from_isr)
  {
    return cb->num_bytes == 0 ? true : false;
  }

  if(cb->enter_critical != NULL)
  {
    cb->enter_critical(cb);
  }

  ret = cb->num_bytes >= cb->capacity ? true : false;

  if(cb->leave_critical != NULL)
  {
    cb->leave_critical(cb);
  }

  return ret;
}

#endif /* CIRC_BUFFER_H_ */
