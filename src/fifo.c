#include "fifo.h"

// Initialize a fifo with the underlying buffer buf with size
uint8_t fifo_init(fifo_t *f, uint8_t *buf, uint32_t size)
{
  f->max_size = size;
  f->buf = buf;
  f->size = 0;
  f->idx = 0;
  f->start = 1;
}

// Push a single element onto the fifo
uint8_t fifo_push(fifo_t *f, uint8_t data)
{
  if(f && (f->size != f->max_size))
  {
    if(f->idx == f->max_size - 1) // if equal to the last element
    {
      f->idx = 0;
    }
    else
    {
      f->idx++;
    }

    // Add the data to the updated idx
    f->buf[f->idx] = data;
    // Increment the size
    f->size++;
  }
  else
  {
    //error
    return 1;
  }

  return 0;
}

uint8_t fifo_peek(fifo_t *f, uint8_t *out)
{
  if(f && f->size != 0)
  {
    *out = f->buf[f->start];
    return 0;
  }
  *out = 0xFF;
  return 1;
}

uint8_t fifo_pop(fifo_t *f, uint8_t *out)
{
  *out = 0xFF;
  if(f && f->size != 0)
  {
    *out = f->buf[f->start];
    if(f->start == f->max_size - 1) // if equal to the last element
    {
      f->start = 0;
    }
    else
    {
      f->start++;
    }
    f->size--;
  }
  else
  {
    // error
    return 1;
  }
  return 0;
}

uint32_t fifo_remaining_space(fifo_t *f)
{
  if(f)
  {
    return f->max_size - f->size;
  }
  return 0;
}

