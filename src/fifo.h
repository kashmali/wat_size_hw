#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct
{
  uint32_t max_size;
  uint8_t *buf;
  uint32_t size;
  uint32_t idx; // Last location with valid data
  uint32_t start; // First location with valid data
} fifo_t;

uint8_t fifo_init(fifo_t *f, uint8_t *buf, uint32_t size);
uint8_t fifo_push(fifo_t *f, uint8_t data);
uint8_t fifo_peek(fifo_t *f, uint8_t *out);
uint8_t fifo_pop(fifo_t *f, uint8_t *out);
uint32_t fifo_flush(fifo_t *f);
uint32_t fifo_remaining_space(fifo_t *f);

#endif
