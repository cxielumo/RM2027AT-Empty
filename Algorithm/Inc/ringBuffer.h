#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *storage;
    size_t capacity;
    size_t read_pos;
    size_t write_pos;
    size_t used;
    uint32_t dropped_bytes;
} ringBuffer_t;

int ringBuffer_init(ringBuffer_t *rb, uint8_t *storage, size_t capacity);
void ringBuffer_reset(ringBuffer_t *rb);
size_t ringBuffer_write(ringBuffer_t *rb, const uint8_t *src, size_t length);
size_t ringBuffer_read(ringBuffer_t *rb, uint8_t *dst, size_t capacity);
size_t ringBuffer_size(const ringBuffer_t *rb);
size_t ringBuffer_free(const ringBuffer_t *rb);
uint32_t ringBuffer_dropped(const ringBuffer_t *rb);

#endif /* RING_BUFFER_H */
