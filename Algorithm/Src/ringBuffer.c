#include "ringBuffer.h"

#include <string.h>

/*
 * The ring buffer owns no memory and is deliberately not synchronized.
 * Callers must serialize every operation that accesses the same instance.
 */
static bool ringBuffer_isValid(const ringBuffer_t *rb)
{
    return (rb != NULL) &&
           (rb->storage != NULL) &&
           (rb->capacity > 0U) &&
           (rb->read_pos < rb->capacity) &&
           (rb->write_pos < rb->capacity) &&
           (rb->used <= rb->capacity);
}

static size_t ringBuffer_copyIn(ringBuffer_t *rb,
                                const uint8_t *src,
                                size_t length)
{
    size_t first_length = rb->capacity - rb->write_pos;

    if (first_length > length) {
        first_length = length;
    }

    if (first_length > 0U) {
        (void)memcpy(&rb->storage[rb->write_pos], src, first_length);
    }

    {
        const size_t second_length = length - first_length;

        if (second_length > 0U) {
            (void)memcpy(rb->storage, &src[first_length], second_length);
        }
    }

    if (length >= (rb->capacity - rb->write_pos)) {
        rb->write_pos = length - (rb->capacity - rb->write_pos);
    } else {
        rb->write_pos += length;
    }

    return length;
}

static size_t ringBuffer_copyOut(ringBuffer_t *rb,
                                 uint8_t *dst,
                                 size_t length)
{
    size_t first_length = rb->capacity - rb->read_pos;

    if (first_length > length) {
        first_length = length;
    }

    if (first_length > 0U) {
        (void)memcpy(dst, &rb->storage[rb->read_pos], first_length);
    }

    {
        const size_t second_length = length - first_length;

        if (second_length > 0U) {
            (void)memcpy(&dst[first_length], rb->storage, second_length);
        }
    }

    if (length >= (rb->capacity - rb->read_pos)) {
        rb->read_pos = length - (rb->capacity - rb->read_pos);
    } else {
        rb->read_pos += length;
    }

    rb->used -= length;
    return length;
}

int ringBuffer_init(ringBuffer_t *rb, uint8_t *storage, size_t capacity)
{
    if ((rb == NULL) || (storage == NULL) || (capacity == 0U)) {
        return -1;
    }

    rb->storage = storage;
    rb->capacity = capacity;
    rb->read_pos = 0U;
    rb->write_pos = 0U;
    rb->used = 0U;
    rb->dropped_bytes = 0U;

    return 0;
}

void ringBuffer_reset(ringBuffer_t *rb)
{
    if (!ringBuffer_isValid(rb)) {
        return;
    }

    rb->read_pos = 0U;
    rb->write_pos = 0U;
    rb->used = 0U;
    rb->dropped_bytes = 0U;
}

size_t ringBuffer_write(ringBuffer_t *rb, const uint8_t *src, size_t length)
{
    size_t writable;
    size_t written;
    size_t dropped;

    if (!ringBuffer_isValid(rb) || ((src == NULL) && (length > 0U))) {
        return 0U;
    }

    if (length == 0U) {
        return 0U;
    }

    writable = rb->capacity - rb->used;
    written = (length < writable) ? length : writable;

    if (written > 0U) {
        (void)ringBuffer_copyIn(rb, src, written);
        rb->used += written;
    }

    dropped = length - written;
    if (dropped > 0U) {
        const uint32_t remaining = UINT32_MAX - rb->dropped_bytes;

        if (dropped > remaining) {
            rb->dropped_bytes = UINT32_MAX;
        } else {
            rb->dropped_bytes += (uint32_t)dropped;
        }
    }

    return written;
}

size_t ringBuffer_read(ringBuffer_t *rb, uint8_t *dst, size_t capacity)
{
    size_t readable;

    if (!ringBuffer_isValid(rb) || ((dst == NULL) && (capacity > 0U))) {
        return 0U;
    }

    if (capacity == 0U) {
        return 0U;
    }

    readable = (capacity < rb->used) ? capacity : rb->used;
    return ringBuffer_copyOut(rb, dst, readable);
}

size_t ringBuffer_size(const ringBuffer_t *rb)
{
    return ringBuffer_isValid(rb) ? rb->used : 0U;
}

size_t ringBuffer_free(const ringBuffer_t *rb)
{
    return ringBuffer_isValid(rb) ? (rb->capacity - rb->used) : 0U;
}

uint32_t ringBuffer_dropped(const ringBuffer_t *rb)
{
    return ringBuffer_isValid(rb) ? rb->dropped_bytes : 0U;
}
