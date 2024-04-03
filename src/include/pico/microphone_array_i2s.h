#pragma once


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "microphone_array_i2s.pio.h"

#define MAX_I2S_RP2 (2)

// The DMA buffer size was empirically determined.  It is a tradeoff between:
// 1. memory use (smaller buffer size desirable to reduce memory footprint)
// 2. interrupt frequency (larger buffer size desirable to reduce interrupt frequency)
#define SIZEOF_DMA_BUFFER_IN_BYTES (256)
#define SIZEOF_HALF_DMA_BUFFER_IN_BYTES (SIZEOF_DMA_BUFFER_IN_BYTES / 2)
#define I2S_NUM_DMA_CHANNELS (2)

#define NUM_I2S_USER_FORMATS (4)
#define I2S_RX_FRAME_SIZE_IN_BYTES (8)

#define SAMPLES_PER_FRAME (2)
#define PIO_INSTRUCTIONS_PER_BIT (2)

#ifndef STATIC
    #define STATIC static
#endif //STATIC

#ifndef m_new
    #define m_new(type, num) ((type *)(malloc(sizeof(type) * (num))))
#endif //m_new

#ifndef m_new_obj
    #define m_new_obj(type) (m_new(type, 1))
#endif //m_new_obj

#define mp_hal_pin_obj_t uint

typedef enum {
    GP_INPUT = 0,
    GP_OUTPUT = 1
} gpio_dir_t;

typedef enum {
    BLOCKING,
    NON_BLOCKING,
    UASYNCIO
} io_mode_t;

typedef struct  {
    uint32_t sample_l[4];
    uint32_t sample_r[4];
} i2s_audio_sample;

typedef struct _ring_buf_t {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} ring_buf_t;

typedef struct _microphone_array_i2s_obj_t {
    uint8_t i2s_id;
    mp_hal_pin_obj_t sck_base;
    mp_hal_pin_obj_t sd_base;
    int8_t bits;
    int32_t rate;
    int32_t ibuf;
    io_mode_t io_mode;
    PIO pio;
    uint8_t sm;
    const pio_program_t *pio_program;
    uint prog_offset;
    int dma_channel[I2S_NUM_DMA_CHANNELS];
    uint8_t dma_buffer[SIZEOF_DMA_BUFFER_IN_BYTES];
    ring_buf_t ring_buffer;
    uint8_t *ring_buffer_storage;
} microphone_array_i2s_obj_t;

// Buffer protocol
typedef struct _mp_buffer_info_t {
    void *buf;      // can be NULL if len == 0
    size_t len;     // in bytes
    int typecode;   // as per binary.h
} mp_buffer_info_t;


microphone_array_i2s_obj_t* create_microphone_array_i2s(uint8_t i2s_id,
              mp_hal_pin_obj_t sck_base, mp_hal_pin_obj_t sd_base,
              int8_t i2s_bits, int32_t ring_buffer_len, int32_t i2s_rate);


int microphone_array_i2s_read_stream(microphone_array_i2s_obj_t *self, void *buf_in, size_t size);

i2s_audio_sample decode_sample(i2s_audio_sample* sample_ptr_in);