#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hardware/pio.h"
#include "hardware/dma.h"

#ifndef MAX_SK9822_RP2
	#define MAX_SK9822_RP2 (2)
#endif //MAX_SK9822_RP2

#ifndef STATIC
	#define STATIC static
#endif //STATIC

#ifndef m_new
	#define m_new(type, num) ((type *)(malloc(sizeof(type) * (num))))
#endif //m_new

#ifndef m_new_obj
	#define m_new_obj(type) (m_new(type, 1))
#endif //m_new_obj

#ifndef SK9822_SERIAL_FREQ
	#define SK9822_SERIAL_FREQ (1 * 1000 * 1000) //(8 * 1000 * 1000)
#endif //SK9822_SERIAL_FREQ

#define SK9822_START_FRAME (0u)
#define SK9822_END_FRAME (~0u)

typedef enum {
	SK_9822_NONE,
	SK_9822_CREATED,
    SK_9822_INITIALIZED,
    SK_9822_BUSY,
    SK_9822_READY
} sk9822_status_t;

typedef struct __attribute__ ((__packed__)){
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t brightness;
} sk9822_pixel;

typedef struct  __sk9822_config{
    uint8_t led_strip_id;
    PIO pio;
    //uint dma_irq;
    uint8_t pin_clk;
    uint8_t pin_din;
    uint16_t num_of_pixels;
} sk9822_config;

typedef struct __sk9822_obj{
    uint8_t led_strip_id;
    sk9822_config* config;
    uint pio_sm;
    uint pio_sm_offset;
    int dma_channel;
    sk9822_pixel* pixels;
    uint32_t *dma_buffer;
    sk9822_status_t status;
    int dma_chan;


} sk9822_obj;

sk9822_obj* init_sk9822(sk9822_config* config);
void deinit_sk9822(sk9822_obj* self);

int set_pixel_sk9822(sk9822_obj* self, uint16_t pixel_idx, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
// int set_pixel_sk9822_audio(sk9822_obj* self, uint16_t pixel_idx, uint32_t audio_sample, uint8_t brightness);
int set_all_pixels_sk9822(sk9822_obj* self, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
sk9822_pixel* get_pixel_sk9822(sk9822_obj* self, uint16_t pixel_idx);

uint32_t get_sk9822_raw_data(sk9822_pixel pixel);
int show_blocking_sk9822(sk9822_obj* self);
int show_non_blocking_sk9822(sk9822_obj* self);