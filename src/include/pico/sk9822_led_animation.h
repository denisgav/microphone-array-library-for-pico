#pragma once 

#ifndef N_LEDS
#define N_LEDS 12
#endif //N_LEDS

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/sk9822.h"

typedef struct _led_animation_obj{

	uint32_t led_status_sample[N_LEDS];

	uint32_t sample_valid_cntr;
	uint32_t point;
	uint32_t distance_between_points;
	uint32_t point_size;

	uint8_t point_r;
	uint8_t point_g;
	uint8_t point_b;
	uint8_t brightness;

	uint32_t msSinceBoot;
  	uint32_t msLedAnimationTriggered;
	
} led_animation_obj;

void sk9822_led_animation(led_animation_obj* self, sk9822_obj* sk9822);
void sk9822_led_animation_round_ring(led_animation_obj* self, sk9822_obj* sk9822);
void sk9822_led_animation_mic_samples(led_animation_obj* self, sk9822_obj* sk9822);
void sk9822_set_status_data(led_animation_obj* self, uint32_t led_status_sample[]);
