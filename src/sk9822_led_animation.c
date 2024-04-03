#include "pico/time.h"

#include "pico/sk9822_led_animation.h"


void sk9822_led_animation(led_animation_obj* self, sk9822_obj* sk9822){
	self->msSinceBoot = to_ms_since_boot(get_absolute_time());
	if ((self->msSinceBoot - self->msLedAnimationTriggered) > 100/25){// 25 FPS
		self->msLedAnimationTriggered = self->msSinceBoot;
		if (self->sample_valid_cntr > 0){ // We have a sample from USB
			sk9822_led_animation_mic_samples(self, sk9822);
			self->sample_valid_cntr--;
		}
		else {
			sk9822_led_animation_round_ring(self, sk9822);
		}
	}
}

void sk9822_led_animation_round_ring(led_animation_obj* self, sk9822_obj* sk9822){
	for(uint16_t led_idx = 0; led_idx < N_LEDS; led_idx++)
	{
		uint32_t led_distance_cw = (self->point >= (led_idx*self->distance_between_points)) ? 
			(self->point - led_idx*self->distance_between_points) : 
			((N_LEDS - led_idx)*self->distance_between_points + self->point);

		uint32_t led_distance_ccw = ((led_idx*self->distance_between_points) >= self->point) ? 
				(led_idx*self->distance_between_points - self->point) : 
				((N_LEDS + led_idx)*self->distance_between_points - self->point);

		uint32_t led_distance = led_distance_cw <= led_distance_ccw ? led_distance_cw : led_distance_ccw;

		if(led_distance <= self->point_size){
				set_pixel_sk9822(sk9822, led_idx, 
						((uint32_t)self->point_r * (self->point_size-led_distance)) / self->point_size, 
						((uint32_t)self->point_g * (self->point_size-led_distance)) / self->point_size, 
						((uint32_t)self->point_b * (self->point_size-led_distance)) / self->point_size, 
						(uint32_t)self->brightness);

		}
		else{
				set_pixel_sk9822(sk9822, led_idx, 0, 0, 0, self->brightness);
		}
	}
	self->point++;
	if(self->point >= N_LEDS*self->distance_between_points)
		self->point = 0;

	if(self->point == 0)
	{
		while(true)
		{
			if((self->point_r > 0) && (self->point_b == 0))
			{
				uint8_t dec_value = (self->point_r >= 1) ? 1 : self->point_r;
				self->point_r -= dec_value;
				self->point_g += dec_value;
				break;
			}
			if(self->point_g > 0)
			{
				uint8_t dec_value = (self->point_g >= 1) ? 1 : self->point_g;
				self->point_g -= dec_value;
				self->point_b += dec_value;
				break;
			}
			if(self->point_b > 0)
			{
				uint8_t dec_value = (self->point_b >= 1) ? 1 : self->point_b;
				self->point_b -= dec_value;
				self->point_r += dec_value;
				break;
			}
		}
	}

	//show_blocking_sk9822(sk9822);
	show_non_blocking_sk9822(sk9822);
}

void sk9822_led_animation_mic_samples(led_animation_obj* self, sk9822_obj* sk9822){
	set_all_pixels_sk9822(sk9822, 0, 0, 0, self->brightness);
	for(int led_idx=0; led_idx<N_LEDS; led_idx++){
		//sk9822_set_pixel_audio_val(self, sk9822, led_idx, self->led_status_sample[led_idx]);
		set_pixel_sk9822(sk9822, led_idx, 8, 8, 8, (uint32_t)self->brightness);
	}
	//show_blocking_sk9822(sk9822);
	show_non_blocking_sk9822(sk9822);
}

void sk9822_set_status_data(led_animation_obj* self, uint32_t led_status_sample[]){
	for(int led_idx=0; led_idx<N_LEDS; led_idx++){
		self->led_status_sample[led_idx] = led_status_sample[led_idx];
	}
	self->sample_valid_cntr = 100;
}