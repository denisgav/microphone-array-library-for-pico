#include "pico/sk9822.h"

#include "sk9822.pio.h"

STATIC sk9822_obj* sk9822_inst[MAX_SK9822_RP2] = {NULL, NULL};


sk9822_obj* init_sk9822(sk9822_config* config) {
	uint8_t led_strip_id = config -> led_strip_id;

	if(led_strip_id >= MAX_SK9822_RP2)
        return NULL;

    sk9822_obj* self;

    if(sk9822_inst[led_strip_id] == NULL){
        self = m_new_obj(sk9822_obj);
        sk9822_inst[led_strip_id] = self;
    } else {
        self = sk9822_inst[led_strip_id];
        deinit_sk9822(self);
    }

    memset(self, 0x00, sizeof(self));
    self->config = config;
    self->led_strip_id = led_strip_id;

    self->pixels = m_new(sk9822_pixel, self->config->num_of_pixels);
    self->dma_buffer = m_new(uint32_t, self->config->num_of_pixels+2);
    memset(self->pixels, 0x00, sizeof(uint32_t) * self->config->num_of_pixels);
    memset(self->dma_buffer, 0x00, sizeof(uint32_t) * self->config->num_of_pixels+2);

    self->pio_sm_offset = pio_add_program(self->config->pio, &sk9822_mini_program);
    self->pio_sm = pio_claim_unused_sm(self->config->pio, true);

    sk9822_mini_program_init(
    	self->config->pio, 
    	self->pio_sm, 
    	self->pio_sm_offset, 
    	SK9822_SERIAL_FREQ, 
    	self->config->pin_clk, 
    	self->config->pin_din);

    // Get a free channel, panic() if there are none
    self->dma_chan = dma_claim_unused_channel(true);

    // 32 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.

    dma_channel_config dma_config = dma_channel_get_default_config(self->dma_chan);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);

    channel_config_set_dreq(&dma_config, pio_get_dreq(self->config->pio, self->pio_sm, true));
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    dma_channel_configure(self->dma_chan,
        &dma_config,
        (void *)&self->config->pio->txf[self->pio_sm],                      // dest = PIO TX FIFO
        self->dma_buffer,                                             // src = DMA buffer
        self->config->num_of_pixels+2,
        false);

    return self;
}

void deinit_sk9822(sk9822_obj* self) {
	pio_sm_set_enabled(
        self->config->pio,
        self->pio_sm,
        false
    );

	free(self->pixels);
	sk9822_inst[self->led_strip_id] == NULL;
}

int set_pixel_sk9822(sk9822_obj* self, uint16_t pixel_idx, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness){
	if(pixel_idx < self->config->num_of_pixels){
        self->pixels[pixel_idx].r = r;
        self->pixels[pixel_idx].g = g;
        self->pixels[pixel_idx].b = b;
        self->pixels[pixel_idx].brightness = brightness;
		return 0;
	};
	return -1;
}

// int set_pixel_sk9822_audio(sk9822_obj* self, uint16_t pixel_idx, uint32_t audio_sample, uint8_t brightness){
// 	uint8_t r = 0; 
// 	uint8_t g = 0; 
// 	uint8_t b = 0;
// }

int set_all_pixels_sk9822(sk9822_obj* self, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness){
	for(uint16_t pixel_idx = 0; pixel_idx < self->config->num_of_pixels; pixel_idx++){
		int res = set_pixel_sk9822(self, pixel_idx, r, g, b, brightness);
		if(res != 0)
			return res;
	}
	return 0;
}

sk9822_pixel* get_pixel_sk9822(sk9822_obj* self, uint16_t pixel_idx){
	if(pixel_idx < MAX_SK9822_RP2){
		return &(self->pixels[pixel_idx]);
	}
	return NULL;
}

uint32_t get_sk9822_raw_data(sk9822_pixel pixel){
	uint32_t res = 0x7 << 29 |                   // magic
                (uint32_t) (pixel.brightness & 0x1f) << 24 |   // brightness parameter
                (uint32_t) pixel.b << 16 |
                (uint32_t) pixel.g << 8 |
                (uint32_t) pixel.r << 0;

    return res;
}

int show_blocking_sk9822(sk9822_obj* self){
	// Start frame
	pio_sm_put_blocking(self->config->pio, self->pio_sm, SK9822_START_FRAME);

	for(uint16_t pixel_idx = 0; pixel_idx < self->config->num_of_pixels; pixel_idx++){
		uint32_t pixel_raw_data = get_sk9822_raw_data(self->pixels[pixel_idx]);
		// Send pixel raw data
		pio_sm_put_blocking(self->config->pio, self->pio_sm, pixel_raw_data);
	}

	// End frame
	pio_sm_put_blocking(self->config->pio, self->pio_sm, SK9822_END_FRAME);
	return 0;
}

int show_non_blocking_sk9822(sk9822_obj* self){
	// TODO: improve smart solution with DMA here
	//return show_blocking_sk9822(self);

	// We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    dma_channel_wait_for_finish_blocking(self->dma_chan);

	// Start frame
	self->dma_buffer[0] = SK9822_START_FRAME;

	for(uint16_t pixel_idx = 0; pixel_idx < self->config->num_of_pixels; pixel_idx++){
		uint32_t pixel_raw_data = get_sk9822_raw_data(self->pixels[pixel_idx]);
		// Send pixel raw data
		self->dma_buffer[pixel_idx+1] = pixel_raw_data;
	}

	// End frame
	self->dma_buffer[self->config->num_of_pixels+1] = SK9822_END_FRAME;

	//dma_channel_set_read_addr (self->dma_chan, self->dma_buffer, true);

	// Everything is ready to go. Tell the control channel to load the first
    // control block. Everything is automatic from here.
    //dma_start_channel_mask(1u << (self->dma_chan));
    //dma_channel_start(self->dma_chan);
    dma_channel_transfer_from_buffer_now(self->dma_chan, self->dma_buffer, self->config->num_of_pixels+2);
}