#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "pico/sk9822.h"

#include "main.h"

sk9822_config config = {
    .led_strip_id = 0,
    .pio = pio1,
    //.dma_irq,
    .pin_clk = LED_PIN_CLK,
    .pin_din = LED_PIN_DIN,
    .num_of_pixels = N_LEDS
};

int main() {
    stdio_init_all();

    sk9822_obj* sk9822 = init_sk9822(&config);

    if(sk9822 == NULL){
        while(1){
            sleep_ms(200);
        }
    }

    // LED strip initialized
    sleep_ms(200);
    set_all_pixels_sk9822(sk9822, 0, 0, 0, BRIGHTNESS);
    show_blocking_sk9822(sk9822);

    uint32_t point = 0;
    uint32_t distance_between_points = 32;
    uint32_t point_size = 4*distance_between_points;

    uint8_t point_r = 255;
    uint8_t point_g = 0;
    uint8_t point_b = 0;

    while (true) {
        //set_all_pixels_sk9822(sk9822, 0, 0, 0, BRIGHTNESS);

        for(uint16_t led_idx = 0; led_idx < N_LEDS; led_idx++){
            uint32_t led_distance_cw = (point >= (led_idx*distance_between_points)) ? 
                (point - led_idx*distance_between_points) : 
                ((N_LEDS - led_idx)*distance_between_points + point);

            uint32_t led_distance_ccw = ((led_idx*distance_between_points) >= point) ? 
                (led_idx*distance_between_points - point) : 
                ((N_LEDS + led_idx)*distance_between_points - point);

            uint32_t led_distance = led_distance_cw <= led_distance_ccw ? led_distance_cw : led_distance_ccw;

            if(led_distance <= point_size)
            {
                set_pixel_sk9822(sk9822, led_idx, 
                    ((uint32_t)point_r * (point_size-led_distance)) / point_size, 
                    ((uint32_t)point_g * (point_size-led_distance)) / point_size, 
                    ((uint32_t)point_b * (point_size-led_distance)) / point_size, 
                    BRIGHTNESS);
            }
            else
            {
                set_pixel_sk9822(sk9822, led_idx, 0, 0, 0, BRIGHTNESS);
            }
        }
        //show_blocking_sk9822(sk9822);
        show_non_blocking_sk9822(sk9822);
        point++;
        if(point >= N_LEDS*distance_between_points)
            point = 0;
        sleep_ms(1000/25); // 25 FPS
    }
}