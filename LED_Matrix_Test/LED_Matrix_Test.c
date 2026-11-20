#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// ---- Configuration ---------------------------------------------------------
#define LED_PIN     16      // GPIO driving the matrix DIN
#define NUM_LEDS    256      // 8x8 matrix. Change if yours differs (e.g. 256 for 16x16)
#define BRIGHTNESS  16      // 0-255. Keep low: 64 LEDs at full white can draw ~3.8 A
#define IS_RGBW     false   // WS2812B is RGB (GRB order on the wire)

// ---- WS2812 PIO program (same as the pico-examples ws2812.pio, pre-assembled)
// .side_set 1 ; T1=2 T2=5 T3=3
// bitloop: out x,1 side 0 [2] ; jmp !x do_zero side 1 [1]
// do_one:  jmp bitloop side 1 [4]
// do_zero: nop side 0 [4]
static const uint16_t ws2812_instr[] = {
    0x6221, // out x, 1        side 0 [2]
    0x1123, // jmp !x, 3       side 1 [1]
    0x1400, // jmp 0           side 1 [4]
    0xa442, // nop             side 0 [4]
};
static const struct pio_program ws2812_program = {
    .instructions = ws2812_instr,
    .length = 4,
    .origin = -1,
};

static PIO  led_pio = pio0;
static uint led_sm;

static void ws2812_init(uint pin, float freq_hz) {
    uint offset = pio_add_program(led_pio, &ws2812_program);
    led_sm = pio_claim_unused_sm(led_pio, true);

    pio_gpio_init(led_pio, pin);
    pio_sm_set_consecutive_pindirs(led_pio, led_sm, pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, IS_RGBW ? 32 : 24); // MSB first, autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    const int cycles_per_bit = 2 + 5 + 3; // T1 + T2 + T3
    float div = clock_get_hz(clk_sys) / (freq_hz * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(led_pio, led_sm, offset, &c);
    pio_sm_set_enabled(led_pio, led_sm, true);
}

static inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    r = (r * BRIGHTNESS) / 255;
    g = (g * BRIGHTNESS) / 255;
    b = (b * BRIGHTNESS) / 255;
    // WS2812 expects GRB; shifted left 8 because the PIO shifts out the top 24 bits
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
}

static inline void put_pixel(uint32_t grb) {
    pio_sm_put_blocking(led_pio, led_sm, grb << 8u);
}

static void fill(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) put_pixel(color);
    sleep_us(300); // latch/reset
}

static void chase(uint32_t color, int delay_ms) {
    for (int pos = 0; pos < NUM_LEDS; pos++) {
        for (int i = 0; i < NUM_LEDS; i++) put_pixel(i == pos ? color : 0);
        sleep_us(300);
        sleep_ms(delay_ms);
    }
}

int main() {
    stdio_init_all();
    ws2812_init(LED_PIN, 800000);

    while (true) {
        printf("Red\n");   fill(rgb(255, 0, 0)); sleep_ms(1000);
        printf("Green\n"); fill(rgb(0, 255, 0)); sleep_ms(1000);
        printf("Blue\n");  fill(rgb(0, 0, 255)); sleep_ms(1000);
        printf("White\n"); fill(rgb(255, 255, 255)); sleep_ms(1000);
        printf("Chase\n"); chase(rgb(255, 255, 255), 40);
        fill(0); sleep_ms(500);
    }
}
