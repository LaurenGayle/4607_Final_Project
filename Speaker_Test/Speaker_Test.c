/*
 * Speaker test for the Pico 2 W + two MAX98357A amps
 *
 * This test plays a repeating sequence over I2S (no Bluetooth involved):
 *   1. 440 Hz tone on the LEFT channel only   (only Amp L / Spk L should sound)
 *   2. 660 Hz tone on the RIGHT channel only  (only Amp R / Spk R should sound)
 *   3. 550 Hz tone on BOTH channels
 * with short silences in between. Each step is also printed over USB serial.
 *
 * Wiring (matches the project wiring diagram):
 *   BCLK -> GP26, LRC -> GP27, DIN -> GP28 (shared by both amps)
 *   Amp L: SD tied to Vin.  Amp R: SD to Vin through the right-channel resistor.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/audio_i2s.h"

#define SAMPLE_RATE        44100
#define SAMPLES_PER_BUFFER 256        /* stereo frames per buffer */
#define AMPLITUDE          8000       /* out of 32767; ~25% to protect the speakers */
#define SINE_TABLE_SIZE    1024

typedef struct {
    const char *name;
    float freq_hz;      /* 0 = silence */
    bool left;
    bool right;
    uint32_t duration_ms;
} test_step_t;

static const test_step_t steps[] = {
    { "LEFT only, 440 Hz",  440.0f, true,  false, 2000 },
    { "silence",              0.0f, false, false,  700 },
    { "RIGHT only, 660 Hz", 660.0f, false, true,  2000 },
    { "silence",              0.0f, false, false,  700 },
    { "BOTH, 550 Hz",       550.0f, true,  true,  2000 },
    { "silence",              0.0f, false, false, 1500 },
};
#define NUM_STEPS (sizeof steps / sizeof steps[0])

static int16_t sine_table[SINE_TABLE_SIZE];

static void build_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = (int16_t)(AMPLITUDE * sinf(2.0f * (float)M_PI * i / SINE_TABLE_SIZE));
    }
}

static struct audio_buffer_pool *audio_init(void) {
    static audio_format_t audio_format = {
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .sample_freq = SAMPLE_RATE,
        .channel_count = 2,                 /* stereo */
    };
    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4,                 /* 2 channels x 16 bits */
    };

    struct audio_buffer_pool *pool =
        audio_new_producer_pool(&producer_format, 3, SAMPLES_PER_BUFFER);

    struct audio_i2s_config config = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,          /* GP28 */
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE, /* GP26 = BCLK, GP27 = LRCLK */
        .dma_channel = 0,
        .pio_sm = 0,
    };

    if (!audio_i2s_setup(&audio_format, &config)) {
        panic("Unable to set up I2S audio\n");
    }
    if (!audio_i2s_connect(pool)) {
        panic("Unable to connect audio buffer pool\n");
    }
    audio_i2s_set_enabled(true);
    return pool;
}

static void play_step(struct audio_buffer_pool *pool, const test_step_t *step) {
    /* Phase accumulator: top 10 bits index the 1024-entry table */
    uint32_t phase = 0;
    uint32_t phase_step = (uint32_t)((step->freq_hz / SAMPLE_RATE) * 4294967296.0);
    uint32_t frames_left = (uint32_t)((uint64_t)step->duration_ms * SAMPLE_RATE / 1000);

    while (frames_left > 0) {
        struct audio_buffer *buffer = take_audio_buffer(pool, true);
        int16_t *samples = (int16_t *)buffer->buffer->bytes;

        uint32_t frames = buffer->max_sample_count;
        if (frames > frames_left) frames = frames_left;

        for (uint32_t i = 0; i < frames; i++) {
            int16_t value = 0;
            if (step->freq_hz > 0.0f) {
                value = sine_table[phase >> 22];
                phase += phase_step;
            }
            samples[2 * i]     = step->left  ? value : 0;  /* left  */
            samples[2 * i + 1] = step->right ? value : 0;  /* right */
        }

        buffer->sample_count = frames;
        give_audio_buffer(pool, buffer);
        frames_left -= frames;
    }
}

int main(void) {
    stdio_init_all();
    build_sine_table();
    struct audio_buffer_pool *pool = audio_init();

    printf("Speaker test starting\n");

    while (true) {
        for (unsigned s = 0; s < NUM_STEPS; s++) {
            printf("%s\n", steps[s].name);
            play_step(pool, &steps[s]);
        }
    }
}