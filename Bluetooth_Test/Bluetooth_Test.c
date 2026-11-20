/*
 * Bluetooth test for the Pico 2 W + two MAX98357A amps
 *
 * The Pico shows up as a Bluetooth speaker ("Pico Speaker XX:XX:...").
 * Pair a laptop/phone with it, then play any audio (e.g. channel_test.wav).
 * The stream is received over Classic Bluetooth A2DP (SBC codec), decoded
 * to 16-bit stereo PCM, and sent out over I2S:
 *   LEFT  channel -> Amp L / Spk L
 *   RIGHT channel -> Amp R / Spk R
 *
 * Wiring (matches the project wiring diagram / Speaker_Test):
 *   BCLK -> GP26, LRC -> GP27, DIN -> GP28 (shared by both amps)
 *
 * Status:
 *   - Onboard LED: off = waiting for a device, on = device connected
 *   - USB serial: connection events, codec settings, and buffer stats
 *
 * Data flow:
 *   BTstack media callback -> SBC decoder -> PCM ring buffer -> I2S buffer pool
 * Everything runs from the main loop (pico_cyw43_arch_poll), so the ring
 * buffer needs no locking.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/audio_i2s.h"
#include "btstack.h"
#include "classic/btstack_sbc_bluedroid.h"

/* ---------- Settings ---------- */
#define DEVICE_NAME        "Pico Speaker 00:00:00:00:00:00" /* zeros replaced with BT address */
#define SAMPLE_RATE        44100
#define SAMPLES_PER_BUFFER 256          /* stereo frames per I2S buffer */
#define NUM_I2S_BUFFERS    4
#define VOLUME_PERCENT     30           /* software gain; keep low to protect speakers */

#define I2S_DATA_PIN       28           /* DIN */
#define I2S_CLOCK_PIN_BASE 26           /* BCLK = GP26, LRC = GP27 */

/* PCM ring buffer: 16384 stereo frames = 64 KB = ~370 ms at 44.1 kHz */
#define RING_FRAMES        16384
#define PREBUFFER_FRAMES   (RING_FRAMES / 2)   /* level to reach before playback starts */
#define DRIFT_HIGH_FRAMES  (RING_FRAMES * 3 / 4)
#define DRIFT_LOW_FRAMES   (RING_FRAMES / 4)

/* ---------- PCM ring buffer (one frame = L and R int16 packed in 32 bits) ---------- */
static uint32_t ring[RING_FRAMES];
static uint32_t ring_head;   /* write index */
static uint32_t ring_tail;   /* read index  */
static bool     playing;     /* true once prebuffer level reached */

static uint32_t stat_overflow_frames;
static uint32_t stat_underruns;
static uint32_t stat_frames_played;

static inline uint32_t ring_level(void) {
    return (ring_head - ring_tail) & (RING_FRAMES - 1);
}

static void ring_reset(void) {
    ring_head = ring_tail = 0;
    playing = false;
}

static inline void ring_push(int16_t left, int16_t right) {
    if (ring_level() == RING_FRAMES - 1) {   /* full: drop newest */
        stat_overflow_frames++;
        return;
    }
    ring[ring_head] = ((uint32_t)(uint16_t)right << 16) | (uint16_t)left;
    ring_head = (ring_head + 1) & (RING_FRAMES - 1);
}

/* ---------- I2S output ---------- */
static struct audio_buffer_pool *audio_init(void) {
    static audio_format_t audio_format = {
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .sample_freq = SAMPLE_RATE,
        .channel_count = 2,
    };
    static struct audio_buffer_format producer_format = {
        .format = &audio_format,
        .sample_stride = 4,                 /* 2 channels x 16 bits */
    };

    struct audio_buffer_pool *pool =
        audio_new_producer_pool(&producer_format, NUM_I2S_BUFFERS, SAMPLES_PER_BUFFER);

    struct audio_i2s_config config = {
        .data_pin = I2S_DATA_PIN,
        .clock_pin_base = I2S_CLOCK_PIN_BASE,
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

/*
 * Move decoded PCM from the ring buffer into free I2S buffers.
 * If no buffer is handed over, the I2S driver plays silence on its own.
 *
 * The laptop's clock and the Pico's I2S clock never match exactly, so the
 * ring slowly fills or drains. When it's too full we skip one frame per
 * buffer; when it's too empty we repeat one frame. That is inaudible and
 * keeps latency roughly constant.
 */
static void pump_audio(struct audio_buffer_pool *pool) {
    if (!playing) {
        if (ring_level() < PREBUFFER_FRAMES) return;
        playing = true;
        printf("Playback started (buffer %lu frames)\n", (unsigned long)ring_level());
    }

    while (ring_level() >= SAMPLES_PER_BUFFER) {
        struct audio_buffer *buffer = take_audio_buffer(pool, false);
        if (!buffer) return;    /* all I2S buffers queued; try again next loop */

        int16_t *samples = (int16_t *)buffer->buffer->bytes;
        uint32_t level = ring_level();
        uint32_t frames = buffer->max_sample_count;

        if (level > DRIFT_HIGH_FRAMES) {
            ring_tail = (ring_tail + 1) & (RING_FRAMES - 1);     /* skip one frame */
        }
        bool repeat_one = level < DRIFT_LOW_FRAMES;

        for (uint32_t i = 0; i < frames; i++) {
            uint32_t frame = ring[ring_tail];
            if (!(repeat_one && i == 0)) {
                ring_tail = (ring_tail + 1) & (RING_FRAMES - 1);
            }
            int32_t left  = (int16_t)(frame & 0xFFFF);
            int32_t right = (int16_t)(frame >> 16);
            samples[2 * i]     = (int16_t)(left  * VOLUME_PERCENT / 100);
            samples[2 * i + 1] = (int16_t)(right * VOLUME_PERCENT / 100);
        }

        buffer->sample_count = frames;
        give_audio_buffer(pool, buffer);
        stat_frames_played += frames;
    }

    /* Ran dry while playing: wait to refill before resuming (avoids stutter) */
    if (ring_level() < SAMPLES_PER_BUFFER) {
        stat_underruns++;
        playing = false;
    }
}

/* ---------- SBC decoding ---------- */
static const btstack_sbc_decoder_t   *sbc_decoder;
static btstack_sbc_decoder_bluedroid_t sbc_decoder_context;

static void handle_pcm_data(int16_t *data, int num_frames, int num_channels,
                            int sample_rate, void *context) {
    (void)sample_rate;
    (void)context;
    for (int i = 0; i < num_frames; i++) {
        if (num_channels == 2) {
            ring_push(data[2 * i], data[2 * i + 1]);
        } else {
            ring_push(data[i], data[i]);    /* mono: same signal on both speakers */
        }
    }
}

static void media_init(void) {
    sbc_decoder = btstack_sbc_decoder_bluedroid_init_instance(&sbc_decoder_context);
    sbc_decoder->configure(&sbc_decoder_context, SBC_MODE_STANDARD, handle_pcm_data, NULL);
    ring_reset();
}

/*
 * Called by BTstack for every received media packet:
 *   12-byte RTP header | 1-byte SBC header | one or more SBC frames
 */
static void handle_media_packet(uint8_t seid, uint8_t *packet, uint16_t size) {
    (void)seid;
    const int rtp_header_len = 12;
    const int sbc_header_len = 1;
    if (!sbc_decoder || size <= rtp_header_len + sbc_header_len) return;

    int csrc_count = packet[0] & 0x0F;            /* optional extra RTP words */
    int offset = rtp_header_len + 4 * csrc_count + sbc_header_len;
    if (offset >= size) return;

    sbc_decoder->decode_signed_16(&sbc_decoder_context, 0, packet + offset, size - offset);
}

/* ---------- A2DP sink ---------- */
/*
 * SBC capabilities we advertise. Only 44.1 kHz is offered so the I2S clock
 * never needs to change (every A2DP source must support 44.1 kHz).
 *   byte 0: sampling freq (0x20 = 44.1k) | channel modes (0x0F = all)
 *   byte 1: block lengths | subbands | allocation methods (0xFF = all)
 *   bytes 2-3: min / max bitpool
 */
static uint8_t sbc_capabilities[] = { 0x2F, 0xFF, 2, 53 };
static uint8_t sbc_configuration[4];
static uint8_t sdp_a2dp_sink_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
            uint16_t rate = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            printf("SBC config: %u Hz, %u channel(s), bitpool %u-%u\n", rate,
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet),
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
            if (rate != SAMPLE_RATE) {
                printf("WARNING: expected %d Hz; audio will play at the wrong speed\n", SAMPLE_RATE);
            }
            break;
        }
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("Source chose a non-SBC codec (not supported)\n");
            break;

        case A2DP_SUBEVENT_STREAM_ESTABLISHED: {
            uint8_t status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                printf("A2DP stream connection failed, status 0x%02x\n", status);
                break;
            }
            bd_addr_t addr;
            a2dp_subevent_stream_established_get_bd_addr(packet, addr);
            printf("Connected to %s\n", bd_addr_to_str(addr));
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            break;
        }
        case A2DP_SUBEVENT_STREAM_STARTED:
            printf("Stream started - receiving audio\n");
            media_init();
            break;

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            printf("Stream paused\n");
            ring_reset();
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            printf("Stream released\n");
            sbc_decoder = NULL;
            ring_reset();
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            printf("Disconnected - waiting for a device\n");
            sbc_decoder = NULL;
            ring_reset();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            break;

        default:
            break;
    }
}

/* Pairing: accept everything automatically ("Just Works") */
static void hci_packet_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t addr;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_local_bd_addr(addr);
                printf("Bluetooth ready. Pair with \"Pico Speaker %s\"\n", bd_addr_to_str(addr));
            }
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:   /* legacy pairing: use PIN 0000 */
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            gap_pin_code_response(addr, "0000");
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            hci_event_user_confirmation_request_get_bd_addr(packet, addr);
            gap_ssp_confirmation_response(addr);
            break;
        default:
            break;
    }
}

static void bluetooth_init(void) {
    l2cap_init();
    sdp_init();
    a2dp_sink_init();

    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&handle_media_packet);
    avdtp_stream_endpoint_t *endpoint = a2dp_sink_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC,
        sbc_capabilities, sizeof(sbc_capabilities),
        sbc_configuration, sizeof(sbc_configuration));
    if (!endpoint) panic("Could not create A2DP stream endpoint\n");

    /* SDP record so the laptop sees us as an audio sink (speaker) */
    memset(sdp_a2dp_sink_buffer, 0, sizeof(sdp_a2dp_sink_buffer));
    a2dp_sink_create_sdp_record(sdp_a2dp_sink_buffer, sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_SPEAKER, NULL, NULL);
    sdp_register_service(sdp_a2dp_sink_buffer);

    gap_set_local_name(DEVICE_NAME);
    gap_set_class_of_device(0x240414);   /* Audio/Video: Loudspeaker */
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH |
                                         LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);
    gap_discoverable_control(1);
    gap_connectable_control(1);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
}

int main(void) {
    stdio_init_all();

    /* Set up I2S first: it claims DMA channel 0 and PIO0 SM0 by number,
     * and the CYW43 driver then takes whatever is still free. */
    struct audio_buffer_pool *pool = audio_init();

    if (cyw43_arch_init()) {
        printf("CYW43 (wireless chip) init failed\n");
        return 1;
    }

    printf("Bluetooth test starting\n");
    bluetooth_init();

    absolute_time_t next_stats = make_timeout_time_ms(5000);
    while (true) {
        cyw43_arch_poll();          /* runs BTstack: receive + decode */
        pump_audio(pool);           /* feed decoded audio to the speakers */

        if (time_reached(next_stats)) {
            next_stats = make_timeout_time_ms(5000);
            if (sbc_decoder) {
                printf("buffer %5lu/%d frames | played %lu | underruns %lu | dropped %lu\n",
                       (unsigned long)ring_level(), RING_FRAMES,
                       (unsigned long)stat_frames_played, (unsigned long)stat_underruns,
                       (unsigned long)stat_overflow_frames);
            }
        }
    }
}
