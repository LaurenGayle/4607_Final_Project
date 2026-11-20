/*
 * Demo script for the sound-reactive speaker (Pico 2 W)
 *
 * Combines the individual tests into one program:
 *   - Bluetooth: the Pico shows up as a speaker ("Pico Speaker XX:XX:...").
 *     Audio arrives over A2DP (SBC), is decoded to 16-bit stereo PCM and sent
 *     over I2S to the two MAX98357A amps / DACs (L -> Spk L, R -> Spk R).
 *     AVRCP is used to read the title of the song that is playing.
 *   - LCD (16x2, I2C):
 *       line 1: "Mode N"           (Mode button cycles 1 -> 2 -> 3 -> 1)
 *       line 2: song title          (scrolls if longer than 16 characters)
 *               "Volume: N/10"     for a few seconds after Vol+ / Vol- is pressed
 *   - LED matrix (WS2812): steps through a list of colors.
 *
 * Wiring:
 *   I2S:     BCLK -> GP26, LRC -> GP27, DIN -> GP28 (shared by both amps)
 *   LCD:     SDA -> GP4, SCL -> GP5
 *   Buttons: Vol+ -> GP13, Mode -> GP14, Vol- -> GP15 (to GND, internal pull-ups)
 *   LEDs:    DIN -> GP16
 *
 * Everything runs from one main loop (pico_cyw43_arch_poll), so no locking
 * is needed. The loop never sleeps: Bluetooth and audio need constant polling.
 * The LED matrix is fed by DMA and the LCD is only written when its text
 * changes, so neither stalls the audio.
 *
 * Data flow:
 *   BTstack media callback -> SBC decoder -> PCM ring buffer -> I2S buffer pool
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/audio_i2s.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "classic/btstack_sbc_bluedroid.h"

/* ====================================================================== */
/* Configuration                                                          */
/* ====================================================================== */

// LCD screen
#define LCD_I2C          i2c0
#define LCD_SDA_PIN      4
#define LCD_SCL_PIN      5
#define LCD_ADDR         0x27      /* 0x3F is also tried automatically */
#define LCD_REFRESH_MS   50        /* how often the display text is re-checked */
#define LCD_RETRY_MS     1000      /* wait after an I2C error before trying again */

// Buttons
#define BTN_VOL_UP_PIN   13
#define BTN_VOL_DOWN_PIN 15
#define BTN_MODE_PIN     14
#define DEBOUNCE_US      20000     /* 20 ms */

#define NUM_MODES        3
#define VOLUME_MIN       0
#define VOLUME_MAX       10
#define VOLUME_START     5
#define VOLUME_SHOW_MS   3000      /* how long line 2 shows the volume */
#define MAX_GAIN_PERCENT 60        /* software gain at volume 10; volume 5 = 30 % (tested level) */

// Song title on the LCD
#define TITLE_MAX        96        /* longest title kept (characters) */
#define SCROLL_STEP_MS   350       /* scroll speed for long titles */
#define SCROLL_HOLD      4         /* steps to pause at the start of the title */
#define SCROLL_GAP       "   "     /* spacing between the end and the start when scrolling */

// LED matrix
#define LED_PIN          16        /* GPIO driving the matrix DIN */
#define NUM_LEDS         256       /* 16x16 matrix */
#define BRIGHTNESS       16        /* 0-255. Keep low: 256 LEDs at full white draw a lot of current */
#define LED_COLOR_MS     1000      /* time each color is shown */

// Bluetooth / audio
#define DEVICE_NAME        "Pico Speaker 00:00:00:00:00:00" /* zeros replaced with BT address */
#define SAMPLE_RATE        44100
#define SAMPLES_PER_BUFFER 256          /* stereo frames per I2S buffer */
#define NUM_I2S_BUFFERS    4
#define AVRCP_CONNECT_DELAY_MS 2000     /* connect AVRCP ourselves if the source hasn't */

#define I2S_DATA_PIN       28           /* DIN */
#define I2S_CLOCK_PIN_BASE 26           /* BCLK = GP26, LRC = GP27 */

// PCM ring buffer: 16384 stereo frames = 64 KB = ~370 ms at 44.1 kHz
#define RING_FRAMES        16384
#define PREBUFFER_FRAMES   (RING_FRAMES / 2)   /* level to reach before playback starts */
#define DRIFT_HIGH_FRAMES  (RING_FRAMES * 3 / 4)
#define DRIFT_LOW_FRAMES   (RING_FRAMES / 4)

/* ====================================================================== */
/* Shared state                                                           */
/* ====================================================================== */

static int  mode   = 1;                 /* 1..NUM_MODES */
static int  volume = VOLUME_START;      /* VOLUME_MIN..VOLUME_MAX */

static bool bt_connected;               /* A2DP connection to a phone/laptop */
static char song_title[TITLE_MAX + 1];  /* LCD-safe ASCII, empty if unknown */
static absolute_time_t title_changed_at;

/* ====================================================================== */
/* PCM ring buffer (one frame = L and R int16 packed in 32 bits)          */
/* ====================================================================== */

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

/* ====================================================================== */
/* I2S output                                                             */
/* ====================================================================== */

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
 * Move decoded PCM from the ring buffer into free I2S buffers, applying the
 * volume set with the buttons. If no buffer is handed over, the I2S driver
 * plays silence on its own.
 *
 * The source's clock and the Pico's I2S clock never match exactly, so the
 * ring slowly fills or drains. When it's too full we skip one frame per
 * buffer; when it's too empty we repeat one frame. That is inaudible and
 * keeps latency roughly constant.
 */
static void pump_audio(struct audio_buffer_pool *pool) {
    if (!playing) {
        if (ring_level() < PREBUFFER_FRAMES) return;
        playing = true;
        printf("[AUDIO] Playback started (buffer %lu frames)\n", (unsigned long)ring_level());
    }

    const int32_t gain = volume * MAX_GAIN_PERCENT / VOLUME_MAX;   /* percent */

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
            samples[2 * i]     = (int16_t)(left  * gain / 100);
            samples[2 * i + 1] = (int16_t)(right * gain / 100);
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

/* ====================================================================== */
/* SBC decoding                                                           */
/* ====================================================================== */

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

/* ====================================================================== */
/* Song title (AVRCP metadata)                                            */
/* ====================================================================== */

/*
 * Titles arrive as UTF-8. The LCD only has ASCII, so curly quotes are
 * replaced with straight ones and any other non-ASCII character with '?'.
 */
static void set_song_title(const uint8_t *utf8, int len) {
    int n = 0;
    for (int i = 0; i < len && n < TITLE_MAX; i++) {
        uint8_t c = utf8[i];
        if (c >= 0x20 && c < 0x7F) {
            song_title[n++] = (char)c;
        } else if (c == 0xE2 && i + 2 < len && utf8[i + 1] == 0x80 &&
                   utf8[i + 2] >= 0x98 && utf8[i + 2] <= 0x9D) {
            song_title[n++] = utf8[i + 2] <= 0x9B ? '\'' : '"';   /* ' ' or " " */
            i += 2;
        } else if (c >= 0xC0) {
            song_title[n++] = '?';      /* start of another multi-byte character */
        }
        /* 0x80-0xBF (continuation bytes) and control characters are skipped */
    }
    song_title[n] = '\0';
    title_changed_at = get_absolute_time();
    printf("[AVRCP] Now playing: %s\n", song_title);
}

static void clear_song_title(void) {
    song_title[0] = '\0';
    title_changed_at = get_absolute_time();
}

/* ====================================================================== */
/* Bluetooth: A2DP sink + AVRCP controller                                */
/* ====================================================================== */

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
static uint8_t sdp_avrcp_controller_buffer[200];
static uint8_t sdp_avrcp_target_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static bd_addr_t source_addr;            /* phone / laptop we are connected to */
static uint16_t  avrcp_cid;              /* 0 = no AVRCP connection */
static btstack_timer_source_t avrcp_connect_timer;

static void request_song_title(void) {
    if (avrcp_cid) avrcp_controller_get_now_playing_info(avrcp_cid);
}

/* Most sources open AVRCP on their own; if this one hasn't, open it ourselves. */
static void avrcp_connect_timeout(btstack_timer_source_t *ts) {
    (void)ts;
    if (bt_connected && avrcp_cid == 0) {
        printf("[AVRCP] Source did not connect AVRCP, connecting to it\n");
        uint16_t cid;
        avrcp_connect(source_addr, &cid);
    }
}

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
            uint16_t rate = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            printf("[A2DP] SBC config: %u Hz, %u channel(s), bitpool %u-%u\n", rate,
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet),
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet),
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet));
            if (rate != SAMPLE_RATE) {
                printf("[A2DP] WARNING: expected %d Hz; audio will play at the wrong speed\n", SAMPLE_RATE);
            }
            break;
        }
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("[A2DP] Source chose a non-SBC codec (not supported)\n");
            break;

        case A2DP_SUBEVENT_STREAM_ESTABLISHED: {
            uint8_t status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                printf("[A2DP] Stream connection failed, status 0x%02x\n", status);
                break;
            }
            a2dp_subevent_stream_established_get_bd_addr(packet, source_addr);
            printf("[A2DP] Connected to %s\n", bd_addr_to_str(source_addr));
            bt_connected = true;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

            btstack_run_loop_remove_timer(&avrcp_connect_timer);
            btstack_run_loop_set_timer_handler(&avrcp_connect_timer, avrcp_connect_timeout);
            btstack_run_loop_set_timer(&avrcp_connect_timer, AVRCP_CONNECT_DELAY_MS);
            btstack_run_loop_add_timer(&avrcp_connect_timer);
            break;
        }
        case A2DP_SUBEVENT_STREAM_STARTED:
            printf("[A2DP] Stream started - receiving audio\n");
            media_init();
            request_song_title();      /* in case the track-changed event was missed */
            break;

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            printf("[A2DP] Stream paused\n");
            ring_reset();
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            printf("[A2DP] Stream released\n");
            sbc_decoder = NULL;
            ring_reset();
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            printf("[A2DP] Disconnected - waiting for a device\n");
            sbc_decoder = NULL;
            ring_reset();
            bt_connected = false;
            clear_song_title();
            btstack_run_loop_remove_timer(&avrcp_connect_timer);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            break;

        default:
            break;
    }
}

/* AVRCP connection events (shared by the controller and target roles) */
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel,
                                 uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    switch (hci_event_avrcp_meta_get_subevent_code(packet)) {
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
            uint8_t status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                printf("[AVRCP] Connection failed, status 0x%02x\n", status);
                avrcp_cid = 0;
                break;
            }
            avrcp_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            printf("[AVRCP] Connected, cid 0x%02x\n", avrcp_cid);
            /* Ask which notifications the source supports; continues below */
            avrcp_controller_get_supported_events(avrcp_cid);
            break;
        }
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("[AVRCP] Disconnected\n");
            avrcp_cid = 0;
            clear_song_title();
            break;

        default:
            break;
    }
}

/* AVRCP controller: we ask the source for track changes and track info */
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel,
                                            uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    if (avrcp_cid == 0) return;

    switch (hci_event_avrcp_meta_get_subevent_code(packet)) {
        case AVRCP_SUBEVENT_GET_CAPABILITY_EVENT_ID_DONE:
            /* The source answers with the current track right away (interim
             * response) and again every time the track changes. */
            avrcp_controller_enable_notification(avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            break;

        case AVRCP_SUBEVENT_NOTIFICATION_TRACK_CHANGED:
            request_song_title();
            break;

        case AVRCP_SUBEVENT_NOW_PLAYING_TITLE_INFO:
            set_song_title(avrcp_subevent_now_playing_title_info_get_value(packet),
                           avrcp_subevent_now_playing_title_info_get_value_len(packet));
            break;

        default:
            break;
    }
}

/* AVRCP target: BTstack requires a handler for this role; we ignore its events */
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel,
                                        uint8_t *packet, uint16_t size) {
    (void)packet_type;
    (void)channel;
    (void)packet;
    (void)size;
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
                printf("[BT] Ready. Pair with \"Pico Speaker %s\"\n", bd_addr_to_str(addr));
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
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();

    /* A2DP sink: receives the audio */
    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&handle_media_packet);
    avdtp_stream_endpoint_t *endpoint = a2dp_sink_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC,
        sbc_capabilities, sizeof(sbc_capabilities),
        sbc_configuration, sizeof(sbc_configuration));
    if (!endpoint) panic("Could not create A2DP stream endpoint\n");

    /* AVRCP: reads the song title */
    avrcp_register_packet_handler(&avrcp_packet_handler);
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    /* SDP records so the source sees us as a speaker with media info support */
    memset(sdp_a2dp_sink_buffer, 0, sizeof(sdp_a2dp_sink_buffer));
    a2dp_sink_create_sdp_record(sdp_a2dp_sink_buffer, sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_SPEAKER, NULL, NULL);
    sdp_register_service(sdp_a2dp_sink_buffer);

    memset(sdp_avrcp_controller_buffer, 0, sizeof(sdp_avrcp_controller_buffer));
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_buffer, sdp_create_service_record_handle(),
                                       1 << AVRCP_CONTROLLER_SUPPORTED_FEATURE_CATEGORY_PLAYER_OR_RECORDER,
                                       NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_buffer);

    memset(sdp_avrcp_target_buffer, 0, sizeof(sdp_avrcp_target_buffer));
    avrcp_target_create_sdp_record(sdp_avrcp_target_buffer, sdp_create_service_record_handle(),
                                   1 << AVRCP_TARGET_SUPPORTED_FEATURE_CATEGORY_MONITOR_OR_AMPLIFIER,
                                   NULL, NULL);
    sdp_register_service(sdp_avrcp_target_buffer);

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

/* ====================================================================== */
/* LCD (HD44780 behind a PCF8574 I2C expander)                            */
/* ====================================================================== */
/*
 * PCF8574 bit layout on the common backpack:
 *   P0 = RS, P1 = RW, P2 = EN, P3 = backlight, P4..P7 = D4..D7
 * The LCD runs in 4-bit mode, so every byte is sent as two nibbles.
 *
 * A whole line is sent in ONE I2C transaction (~6 ms at 100 kHz). Each I2C
 * byte takes ~90 us, which is longer than the EN pulse and the LCD's
 * command time, so no extra delays are needed between characters.
 */
#define LCD_RS         0x01
#define LCD_EN         0x04
#define LCD_BACKLIGHT  0x08

#define LCD_CMD_CLEAR        0x01
#define LCD_CMD_ENTRY_MODE   0x06  /* cursor moves right, no display shift */
#define LCD_CMD_DISPLAY_ON   0x0C  /* display on, cursor off, blink off */
#define LCD_CMD_FUNCTION_SET 0x28  /* 4-bit bus, 2 lines, 5x8 font */
#define LCD_CMD_SET_DDRAM    0x80

static uint8_t lcd_addr = LCD_ADDR;
static char    lcd_shadow[2][17];              /* what is currently on screen */
static absolute_time_t lcd_retry_at;           /* skip writes until this time after an error */
static uint32_t lcd_fail_count;

static void pcf_write(uint8_t value) {
    i2c_write_timeout_us(LCD_I2C, lcd_addr, &value, 1, false, 10000);
}

/* Used only during the init sequence, which needs real delays */
static void lcd_write_nibble_slow(uint8_t nibble, uint8_t rs) {
    uint8_t data = (uint8_t)((nibble << 4) | rs | LCD_BACKLIGHT);
    pcf_write(data | LCD_EN);
    sleep_us(1);
    pcf_write(data);
    sleep_us(50);
}

static void lcd_command_slow(uint8_t cmd) {
    lcd_write_nibble_slow(cmd >> 4, 0);
    lcd_write_nibble_slow(cmd & 0x0F, 0);
    if (cmd == LCD_CMD_CLEAR) sleep_ms(2);
}

/* Adds one LCD byte (2 nibbles x EN high/low = 4 I2C bytes) to buf */
static int lcd_append_byte(uint8_t *buf, int n, uint8_t byte, uint8_t rs) {
    uint8_t hi = (uint8_t)((byte & 0xF0) | rs | LCD_BACKLIGHT);
    uint8_t lo = (uint8_t)((byte << 4) | rs | LCD_BACKLIGHT);
    buf[n++] = hi | LCD_EN;
    buf[n++] = hi;
    buf[n++] = lo | LCD_EN;
    buf[n++] = lo;
    return n;
}

/* Writes a full 16-character line (padded with spaces) if it changed. */
static void lcd_print_line(int row, const char *text) {
    char line[17];
    snprintf(line, sizeof line, "%-16.16s", text);
    if (memcmp(line, lcd_shadow[row], 16) == 0) return;      /* nothing to do */
    if (!time_reached(lcd_retry_at)) return;                 /* recovering from an error */

    uint8_t buf[17 * 4];
    int n = lcd_append_byte(buf, 0, LCD_CMD_SET_DDRAM | (row == 0 ? 0x00 : 0x40), 0);
    for (int i = 0; i < 16; i++) {
        n = lcd_append_byte(buf, n, (uint8_t)line[i], LCD_RS);
    }

    int ret = i2c_write_timeout_us(LCD_I2C, lcd_addr, buf, (size_t)n, false, 20000);
    if (ret == n) {
        memcpy(lcd_shadow[row], line, sizeof line);
    } else {
        if (++lcd_fail_count <= 5) {
            printf("[LCD] I2C write to 0x%02X failed (ret=%d)\n", lcd_addr, ret);
        }
        lcd_retry_at = make_timeout_time_ms(LCD_RETRY_MS);
    }
}

/* Finds the backpack: LCD_ADDR first, then the PCF8574 / PCF8574A address ranges */
static void lcd_find_address(void) {
    uint8_t dummy;
    if (i2c_read_timeout_us(LCD_I2C, LCD_ADDR, &dummy, 1, false, 5000) >= 0) {
        lcd_addr = LCD_ADDR;
        printf("[LCD] Found at 0x%02X\n", lcd_addr);
        return;
    }
    for (int addr = 0x20; addr <= 0x3F; addr++) {
        if (addr == 0x28) addr = 0x38;       /* skip the gap between the two ranges */
        if (i2c_read_timeout_us(LCD_I2C, (uint8_t)addr, &dummy, 1, false, 5000) >= 0) {
            lcd_addr = (uint8_t)addr;
            printf("[LCD] Found at 0x%02X\n", lcd_addr);
            return;
        }
    }
    printf("[LCD] Not found on the I2C bus - check wiring. Using 0x%02X anyway.\n", lcd_addr);
}

static void lcd_init(void) {
    i2c_init(LCD_I2C, 100 * 1000);   /* 100 kHz (PCF8574 maximum) */
    gpio_set_function(LCD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL_PIN, GPIO_FUNC_I2C);
    /* No internal pull-ups: the backpack has its own on SDA/SCL. */

    sleep_ms(50);                    /* LCD power-up time */
    lcd_find_address();

    /* Standard HD44780 reset sequence to force 4-bit mode */
    lcd_write_nibble_slow(0x03, 0);
    sleep_ms(5);
    lcd_write_nibble_slow(0x03, 0);
    sleep_us(150);
    lcd_write_nibble_slow(0x03, 0);
    lcd_write_nibble_slow(0x02, 0);

    lcd_command_slow(LCD_CMD_FUNCTION_SET);
    lcd_command_slow(LCD_CMD_DISPLAY_ON);
    lcd_command_slow(LCD_CMD_CLEAR);
    lcd_command_slow(LCD_CMD_ENTRY_MODE);

    memset(lcd_shadow, ' ', sizeof lcd_shadow);   /* screen is blank after clear */
    lcd_retry_at = get_absolute_time();
}

/* ====================================================================== */
/* Display contents                                                       */
/* ====================================================================== */

static bool volume_visible;
static absolute_time_t volume_hide_at;

/* Fills out[17] with the 16-character window of the song title to show now */
static void title_window(char *out) {
    int len = (int)strlen(song_title);
    if (len <= 16) {
        snprintf(out, 17, "%s", song_title);
        return;
    }

    /* Long title: scroll it, pausing briefly at the start of each pass */
    const int gap = (int)strlen(SCROLL_GAP);
    const int cycle = len + gap;
    int step = (int)(absolute_time_diff_us(title_changed_at, get_absolute_time()) /
                     (SCROLL_STEP_MS * 1000));
    int pos = step % (cycle + SCROLL_HOLD);
    pos = pos < SCROLL_HOLD ? 0 : pos - SCROLL_HOLD;

    for (int i = 0; i < 16; i++) {
        int k = (pos + i) % cycle;
        out[i] = k < len ? song_title[k] : SCROLL_GAP[k - len];
    }
    out[16] = '\0';
}

static void update_display(void) {
    char line[17];

    snprintf(line, sizeof line, "Mode %d", mode);
    lcd_print_line(0, line);

    if (volume_visible && time_reached(volume_hide_at)) {
        volume_visible = false;
    }

    if (volume_visible) {
        snprintf(line, sizeof line, "Volume: %d/%d", volume, VOLUME_MAX);
    } else if (!bt_connected) {
        snprintf(line, sizeof line, "Waiting for BT");
    } else if (song_title[0] == '\0') {
        snprintf(line, sizeof line, "BT connected");
    } else {
        title_window(line);
    }
    lcd_print_line(1, line);
}

/* ====================================================================== */
/* Buttons                                                                */
/* ====================================================================== */

typedef struct {
    uint pin;
    bool last_raw;            /* last reading, pressed = true */
    bool stable;              /* debounced state */
    absolute_time_t changed;  /* when last_raw last changed */
} button_t;

static void button_init(button_t *b, uint pin) {
    b->pin = pin;
    b->last_raw = false;
    b->stable = false;
    b->changed = get_absolute_time();
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);        /* button pulls the pin to GND when pressed */
}

/* Returns true once per press (on the debounced press edge). */
static bool button_was_pressed(button_t *b) {
    bool raw = !gpio_get(b->pin);   /* active low */
    absolute_time_t now = get_absolute_time();

    if (raw != b->last_raw) {
        b->last_raw = raw;
        b->changed = now;
    }

    if (raw != b->stable && absolute_time_diff_us(b->changed, now) > DEBOUNCE_US) {
        b->stable = raw;
        return raw;                 /* true only on press, not release */
    }
    return false;
}

/* ====================================================================== */
/* LED matrix (WS2812, driven by PIO + DMA)                               */
/* ====================================================================== */

/* WS2812 PIO program (same as the pico-examples ws2812.pio, pre-assembled)
 * .side_set 1 ; T1=2 T2=5 T3=3
 * bitloop: out x,1 side 0 [2] ; jmp !x do_zero side 1 [1]
 * do_one:  jmp bitloop side 1 [4]
 * do_zero: nop side 0 [4]                                                  */
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

static PIO      led_pio;
static uint     led_sm;
static int      led_dma = -1;
static uint32_t led_frame[NUM_LEDS];   /* one word per LED, GRB in the top 24 bits */

typedef struct { const char *name; uint8_t r, g, b; } color_t;
static const color_t led_colors[] = {
    { "Red",    255,   0,   0 },
    { "Orange", 255,  80,   0 },
    { "Yellow", 255, 200,   0 },
    { "Green",    0, 255,   0 },
    { "Cyan",     0, 255, 255 },
    { "Blue",     0,   0, 255 },
    { "Purple", 140,   0, 255 },
    { "Pink",   255,   0, 120 },
};
#define NUM_LED_COLORS (sizeof led_colors / sizeof led_colors[0])

/* The PIO is chosen at run time because I2S and the wireless chip use PIOs too */
static void led_init(void) {
    uint offset;
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &led_pio, &led_sm,
                                                           &offset, LED_PIN, 1, true)) {
        printf("[LED] No free PIO state machine - LED matrix disabled\n");
        return;
    }

    pio_gpio_init(led_pio, LED_PIN);
    pio_sm_set_consecutive_pindirs(led_pio, led_sm, LED_PIN, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, LED_PIN);
    sm_config_set_out_shift(&c, false, true, 24);    /* MSB first, autopull 24 bits (RGB) */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    const int cycles_per_bit = 2 + 5 + 3;            /* T1 + T2 + T3 */
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (800000.0f * cycles_per_bit));

    pio_sm_init(led_pio, led_sm, offset, &c);
    pio_sm_set_enabled(led_pio, led_sm, true);

    /* DMA streams a whole frame into the PIO, so the CPU never waits on the LEDs */
    led_dma = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(led_dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(led_pio, led_sm, true));
    dma_channel_configure(led_dma, &dc, &led_pio->txf[led_sm], led_frame, NUM_LEDS, false);
}

/* Sets every LED to one color. Skipped if the previous frame is still being sent. */
static void led_fill(const color_t *color) {
    if (led_dma < 0 || dma_channel_is_busy(led_dma)) return;

    uint32_t r = color->r * BRIGHTNESS / 255;
    uint32_t g = color->g * BRIGHTNESS / 255;
    uint32_t b = color->b * BRIGHTNESS / 255;
    uint32_t grb = (g << 24) | (r << 16) | (b << 8);   /* WS2812 order, left-aligned */

    for (int i = 0; i < NUM_LEDS; i++) led_frame[i] = grb;
    dma_channel_transfer_from_buffer_now(led_dma, led_frame, NUM_LEDS);
}

int main(void) {
    stdio_init_all();
    printf("\n=== Demo_Scripts starting ===\n");

    /* Set up I2S first: it claims DMA channel 0 and PIO0 SM0 by number,
     * and the other drivers then take whatever is still free. */
    struct audio_buffer_pool *pool = audio_init();

    if (cyw43_arch_init()) {
        printf("CYW43 (wireless chip) init failed\n");
        return 1;
    }

    lcd_init();
    led_init();

    button_t btn_up, btn_down, btn_mode;
    button_init(&btn_up, BTN_VOL_UP_PIN);
    button_init(&btn_down, BTN_VOL_DOWN_PIN);
    button_init(&btn_mode, BTN_MODE_PIN);

    bluetooth_init();

    uint led_color = 0;
    led_fill(&led_colors[led_color]);

    absolute_time_t next_led     = make_timeout_time_ms(LED_COLOR_MS);
    absolute_time_t next_display = get_absolute_time();
    absolute_time_t next_stats   = make_timeout_time_ms(5000);

    while (true) {
        cyw43_arch_poll();          /* runs BTstack: receive + decode */
        pump_audio(pool);           /* feed decoded audio to the speakers */

        /* Buttons and LED Matrix */
        if (button_was_pressed(&btn_mode)) {
            mode = (mode % NUM_MODES) + 1;          /* 1 -> 2 -> 3 -> 1 */
            printf("[BTN] Mode %d\n", mode);
            led_color = (led_color + 1) % NUM_LED_COLORS;
            led_fill(&led_colors[led_color]);
        }
        bool up = button_was_pressed(&btn_up);
        bool down = button_was_pressed(&btn_down);
        if (up || down) {
            if (up && volume < VOLUME_MAX) volume++;
            if (down && volume > VOLUME_MIN) volume--;
            volume_visible = true;
            volume_hide_at = make_timeout_time_ms(VOLUME_SHOW_MS);
            next_display = get_absolute_time();     /* show it right away */
            printf("[BTN] Volume %d\n", volume);
        }

        /* LCD (only writes when the text actually changes) */
        if (time_reached(next_display)) {
            next_display = make_timeout_time_ms(LCD_REFRESH_MS);
            update_display();
        }

        /* Serial stats */
        if (time_reached(next_stats)) {
            next_stats = make_timeout_time_ms(5000);
            if (sbc_decoder) {
                printf("[AUDIO] buffer %5lu/%d frames | played %lu | underruns %lu | dropped %lu\n",
                       (unsigned long)ring_level(), RING_FRAMES,
                       (unsigned long)stat_frames_played, (unsigned long)stat_underruns,
                       (unsigned long)stat_overflow_frames);
            }
        }
    }
}
