/*
 * LCD mode / volume demo for the sound-reactive speaker
 *
 * Line 1 always shows "Mode N" (N cycles 1 -> 2 -> 3 -> 1 on the mode button).
 * Line 2 shows "Volume: N" for a few seconds after a volume button is pressed.
 *
 * Wiring (matches the project wiring diagram):
 *   LCD1602 I2C backpack: SDA -> GP4, SCL -> GP5, VCC -> 5V rail, GND -> GND
 *   Buttons (to GND, internal pull-ups): Vol+ -> GP13, Vol- -> GP14, Mode -> GP15
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

/* ---------- Configuration ---------- */
#define LCD_I2C          i2c0
#define LCD_SDA_PIN      4
#define LCD_SCL_PIN      5
#define LCD_ADDR         0x27      /* try 0x3F if the screen stays blank */

#define BTN_VOL_UP_PIN   13
#define BTN_VOL_DOWN_PIN 15
#define BTN_MODE_PIN     14

#define NUM_MODES        3
#define VOLUME_MIN       0
#define VOLUME_MAX       10
#define VOLUME_START     5
#define VOLUME_SHOW_MS   3000      /* how long line 2 shows the volume */
#define DEBOUNCE_US      20000     /* 20 ms */

/* ---------- LCD (HD44780 behind a PCF8574 I2C expander) ----------
 * PCF8574 bit layout on the common backpack:
 *   P0 = RS, P1 = RW, P2 = EN, P3 = backlight, P4..P7 = D4..D7
 * The LCD runs in 4-bit mode, so every byte is sent as two nibbles.
 */
#define LCD_RS         0x01
#define LCD_EN         0x04
#define LCD_BACKLIGHT  0x08

#define LCD_CMD_CLEAR        0x01
#define LCD_CMD_ENTRY_MODE   0x06  /* cursor moves right, no display shift */
#define LCD_CMD_DISPLAY_ON   0x0C  /* display on, cursor off, blink off */
#define LCD_CMD_FUNCTION_SET 0x28  /* 4-bit bus, 2 lines, 5x8 font */
#define LCD_CMD_SET_DDRAM    0x80

/* Debug counters: how many I2C writes to the LCD succeeded / failed */
static uint32_t i2c_ok_count = 0;
static uint32_t i2c_fail_count = 0;
static int last_i2c_error = 0;

/* Address actually used; starts at LCD_ADDR but switches to whatever the scan finds */
static uint8_t lcd_addr = LCD_ADDR;

static void pcf_write(uint8_t value) {
    /* Timeout version so a stuck bus can't hang the program silently */
    int ret = i2c_write_timeout_us(LCD_I2C, lcd_addr, &value, 1, false, 10000);
    if (ret == 1) {
        i2c_ok_count++;
    } else {
        i2c_fail_count++;
        last_i2c_error = ret;
        /* Only print the first few failures so the monitor isn't flooded */
        if (i2c_fail_count <= 5) {
            printf("[LCD] I2C write to 0x%02X FAILED (ret=%d: %s)\n",
                   lcd_addr, ret,
                   ret == PICO_ERROR_TIMEOUT ? "timeout - bus stuck?" :
                   ret == PICO_ERROR_GENERIC ? "no ACK - wrong address or not connected" :
                   "other error");
        } else if (i2c_fail_count == 6) {
            printf("[LCD] ...further I2C failures suppressed\n");
        }
    }
}

/* Probes every 7-bit address and prints the ones that ACK.
 * Returns the address to use for the LCD (LCD_ADDR if it answered,
 * otherwise the first device found), or -1 if nothing answered. */
static int i2c_scan(void) {
    printf("[I2C] Scanning bus (SDA=GP%d, SCL=GP%d)...\n", LCD_SDA_PIN, LCD_SCL_PIN);
    int found = 0;
    int best = -1;
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_timeout_us(LCD_I2C, (uint8_t)addr, &dummy, 1, false, 5000);
        if (ret >= 0) {
            printf("[I2C]   device found at 0x%02X%s\n", addr,
                   addr == LCD_ADDR ? "  <-- matches LCD_ADDR" : "");
            if (best < 0 || addr == LCD_ADDR) best = addr;
            found++;
        } else if (ret == PICO_ERROR_TIMEOUT) {
            printf("[I2C]   timeout at 0x%02X - bus may be stuck (check wiring / pull-ups)\n", addr);
            break;
        }
    }
    if (found == 0) {
        printf("[I2C] No devices found! Check SDA/SCL wiring, VCC/GND, and that SDA/SCL aren't swapped.\n");
    } else {
        printf("[I2C] Scan done, %d device(s) found.\n", found);
    }
    return best;
}

/* Reads SDA/SCL pad levels (works even while the pins are in I2C mode). */
static void print_line_levels(void) {
    bool sda = gpio_get(LCD_SDA_PIN);
    bool scl = gpio_get(LCD_SCL_PIN);
    printf("[I2C] Line levels: SDA(GP%d)=%d SCL(GP%d)=%d (expect 1 and 1)\n",
           LCD_SDA_PIN, sda, LCD_SCL_PIN, scl);
    if (!sda || !scl) {
        printf("[I2C] WARNING: a line is LOW at idle - LCD backpack unpowered, "
               "no pull-ups, or line shorted to GND.\n");
    }
}

static void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (uint8_t)((nibble << 4) | rs | LCD_BACKLIGHT);
    pcf_write(data | LCD_EN);   /* EN high */
    sleep_us(1);
    pcf_write(data);            /* EN low: LCD latches the nibble */
    sleep_us(50);               /* most commands finish within ~40 us */
}

static void lcd_send(uint8_t byte, uint8_t rs) {
    lcd_write_nibble(byte >> 4, rs);
    lcd_write_nibble(byte & 0x0F, rs);
}

static void lcd_command(uint8_t cmd) {
    lcd_send(cmd, 0);
    if (cmd == LCD_CMD_CLEAR) {
        sleep_ms(2);            /* clear takes ~1.5 ms */
    }
}

static void lcd_set_cursor(int row, int col) {
    uint8_t row_offset = (row == 0) ? 0x00 : 0x40;
    lcd_command(LCD_CMD_SET_DDRAM | (uint8_t)(row_offset + col));
}

/* Writes a full 16-character line, padding with spaces.
 * Overwriting the whole line avoids the flicker of a full clear. */
static void lcd_print_line(int row, const char *text) {
    lcd_set_cursor(row, 0);
    int i = 0;
    for (; i < 16 && text[i] != '\0'; i++) {
        lcd_send((uint8_t)text[i], LCD_RS);
    }
    for (; i < 16; i++) {
        lcd_send(' ', LCD_RS);
    }
}

static void lcd_init(void) {
    /* Check idle line levels before handing pins to I2C.
     * With pull-ups present, both lines should idle HIGH (1). */
    gpio_init(LCD_SDA_PIN);
    gpio_init(LCD_SCL_PIN);
    gpio_set_dir(LCD_SDA_PIN, GPIO_IN);
    gpio_set_dir(LCD_SCL_PIN, GPIO_IN);
    gpio_disable_pulls(LCD_SDA_PIN);
    gpio_disable_pulls(LCD_SCL_PIN);
    sleep_ms(1);
    bool sda_level = gpio_get(LCD_SDA_PIN);
    bool scl_level = gpio_get(LCD_SCL_PIN);
    printf("[I2C] Idle levels: SDA=%d SCL=%d (expect 1 and 1)\n", sda_level, scl_level);
    if (!sda_level || !scl_level) {
        printf("[I2C] WARNING: a line is LOW at idle - LCD backpack unpowered, "
               "no pull-ups, or line shorted to GND.\n");
    }

    uint actual = i2c_init(LCD_I2C, 100 * 1000);  /* 100 kHz */
    gpio_set_function(LCD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL_PIN, GPIO_FUNC_I2C);
    /* No internal pull-ups: the backpack already has its own on SDA/SCL. */
    printf("[I2C] i2c0 initialised at %u Hz\n", actual);

    sleep_ms(50);                   /* LCD power-up time */

    int found = i2c_scan();
    if (found >= 0) lcd_addr = (uint8_t)found;
}

/* HD44780 reset + configuration. Safe to call again to re-initialise. */
static void lcd_reset(void) {
    printf("[LCD] Sending init sequence to 0x%02X...\n", lcd_addr);
    uint32_t ok_before = i2c_ok_count, fail_before = i2c_fail_count;

    /* Standard HD44780 reset sequence to force 4-bit mode */
    lcd_write_nibble(0x03, 0);
    sleep_ms(5);
    lcd_write_nibble(0x03, 0);
    sleep_us(150);
    lcd_write_nibble(0x03, 0);
    lcd_write_nibble(0x02, 0);

    lcd_command(LCD_CMD_FUNCTION_SET);
    lcd_command(LCD_CMD_DISPLAY_ON);
    lcd_command(LCD_CMD_CLEAR);
    lcd_command(LCD_CMD_ENTRY_MODE);

    printf("[LCD] Init done: %lu I2C writes OK, %lu failed\n",
           (unsigned long)(i2c_ok_count - ok_before),
           (unsigned long)(i2c_fail_count - fail_before));
    if (i2c_fail_count == fail_before) {
        printf("[LCD] I2C looks fine. If the screen is blank but backlit, "
               "turn the blue contrast pot on the backpack.\n");
    }
}

/* ---------- Buttons ---------- */
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
        printf("[BTN] GP%u %s (pin reads %d)\n", b->pin,
               raw ? "PRESSED" : "released", !raw);
        return raw;                 /* true only on press, not release */
    }
    return false;
}

static void print_button_levels(const button_t *b, const char *name) {
    int level = gpio_get(b->pin);
    printf("[BTN] %-5s GP%-2u level=%d -> %s\n", name, b->pin, level,
           level ? "released (idle HIGH, OK)"
                 : "reads PRESSED - stuck LOW? (button rotated 90 deg on breadboard, or shorted to GND)");
}

/* ---------- Pin finder (debug) ----------
 * Watches every free GPIO with a pull-up and prints any pin that changes.
 * Press each button and the monitor tells you which GPIO it is really on.
 * Set to 0 once the buttons work. */
#define PIN_FINDER 1

static const uint finder_pins[] = {
    0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 26, 27, 28   /* skips GP4/5 (LCD) and wireless-chip pins */
};
#define NUM_FINDER_PINS (sizeof finder_pins / sizeof finder_pins[0])
static bool finder_last[NUM_FINDER_PINS];

static void pin_finder_init(void) {
    for (uint i = 0; i < NUM_FINDER_PINS; i++) {
        uint p = finder_pins[i];
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }
    sleep_ms(1);
    for (uint i = 0; i < NUM_FINDER_PINS; i++) {
        finder_last[i] = gpio_get(finder_pins[i]);
        if (!finder_last[i]) {
            printf("[FIND] GP%u is LOW at startup (something is pulling it to GND)\n",
                   finder_pins[i]);
        }
    }
    printf("[FIND] Pin finder on: press each button to see which GPIO it is on.\n");
}

static void pin_finder_poll(void) {
    for (uint i = 0; i < NUM_FINDER_PINS; i++) {
        bool level = gpio_get(finder_pins[i]);
        if (level != finder_last[i]) {
            finder_last[i] = level;
            printf("[FIND] GP%u -> %s\n", finder_pins[i], level ? "HIGH" : "LOW");
        }
    }
}

/* ---------- Display state ---------- */
static void draw_mode(int mode) {
    char line[17];
    snprintf(line, sizeof line, "Mode %d", mode);
    lcd_print_line(0, line);
}

static void draw_volume(int volume, bool visible) {
    char line[17];
    if (visible) {
        snprintf(line, sizeof line, "Volume: %d", volume);
    } else {
        line[0] = '\0';             /* blank line */
    }
    lcd_print_line(1, line);
}

int main(void) {
    stdio_init_all();

    /* Wait (up to 5 s) for the Serial Monitor to connect over USB,
     * otherwise the startup diagnostics are printed before anyone can see them. */
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }
    sleep_ms(200);
    printf("\n=== LCD_Test starting ===\n");

    lcd_init();     /* I2C bus setup + scan */
    lcd_reset();    /* HD44780 init sequence */

#if PIN_FINDER
    pin_finder_init();
#endif

    button_t btn_up, btn_down, btn_mode;
    button_init(&btn_up, BTN_VOL_UP_PIN);
    button_init(&btn_down, BTN_VOL_DOWN_PIN);
    button_init(&btn_mode, BTN_MODE_PIN);
    sleep_ms(1);
    print_button_levels(&btn_up, "Vol+");
    print_button_levels(&btn_down, "Vol-");
    print_button_levels(&btn_mode, "Mode");

    int mode = 1;
    int volume = VOLUME_START;
    bool volume_visible = false;
    absolute_time_t volume_hide_at = get_absolute_time();

    draw_mode(mode);
    draw_volume(volume, false);
    printf("[MAIN] Entering main loop. I2C OK=%lu fail=%lu\n",
           (unsigned long)i2c_ok_count, (unsigned long)i2c_fail_count);

    absolute_time_t next_heartbeat = make_timeout_time_ms(5000);

    while (true) {
        /* Heartbeat every 5 s so you can tell the program is still running */
        if (time_reached(next_heartbeat)) {
            printf("[MAIN] alive - mode=%d volume=%d, I2C OK=%lu fail=%lu\n",
                   mode, volume, (unsigned long)i2c_ok_count, (unsigned long)i2c_fail_count);
            print_button_levels(&btn_up, "Vol+");
            print_button_levels(&btn_down, "Vol-");
            print_button_levels(&btn_mode, "Mode");

            /* While the LCD has never answered, keep re-running the diagnostics
             * (the startup ones are often missed before the Serial Monitor connects). */
            if (i2c_ok_count == 0) {
                printf("\n[DIAG] LCD has not answered yet. Last error=%d (%s)\n",
                       last_i2c_error,
                       last_i2c_error == PICO_ERROR_TIMEOUT ? "timeout - bus stuck / no pull-ups" :
                       last_i2c_error == PICO_ERROR_GENERIC ? "no ACK - wrong address or not connected" :
                       "none/other");
                printf("[DIAG] Trying LCD address 0x%02X\n", lcd_addr);
                print_line_levels();
                int found = i2c_scan();
                if (found >= 0) {
                    lcd_addr = (uint8_t)found;
                    printf("[DIAG] Using address 0x%02X and re-initialising LCD\n", lcd_addr);
                    lcd_reset();
                    draw_mode(mode);
                    draw_volume(volume, volume_visible);
                }
                printf("\n");
            }
            next_heartbeat = make_timeout_time_ms(5000);
        }

#if PIN_FINDER
        pin_finder_poll();
#endif

        if (button_was_pressed(&btn_mode)) {
            mode = (mode % NUM_MODES) + 1;          /* 1 -> 2 -> 3 -> 1 */
            draw_mode(mode);
            printf("Mode %d\n", mode);
        }

        bool up = button_was_pressed(&btn_up);
        bool down = button_was_pressed(&btn_down);
        if (up || down) {
            if (up && volume < VOLUME_MAX) volume++;
            if (down && volume > VOLUME_MIN) volume--;
            volume_visible = true;
            volume_hide_at = make_timeout_time_ms(VOLUME_SHOW_MS);
            draw_volume(volume, true);
            printf("Volume %d\n", volume);
        }

        if (volume_visible && time_reached(volume_hide_at)) {
            volume_visible = false;
            draw_volume(volume, false);
        }

        sleep_ms(5);
    }
}