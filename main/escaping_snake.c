#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "led_strip.h"

#define LED_PIN GPIO_NUM_18
#define LED_COUNT 36
#define SNAKE_LENGTH 6
#define STEP_MS 85
#define SENSOR_CHANNEL ADC_CHANNEL_6 /* GPIO34 */
#define MOTION_THRESHOLD 250

static led_strip_handle_t strip;
static adc_oneshot_unit_handle_t adc;
static bool on_led = false;
static bool moving = false;
static int samples[8];
static int sample_pos = 0;
static int64_t armed_after = 0;

static void send_msg(const char *s) { printf("%s\n", s); fflush(stdout); }
static void clear_strip(void) { ESP_ERROR_CHECK(led_strip_clear(strip)); }
static void show_snake(int head) {
    clear_strip();
    for (int i = 0; i < SNAKE_LENGTH; i++) {
        int pixel = head - i;
        if (pixel >= 0 && pixel < LED_COUNT)
            ESP_ERROR_CHECK(led_strip_set_pixel(strip, pixel, 0, 40, 0));
    }
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}
static int sensor_read(void) {
    int total = 0, valid = 0;
    for (int i = 0; i < 8; i++) {
        int v;
        if (adc_oneshot_read(adc, SENSOR_CHANNEL, &v) == ESP_OK) { total += v; valid++; }
    }
    return valid ? total / valid : 0;
}
static void sensor_reset(void) {
    int v = sensor_read();
    for (int i = 0; i < 8; i++) samples[i] = v;
    sample_pos = 0;
    armed_after = (int64_t)esp_log_timestamp() + 1000;
}
static bool sensor_moved(void) {
    int current = sensor_read();
    int old = samples[sample_pos];
    samples[sample_pos] = current;
    sample_pos = (sample_pos + 1) % 8;
    return (int64_t)esp_log_timestamp() > armed_after && abs(current - old) > MOTION_THRESHOLD;
}
static void animate_out(void) {
    moving = true;
    for (int head = 0; head < LED_COUNT; head++) {
        show_snake(head);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
    on_led = true;
    moving = false;
    sensor_reset();
    send_msg("AT_LED");
}
static void animate_home(void) {
    moving = true;
    on_led = false;
    /* Head moves left, and the body follows from the right. */
    for (int head = LED_COUNT - 1; head >= -SNAKE_LENGTH; head--) {
        clear_strip();
        for (int i = 0; i < SNAKE_LENGTH; i++) {
            int pixel = head + i;
            if (pixel >= 0 && pixel < LED_COUNT)
                ESP_ERROR_CHECK(led_strip_set_pixel(strip, pixel, 0, 40, 0));
        }
        ESP_ERROR_CHECK(led_strip_refresh(strip));
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
    clear_strip();
    moving = false;
    send_msg("AT_HOME");
}
static void init_hardware(void) {
    led_strip_config_t sc = {
        .strip_gpio_num = LED_PIN, .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10000000,
        .mem_block_symbols = 64
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &strip));
    clear_strip();
    adc_oneshot_unit_init_cfg_t ac = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ac, &adc));
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, SENSOR_CHANNEL, &cc));
    sensor_reset();
}
static void command(const char *line) {
    if (strcmp(line, "PING") == 0) send_msg("PONG");
    else if (strcmp(line, "GO_LED") == 0 && !moving && !on_led) animate_out();
    else if (strcmp(line, "GO_HOME") == 0 && !moving && on_led) animate_home();
}
void app_main(void) {
    init_hardware();
    /* Same console-input approach as the working windshield project. */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    send_msg("READY");
    char line[32];
    size_t n = 0;
    int64_t last_check = 0;
    while (1) {
        int c = getchar();
        if (c == '\n' || c == '\r') {
            if (n) { line[n] = '\0'; command(line); n = 0; }
        } else if (c >= 32 && c <= 126) {
            if (n < sizeof(line) - 1) line[n++] = (char)c;
            else n = 0;
        }
        int64_t now = esp_log_timestamp();
        if (on_led && !moving && now - last_check >= 40) {
            last_check = now;
            if (sensor_moved()) { send_msg("MOTION"); animate_home(); }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}