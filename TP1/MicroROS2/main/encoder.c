#include "encoder.h"

#include "driver/pulse_cnt.h"   /* nuevo driver PCNT (ESP-IDF v5.x): esp_driver_pcnt */
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "encoder";

static pcnt_unit_handle_t    s_unit   = NULL;
static pcnt_channel_handle_t s_chan_a = NULL;
static pcnt_channel_handle_t s_chan_b = NULL;

int32_t encoder_get_position(void)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_unit, &count));
    return (int32_t)count;
}

void encoders_init(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_unit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_LOW_LIMIT));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENC_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &chan_a_config, &s_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &chan_b_config, &s_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* flanco de subida de A */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); /* flanco de bajada de A */
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* B en alto -> mantener */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); /* B en bajo -> invertir */

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* flanco de subida de B */
        PCNT_CHANNEL_EDGE_ACTION_DECREASE)); /* flanco de bajada de B */
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* A en alto -> mantener */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); /* A en bajo -> invertir */

    gpio_set_pull_mode(PIN_ENC_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_ENC_B, GPIO_PULLUP_ONLY);

    /* 5) Habilitar, poner en cero y arrancar. */
    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));

    ESP_LOGI(TAG, "Encoder PCNT 4x listo (A=%d, B=%d, CPR=%d)",
             PIN_ENC_A, PIN_ENC_B, CPR);
}