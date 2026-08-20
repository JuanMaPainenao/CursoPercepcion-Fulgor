#include "pot.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

/* Handle del ADC: persiste entre llamadas, por eso es file-scope.
 * Los configs son locales a pot_init(): solo se usan al inicializar. */
static adc_oneshot_unit_handle_t adc1_handle = NULL;

void pot_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // ESP32 -> 12 bits, 0..4095
        .atten    = ADC_ATTEN_DB_12,       // rango ~0..3.3 V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_POT_CHAN,
                                               &chan_config));
}

int pot_get_raw(void)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_POT_CHAN, &raw));
    return raw;   // 0..4095
}

int32_t pot_get_mv(void)
{
    return (pot_get_raw() * 3300) / 4095;   // 0..3300 mV
}