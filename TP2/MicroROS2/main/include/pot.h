#ifndef POT_H
#define POT_H

#include <stdint.h>

/* Potenciómetro sobre GPIO34 = ADC1_CH6 (pin solo-entrada, ideal para ADC).
 * Wiper -> GPIO34, extremos -> 3V3 y GND. */
#define PIN_ADC       GPIO_NUM_34
#define ADC_POT_CHAN  ADC_CHANNEL_6   /* GPIO34 corresponde a este canal */

/* ============================================================
 *  API pública
 * ============================================================ */
void    pot_init(void);        /* Configura la unidad ADC1 y el canal */
int     pot_get_raw(void);     /* Valor crudo (0..4095) */
int32_t pot_get_mv(void);      /* Milivoltios (0..3300) */

#endif /* POT_H */