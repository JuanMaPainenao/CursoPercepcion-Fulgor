#ifndef POT_H
#define POT_H

#include <stdint.h>

#define PIN_ADC       GPIO_NUM_34
#define ADC_POT_CHAN  ADC_CHANNEL_6   /* GPIO34 corresponde a este canal */

/* ============================================================
 *  Calibración FIJA del sensor
 * ============================================================ */
#define POT_VMIN_V    0.00f   /* tensión que corresponde a 0 %   */
#define POT_VMAX_V    2.67f   /* tensión que corresponde a 100 % */

/* ============================================================
 *  API pública
 * ============================================================ */
void    pot_init(void);        /* Configura la unidad ADC1 y el canal */
int     pot_get_raw(void);     /* Valor crudo (0..4095) */
int32_t pot_get_mv(void);      /* Milivoltios (0..3300) */

#endif /* POT_H */