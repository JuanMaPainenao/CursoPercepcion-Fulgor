#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "driver/gpio.h"

#define PIN_ENC_A   GPIO_NUM_21   /* Canal A */
#define PIN_ENC_B   GPIO_NUM_26   /* Canal B */

#define PPR         600           /* Pulsos por vuelta de UN canal (LPD3806-600: 600 PPR) */
#define CPR         (4 * PPR)     /* Conteos por vuelta en cuadratura 4x => 2400 */
#define PERIOD_MS   1000          /* Periodo de publicacion Y ventana de velocidad (ms) */
#define PCNT_HIGH_LIMIT   30000
#define PCNT_LOW_LIMIT   -30000
#define ENC_GLITCH_NS     1000

void    encoders_init(void);          /* Configura y arranca el PCNT en 4x */
int32_t encoder_get_position(void);   /* Posicion: tics acumulados (con signo) */

#endif /* ENCODER_H */