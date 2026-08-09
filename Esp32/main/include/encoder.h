#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "driver/gpio.h"

/* ============================================================
 *  Configuración del encoder de cuadratura
 * ============================================================
 *  UN solo encoder de cuadratura: dos señales A y B en dos GPIO.
 *  La señal Z (índice) NO se usa acá porque medimos posición y
 *  velocidad RELATIVAS. Se agregaría solo para homing absoluto.
 *
 *  Para escalar a dos ruedas: duplicar unidad + canales con
 *  otros dos pines (el ESP32 tiene 8 unidades PCNT).
 * ============================================================ */

#define PIN_ENC_A   GPIO_NUM_21   /* Canal A */
#define PIN_ENC_B   GPIO_NUM_26   /* Canal B */

#define PPR         20            /* Pulsos por vuelta de UN canal (dato del encoder) */
#define CPR         (4 * PPR)     /* Conteos por vuelta en cuadratura 4x => 80 */
#define PERIOD_MS   1000          /* Ventana de medición de velocidad (ms) */

/* Límites del contador de hardware.
 *
 * El contador del PCNT es de 16 bits CON SIGNO: sólo puede representar
 * -32768..32767. Cuando llega a high_limit o low_limit el hardware lo
 * resetea a 0 solo. Con la bandera `accum_count` (ver encoder.c) el driver
 * de ESP-IDF suma ese salto en un acumulador interno, y así la cuenta que
 * leemos se extiende más allá de los 16 bits.
 *
 * Conviene poner estos límites lo MÁS ANCHOS posible dentro del rango de
 * 16 bits: cuanto menos seguido se toca el límite, menos interrupciones
 * hay y menor es la probabilidad de leer justo en la ventana de pocos
 * microsegundos que va entre el reset del hardware y la acumulación en
 * la ISR (que es el único momento en que la lectura sería incorrecta). */
#define PCNT_HIGH_LIMIT   30000
#define PCNT_LOW_LIMIT   -30000

/* Filtro de glitches: descarta pulsos más cortos que este tiempo
 * (elimina ruido eléctrico y rebote de contactos).
 * Referencia: a 3000 RPM con CPR=80 hay un conteo cada ~250 us, así que
 * 1 us es holgadamente seguro. Si el encoder es mecánico y rebota mucho,
 * se puede subir (el máximo es ~12.7 us con APB a 80 MHz). */
#define ENC_GLITCH_NS     1000

/* ============================================================
 *  API pública
 * ============================================================ */
void    encoders_init(void);          /* Configura y arranca el PCNT + timer */
int32_t encoder_get_position(void);   /* Posición: tics acumulados (con signo) */
float   encoder_get_rpm(void);        /* Velocidad en RPM (última ventana) */
float   encoder_get_angle_deg(void);  /* Ángulo en grados, derivado de la posición */

#endif /* ENCODER_H */
