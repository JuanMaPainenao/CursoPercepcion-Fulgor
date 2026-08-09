#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"

/* ============================================================
 *  PRIMERA ENTREGA: salida por SERIAL (USB), sin micro-ROS.
 *
 *  Formato de línea:  posicion_tics,velocidad_rpm\n
 *  (CSV de dos campos). El nodo ROS2 con pyserial parsea
 *  exactamente este formato. Para la entrega final, este bucle
 *  se reemplaza por el publisher de micro-ROS (el firmware del
 *  encoder NO cambia).
 *
 *  Dos detalles del stream serial que hay que tener presentes:
 *
 *   - ESP-IDF traduce el '\n' a "\r\n" en la consola
 *     (CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF). Por eso el nodo de
 *     ROS2 hace .strip() antes de parsear.
 *
 *   - Por este mismo UART también salen el log del bootloader y los
 *     ESP_LOGI. El nodo descarta toda línea que no sean exactamente
 *     dos campos numéricos separados por coma, así que esa basura
 *     de arranque no molesta.
 * ============================================================ */
void app_main(void)
{
    encoders_init();

    while (1) {
        int32_t pos = encoder_get_position();
        float   rpm = encoder_get_rpm();

        /* %ld con cast a long: portable para int32_t en Xtensa.
         * Nota: `pos` es el valor instantáneo y `rpm` es el de la última
         * ventana ya cerrada, así que la velocidad puede venir hasta una
         * ventana "atrasada" respecto de la posición. Con 1 Hz alcanza;
         * si hiciera falta correlación exacta habría que latchear ambos
         * dentro de speed_timer_cb(). */
        printf("%ld,%.2f\n", (long)pos, rpm);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}
