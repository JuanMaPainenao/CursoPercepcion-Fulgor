#include "encoder.h"

#include "driver/pulse_cnt.h"   /* nuevo driver PCNT (ESP-IDF v5.x): esp_driver_pcnt */
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "encoder";

/* ---- Handles del periférico ---- */
static pcnt_unit_handle_t    s_unit        = NULL;
static pcnt_channel_handle_t s_chan_a      = NULL;
static pcnt_channel_handle_t s_chan_b      = NULL;
static esp_timer_handle_t    s_speed_timer = NULL;

/* ---- Estado de velocidad ----
 * s_last_pos lo toca SOLO el callback del timer (un único hilo), así que no
 * necesita protección. s_rpm lo escribe el timer y lo lee app_main; es un
 * float de 32 bits alineado, o sea que la escritura es atómica en Xtensa:
 * alcanza con `volatile` para que el compilador no lo cachee en un registro. */
static int32_t        s_last_pos = 0;
static volatile float s_rpm      = 0.0f;

/* ------------------------------------------------------------
 *  Posición: tics acumulados con signo.
 *
 *  No hace falta acumulador propio ni sección crítica: al crear la unidad
 *  con `flags.accum_count = 1`, pcnt_unit_get_count() ya devuelve
 *      conteo_del_hardware + lo_acumulado_por_la_ISR_en_cada_reset
 *  y el driver protege ese par con su propio spinlock, así que la lectura
 *  es coherente aunque la ISR salte en el otro núcleo.
 * ------------------------------------------------------------ */
int32_t encoder_get_position(void)
{
    int count = 0;
    /* Sólo puede fallar si s_unit es NULL, o sea si alguien se olvidó de
     * llamar a encoders_init(). Que aborte con un mensaje claro es mejor
     * que devolver 0 en silencio. */
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_unit, &count));
    return (int32_t)count;
}

float encoder_get_rpm(void)
{
    return s_rpm;
}

float encoder_get_angle_deg(void)
{
    return (float)encoder_get_position() / CPR * 360.0f;
}

/* ------------------------------------------------------------
 *  Timer periódico: calcula RPM por diferencia de posición.
 *  vueltas_en_la_ventana * (ms_por_minuto / ventana_en_ms)
 *
 *  Resolución: con CPR=80 y ventana de 1 s, el escalón mínimo es
 *  1 tic/s = 0.75 RPM. Para medir más fino hay que agrandar la ventana
 *  (más resolución, más retardo) o usar un encoder de más PPR.
 * ------------------------------------------------------------ */
static void speed_timer_cb(void *arg)
{
    int32_t pos   = encoder_get_position();
    int32_t delta = pos - s_last_pos;
    s_last_pos    = pos;

    s_rpm = (float)delta / CPR * (60000.0f / PERIOD_MS);
}

/* ------------------------------------------------------------
 *  Inicialización del PCNT en cuadratura 4x
 * ------------------------------------------------------------ */
void encoders_init(void)
{
    /* 1) Unidad PCNT.
     *    high/low_limit definen dónde el HW resetea el contador a 0.
     *    accum_count = 1 le pide al driver que instale su propia ISR y
     *    acumule esos saltos, extendiendo el contador de 16 bits a 32. */
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_unit));

    /* 2) Filtro de glitches (ruido / rebote). */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENC_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filter_config));

    /* 3) Dos canales -> decodificación en cuadratura.
     *    Canal A: cuenta flancos de A, mira el NIVEL de B para saber el sentido.
     *    Canal B: cuenta flancos de B, mira el NIVEL de A. */
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

    /* 4) Acciones de conteo: ESTO decodifica el sentido de giro.
     *
     *    Firmas reales del driver (¡el orden importa!):
     *      pcnt_channel_set_edge_action (chan, accion_flanco_SUBIDA,
     *                                          accion_flanco_BAJADA)
     *      pcnt_channel_set_level_action(chan, accion_nivel_ALTO,
     *                                          accion_nivel_BAJO)
     *
     *    Cada canal cuenta en subida Y en bajada -> 4 conteos por período
     *    de cuadratura => modo 4x.
     *
     *    Cómo decodifica el sentido: el canal A suma en flanco de subida y
     *    resta en bajada; INVERSE dice "dá vuelta esa regla si el nivel de B
     *    está en bajo". El canal B hace lo simétrico. Resultado: en un
     *    sentido de giro los 4 flancos de un período suman +1 cada uno, y en
     *    el otro sentido restan -1 cada uno.
     *
     *    Si al probar el giro te queda con el signo al revés, la corrección
     *    es intercambiar PIN_ENC_A y PIN_ENC_B en encoder.h (o dar vuelta
     *    INCREASE/DECREASE en los dos canales). No toques uno solo. */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* flanco de subida de A */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); /* flanco de bajada de A */
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* B en alto  -> mantener */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); /* B en bajo  -> invertir */

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* flanco de subida de B */
        PCNT_CHANNEL_EDGE_ACTION_DECREASE)); /* flanco de bajada de B */
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* A en alto  -> mantener */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); /* A en bajo  -> invertir */

    /* 5) Watchpoints en los límites.
     *    Son OBLIGATORIOS para que accum_count funcione: la acumulación se
     *    hace dentro de la ISR del evento "llegué al límite alto/bajo", y esa
     *    ISR sólo corre si el watchpoint está dado de alta.
     *
     *    OJO: acá NO llamamos a pcnt_unit_register_event_callbacks(). No hace
     *    falta ningún callback nuestro, y además registrar uno con
     *    on_reach = NULL DESHABILITA la interrupción del PCNT y rompería
     *    silenciosamente la acumulación. */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_LOW_LIMIT));

    /* 6) Pull-ups internos (por si el encoder es "pelado", sin pull-ups propios).
     *    Va DESPUÉS de pcnt_new_channel(), que es quien configura el GPIO:
     *    al revés nos lo pisaría. Si tu módulo ya trae pull-ups, es inofensivo. */
    gpio_set_pull_mode(PIN_ENC_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_ENC_B, GPIO_PULLUP_ONLY);

    /* 7) Habilitar, limpiar y arrancar el contador.
     *    El clear también deja en cero el acumulador interno del driver, y es
     *    lo que hace efectivos los valores de límite recién configurados. */
    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));

    /* 8) Timer periódico para la velocidad. Corre en la tarea de esp_timer
     *    (no en contexto de ISR), así que puede tomar locks tranquilo. */
    const esp_timer_create_args_t timer_args = {
        .callback = &speed_timer_cb,
        .name     = "speed_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_speed_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_speed_timer,
                                             (uint64_t)PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Encoder PCNT 4x listo (A=%d, B=%d, CPR=%d)",
             PIN_ENC_A, PIN_ENC_B, CPR);
}
