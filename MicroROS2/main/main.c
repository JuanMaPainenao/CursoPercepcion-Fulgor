#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "encoder.h"

/* ============================================================
 *  TP1 — "Del Encoder a ROS2": nodo micro-ROS sobre WiFi/UDP.
 *
 *  Publica:
 *    /encoder/position   std_msgs/Int32     posición en tics (cuadratura 4x, con signo)
 *    /encoder/velocity   std_msgs/Float32   velocidad en RPM (ventana de PERIOD_MS)
 *
 *  Son los mismos tópicos y tipos que consume
 *  Ros2/src/monitor_package/monitor_package/monitor_subscriber.py,
 *  así que ese nodo funciona sin ningún cambio.
 *
 *  Transporte: UDP4 contra el Agent en CONFIG_MICRO_ROS_AGENT_IP:PUERTO
 *  (menuconfig -> micro-ROS Settings). Del lado de la PC:
 *      ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 *  Nota sobre el alcance: este main quedó reducido al encoder, que es lo que
 *  pide el TP. Los drivers del robot (motor_driver.c, ultrasonic.c) siguen en
 *  la carpeta pero salieron del build (ver main/CMakeLists.txt); el servicio
 *  ControlRobot se sacó porque el paquete `interfaces` no está en el repo y
 *  rompía la compilación.
 * ============================================================ */

#define RCCHECK(fn) { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Fallo en la linea %d: %d. Abortando.", __LINE__, (int)rc); \
        vTaskDelete(NULL); \
    } }

#define RCSOFTCHECK(fn) { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "Fallo en la linea %d: %d. Continuando.", __LINE__, (int)rc); \
    } }

#define MICRO_ROS_APP_STACK      24000
#define MICRO_ROS_APP_TASK_PRIO  5

/* Dominio DDS en el que se crea el nodo. Tiene que coincidir con el
 * ROS_DOMAIN_ID de la PC (el de tu .bashrc es 33). */
#define ROS_DOMAIN_ID  33

static const char *TAG = "micro_ros";

static rcl_publisher_t        s_pos_pub;
static rcl_publisher_t        s_vel_pub;
static std_msgs__msg__Int32   s_pos_msg;
static std_msgs__msg__Float32 s_vel_msg;

/* ------------------------------------------------------------
 *  Una publicación por ventana de medición.
 *
 *  Los dos valores se leen acá, juntos, y recién después se publican: así el
 *  par (posición, velocidad) que recibe el monitor corresponde al mismo
 *  instante de muestreo.
 *
 *  Queda igual una asimetría propia de la medición, que conviene tener
 *  presente al leer el CSV: la posición es el valor de ESTE instante, y la
 *  RPM es el promedio de la última ventana cerrada de PERIOD_MS.
 * ------------------------------------------------------------ */
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) {
        return;
    }

    s_pos_msg.data = encoder_get_position();
    s_vel_msg.data = encoder_get_rpm();

    /* RCSOFTCHECK y no RCCHECK: si se cae el WiFi o el Agent, la publicación
     * falla pero la tarea tiene que seguir viva para reconectar. */
    RCSOFTCHECK(rcl_publish(&s_pos_pub, &s_pos_msg, NULL));
    RCSOFTCHECK(rcl_publish(&s_vel_pub, &s_vel_msg, NULL));

    ESP_LOGI(TAG, "pos = %ld tics | vel = %.2f rpm",
             (long)s_pos_msg.data, s_vel_msg.data);
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    /* Dominio DDS. HAY que setearlo explícitamente.
     *
     * micro-ROS NO hereda el ROS_DOMAIN_ID del Agent: es el CLIENTE el que
     * decide el dominio y se lo pide al Agent en el CREATE_PARTICIPANT. Y el
     * default no es "el del entorno" sino un 0 hardcodeado:
     *     rmw_init.c:58    init_options->domain_id = 0;
     *     rmw_init.c:210   context->actual_domain_id = options->domain_id;
     *
     * Si esto queda sin setear, el nodo se crea y publica perfecto en el
     * dominio 0, pero desde una PC con ROS_DOMAIN_ID=33 no lo ve nadie:
     * `ros2 topic list` no muestra los tópicos y `echo` se queda mudo. */
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
    ESP_LOGI(TAG, "Agent configurado en %s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

    /* Esperar a que el Agent responda ANTES de inicializar. Si arranca la
     * placa antes que el Agent, rclc_support_init falla y no hay reintento:
     * la placa quedaría muerta hasta un reset manual.
     *
     * OJO con cuál de las dos variantes se usa acá. La de dos argumentos,
     * rmw_uros_ping_agent(timeout, intentos), NO mira las init_options: usa
     * la dirección compilada por defecto, que para transporte UDP es
     *     #define RMW_UXRCE_DEFAULT_IP "127.0.0.1"
     * (rmw_microxrcedds_c/config.h). O sea que la placa se pinguea a sí misma,
     * nunca recibe respuesta y este while no termina nunca — se queda colgado
     * justo acá, con el WiFi conectado y el Agent corriendo, sin que llegue un
     * solo datagrama.
     *
     * La variante _options sí usa el transporte configurado en rmw_options,
     * que es donde rmw_uros_options_set_udp_address() dejó la IP del Agent. */
    ESP_LOGI(TAG, "Esperando al micro-ROS Agent...");
    while (rmw_uros_ping_agent_options(500, 1, rmw_options) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Agent encontrado");
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_encoder", "", &support));
    ESP_LOGI(TAG, "Nodo /esp32_encoder creado");

    /* init_default => QoS RELIABLE con depth 10.
     *
     * No es un default cómodo, es un requisito: monitor_subscriber.py empareja
     * el i-ésimo mensaje de posición con el i-ésimo de velocidad, y eso sólo
     * vale si ninguno de los dos tópicos pierde mensajes. Con best-effort (y
     * más sobre WiFi) una sola pérdida corre la columna de velocidad del CSV
     * de forma permanente. Si alguna vez necesitás best-effort, hay que mandar
     * las dos magnitudes en UN solo mensaje. */
    RCCHECK(rclc_publisher_init_default(
        &s_pos_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder/position"));

    RCCHECK(rclc_publisher_init_default(
        &s_vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "encoder/velocity"));

    /* Misma cadencia que la ventana de velocidad: publicar más rápido que
     * PERIOD_MS sólo repetiría la misma RPM. */
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &timer, &support, RCL_MS_TO_NS(PERIOD_MS), timer_callback, true));

    /* Un solo handle: el timer. No hay suscripciones ni servicios. */
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    s_pos_msg.data = 0;
    s_vel_msg.data = 0.0f;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);   /* cede CPU al idle task */
    }

    /* Inalcanzable, pero deja asentado qué habría que liberar. */
    RCCHECK(rcl_publisher_fini(&s_pos_pub, &node));
    RCCHECK(rcl_publisher_fini(&s_vel_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    /* Levanta WiFi con el SSID/password de menuconfig y bloquea hasta tener IP. */
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    /* El encoder arranca primero: así el PCNT ya cuenta y el timer de
     * velocidad ya tiene una ventana cerrada para cuando aparezca el Agent. */
    encoders_init();

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}
