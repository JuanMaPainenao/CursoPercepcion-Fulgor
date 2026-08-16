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

    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
    ESP_LOGI(TAG, "Agent configurado en %s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

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
