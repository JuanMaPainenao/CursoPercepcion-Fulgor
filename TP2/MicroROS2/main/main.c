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
#include <std_msgs/msg/float32.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "pot.h"


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

#define ROS_DOMAIN_ID       33
#define PUBLISH_PERIOD_MS   100
static const char *TAG = "micro_ros";

static rcl_publisher_t        s_pos_pub;   /* posición 0..100 %   */
static rcl_publisher_t        s_volt_pub;  /* voltaje  0..3.3 V    */
static std_msgs__msg__Float32 s_pos_msg;
static std_msgs__msg__Float32 s_volt_msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    /* Valores extremos aprendidos automáticamente */
    static int raw_min = 4095;
    static int raw_max = 0;

    int raw = pot_get_raw();

    /* Auto-calibración */
    if (raw < raw_min) {
        raw_min = raw;
    }

    if (raw > raw_max) {
        raw_max = raw;
    }

    /* Posición normalizada 0-100 % */
    float position = 0.0f;

    if (raw_max > raw_min) {
        position = ((float)(raw - raw_min) * 100.0f) /
                   ((float)(raw_max - raw_min));
    }

    /* Saturación por seguridad */
    if (position < 0.0f) {
        position = 0.0f;
    }

    if (position > 100.0f) {
        position = 100.0f;
    }

    s_pos_msg.data = position;

    /*
     * Voltaje del ADC.
     * Esto NO se autoescala: representa la tensión medida.
     */
    s_volt_msg.data = (raw * 3.3f) / 4095.0f;

    RCSOFTCHECK(rcl_publish(&s_pos_pub, &s_pos_msg, NULL));
    RCSOFTCHECK(rcl_publish(&s_volt_pub, &s_volt_msg, NULL));

    ESP_LOGI(TAG,
             "raw=%4d | min=%4d | max=%4d | pos=%.1f %% | volt=%.2f V",
             raw,
             raw_min,
             raw_max,
             s_pos_msg.data,
             s_volt_msg.data);
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
    RCCHECK(rclc_node_init_default(&node, "esp32_pot", "", &support));
    ESP_LOGI(TAG, "Nodo /esp32_pot creado");

    RCCHECK(rclc_publisher_init_default(
        &s_pos_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pot/position"));

    RCCHECK(rclc_publisher_init_default(
        &s_volt_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pot/voltage"));

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback, true));

    /* Un solo handle: el timer. No hay suscripciones ni servicios. */
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    s_pos_msg.data  = 0.0f;
    s_volt_msg.data = 0.0f;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);   /* cede CPU al idle task */
    }

    RCCHECK(rcl_publisher_fini(&s_pos_pub,  &node));
    RCCHECK(rcl_publisher_fini(&s_volt_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    /* Levanta WiFi con el SSID/password de menuconfig y bloquea hasta tener IP. */
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    pot_init();

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}