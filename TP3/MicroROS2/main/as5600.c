#include "as5600.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#define REG_STATUS       0x0B   /* 1 byte: bits de detección del imán */
#define REG_RAW_ANGLE    0x0C   /* 0x0C alto, 0x0D bajo */
#define REG_ANGLE        0x0E   /* 0x0E alto, 0x0F bajo */

#define I2C_TIMEOUT_MS   100

static const char *TAG = "as5600";

/* Handles: persisten entre llamadas, por eso son file-scope. */
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

/* Lee un registro de 12 bits (par alto/bajo) y descarta los 4 bits sobrantes. */
static uint16_t read_reg16(uint8_t reg_high)
{
    uint8_t data[2] = {0};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(
        dev_handle, &reg_high, 1, data, 2, I2C_TIMEOUT_MS));

    return (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF;   /* 0..4095 */
}

void as5600_init(void)
{
    /* 1) Crear el bus I2C (el ESP32 como maestro). */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = AS5600_SDA_GPIO,
        .scl_io_num        = AS5600_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  /* respaldo; el módulo ya trae pull-ups */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    /* Diagnóstico temporal: lista lo que responde en el bus. */
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        if (i2c_master_probe(bus_handle, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo I2C encontrado en 0x%02X", addr);
        }
    }
    /* 2) Agregar el AS5600 al bus, con su dirección y la velocidad de 400 kHz. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_ADDR,
        .scl_speed_hz    = AS5600_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    uint8_t reg = REG_STATUS;
    uint8_t status = 0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(
        dev_handle, &reg, 1, &status, 1, I2C_TIMEOUT_MS));

    if (status & 0x20) {   /* bit 5 = MD */
        ESP_LOGI(TAG, "Iman detectado (STATUS=0x%02X)", status);
    } else {
        ESP_LOGW(TAG, "Iman NO detectado (STATUS=0x%02X). Revisar posicion.", status);
    }
}

uint16_t as5600_get_raw(void)
{
    return read_reg16(REG_RAW_ANGLE);
}

uint16_t as5600_get_angle(void)
{
    return read_reg16(REG_ANGLE);
}

float as5600_get_degrees(void)
{
    return as5600_get_raw() * 360.0f / 4096.0f;   /* 0..360 */
}