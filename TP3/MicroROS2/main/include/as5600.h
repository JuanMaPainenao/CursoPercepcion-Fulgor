#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

#define AS5600_SDA_GPIO      GPIO_NUM_21
#define AS5600_SCL_GPIO      GPIO_NUM_22
#define AS5600_ADDR          0x36
#define AS5600_I2C_HZ        400000   /* 400 kHz*/


void     as5600_init(void);         /* Inicializa el bus I2C y el dispositivo */
uint16_t as5600_get_raw(void);      /* RAW ANGLE, sin escalar   (0..4095) */
uint16_t as5600_get_angle(void);    /* ANGLE, escalado por el chip (0..4095) */
float    as5600_get_degrees(void);  /* Grados 0..360 a partir del RAW */

#endif /* AS5600_H */