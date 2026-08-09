idf_component_register(
    SRCS "main.c" "encoder.c"
    INCLUDE_DIRS "include"
    # esp_driver_gpio es PÚBLICO porque encoder.h (que está en INCLUDE_DIRS)
    # incluye driver/gpio.h para los GPIO_NUM_*.
    # Antes acá figuraba el componente paraguas `driver`, que arrastra todos
    # los esp_driver_* aunque no se usen; con MINIMAL_BUILD ON conviene pedir
    # sólo lo que realmente se necesita.
    REQUIRES esp_driver_pcnt esp_driver_gpio esp_timer
)
