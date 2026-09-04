# Sólo el encoder: es lo que pide el TP1.
#
# motor_driver.c y ultrasonic.c siguen en la carpeta pero quedan fuera del
# build. Para reincorporarlos, agregalos a SRCS y volvé a poner
# `target_link_libraries(${COMPONENT_LIB} PRIVATE m)` (lo usa la cinemática
# diferencial por fabsf).
idf_component_register(SRCS "pot.c" "main.c" "as5600.c"
                       INCLUDE_DIRS "." "include"
                       PRIV_REQUIRES micro_ros_espidf_component
                       REQUIRES driver esp_timer esp_adc)
