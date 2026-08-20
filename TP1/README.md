# TP1 — Del Encoder a ROS2: Lectura de Velocidad y Posición

Un ESP32 lee un encoder incremental de cuadratura con el módulo **PCNT** en modo
**4x** y publica posición y velocidad como nodo **micro-ROS** por WiFi. En la PC,
el **micro-ROS Agent** hace de puente hacia ROS 2 y un nodo Python muestra los
datos en vivo y los registra en un CSV.

```
  Encoder A/B ──► ESP32 (PCNT 4x)          PC
                  micro-ROS client         micro-ROS Agent        monitor_subscriber
                        │                        │                        │
                        └──── WiFi / UDP4 ───────┤                        │
                                 :8888           └──── DDS (dominio 33) ──┘
                                                    /encoder/position   Int32   (tics)
                                                    /encoder/velocity   Float32 (RPM)
```

## Puesta en marcha

### Paso 0 — Configuración (solo la primera vez, o si cambió la IP de la PC)

El firmware necesita la IP de la PC donde corre el Agent. Averiguala:

```bash
hostname -I | awk '{print $1}'
```

Si no coincide con la que está compilada, entrá al menú y corregila:

```bash
source ~/.espressif/v5.5/esp-idf/export.sh
cd ~/Documents/Fulgor/CursoPerception/Proyecto1/MicroROS2
idf.py menuconfig
```

- `micro-ROS Settings` → **micro-ROS Agent IP** y **micro-ROS Agent Port** (`8888`)
- `micro-ROS Settings` → `WiFi Configuration` → **SSID** y **password** de la red

> La PC y el ESP32 tienen que estar en la **misma red**. El SSID y la contraseña
> quedan guardados en texto plano en `MicroROS2/sdkconfig`.

**El `ROS_DOMAIN_ID` tiene que coincidir de los dos lados.** En la PC ya está
puesto en `~/.bashrc` (`export ROS_DOMAIN_ID=33`); en el firmware está fijo en
`MicroROS2/main/main.c` (`#define ROS_DOMAIN_ID 33`). micro-ROS **no** hereda el
dominio del Agent: si no coinciden, el nodo publica perfecto pero desde la PC no
lo ve nadie — `ros2 topic list` no muestra nada y `echo` se queda mudo.

### Paso 1 — Compilar y flashear el ESP32

```bash
source ~/.espressif/v5.5/esp-idf/export.sh
cd ~/Documents/Fulgor/CursoPerception/Proyecto1/MicroROS2
idf.py build
idf.py -p /dev/ttyACM0 flash
```

La primera compilación tarda bastante: construye `libmicroros`.

### Paso 2 — Compilar los workspaces de ROS 2 (solo la primera vez)

```bash
source /opt/ros/jazzy/setup.bash

cd ~/Documents/Fulgor/CursoPerception/Proyecto1/TP1Agentev2
colcon build

cd ~/Documents/Fulgor/CursoPerception/Proyecto1/Ros2
colcon build --packages-select monitor_package
```

> Cada vez que edites un `.py` de `monitor_package` hay que volver a correr ese
> `colcon build`: el `install/` guarda una **copia**, no un symlink.

---

## Correr el sistema — cuatro terminales

### Terminal 1 — micro-ROS Agent

```bash
cd ~/Documents/Fulgor/CursoPerception/Proyecto1/TP1Agentev2
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
```

Cuando la placa se conecta, aparecen `create_participant`, `create_topic` y
`create_publisher`. Si no aparece nada, el problema es de red o de IP.

### Terminal 2 — Log de la placa (opcional pero recomendado)

```bash
source ~/.espressif/v5.5/esp-idf/export.sh
cd ~/Documents/Fulgor/CursoPerception/Proyecto1/MicroROS2
idf.py -p /dev/ttyACM0 monitor
```

Tenés que ver, en orden:

```
Encoder PCNT 4x listo (A=21, B=26, CPR=80)
Agent configurado en 172.16.0.39:8888
Esperando al micro-ROS Agent...
Agent encontrado
Nodo /esp32_encoder creado
pos = ... tics | vel = ... rpm
```

El orden de arranque no importa: si la placa no encuentra el Agent, se queda
pingueando hasta que aparezca. Salir del monitor: `Ctrl-]`.

### Terminal 3 — Verificar antes de capturar

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list                        # debe aparecer /esp32_encoder
ros2 topic list                       # /encoder/position y /encoder/velocity
ros2 topic hz /encoder/position       # ~1 Hz
ros2 topic echo /encoder/velocity     # valores en RPM
```

Si `ros2 topic hz` no muestra nada, **no sigas**: revisá el `ROS_DOMAIN_ID` y la
IP del Agent. El CSV te va a salir vacío igual.

### Terminal 4 — Monitor: visualización + registro

```bash
cd ~/Documents/Fulgor/CursoPerception/Proyecto1/Ros2
source install/setup.bash
ros2 run monitor_package monitor_subscriber
```

Se abre una ventana con dos gráficos que se actualizan en vivo — posición
arriba, velocidad abajo, con el eje de tiempo compartido — y en consola sale la
posición actual muestra a muestra.

**Ahora girá el eje del encoder**: un rato en un sentido, pausa, un rato en el
otro. Con 1 Hz de publicación, 60-90 segundos dan una curva presentable.

Para terminar, **cerrá la ventana del gráfico**: eso dispara el cierre ordenado
(primero frena ROS, después cierra el CSV). `Ctrl-C` también sirve.

---




