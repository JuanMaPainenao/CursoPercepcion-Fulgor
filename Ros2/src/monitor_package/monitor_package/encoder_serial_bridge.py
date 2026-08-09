import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import serial


class EncoderSerialBridge(Node):
    """Lee lineas 'posicion,rpm' del ESP32 por serial y las publica en ROS2.

    Publica:
      /encoder/position  (std_msgs/Int32)   -> posicion en tics
      /encoder/velocity  (std_msgs/Float32) -> velocidad en RPM

    El firmware imprime una linea por ventana de medicion (1 Hz) con
    printf("%ld,%.2f\\n", pos, rpm). ESP-IDF traduce ese '\\n' a "\\r\\n",
    y por el mismo UART tambien salen el log del bootloader y los ESP_LOGI,
    asi que este nodo tiene que ser tolerante a lineas basura.
    """

    def __init__(self):
        super().__init__("encoder_serial_bridge")

        # Parametros (se pueden sobreescribir al lanzar el nodo)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.pos_pub = self.create_publisher(Int32, "encoder/position", 10)
        self.vel_pub = self.create_publisher(Float32, "encoder/velocity", 10)

        try:
            # timeout corto a proposito: readline() bloquea al executor de ROS
            # mientras espera el '\n'. Si el ESP32 alcanzo a mandar media linea
            # y se colgo, con timeout=1.0 congelabamos el nodo un segundo entero.
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir {port}: {e}")
            raise

        # Descarta lo que haya quedado en el buffer del SO. Ojo: abrir el puerto
        # activa DTR/RTS y en la mayoria de las devkits eso RESETEA el ESP32, asi
        # que el log de arranque igual va a llegar despues de esta linea. Lo
        # filtra el parser de abajo.
        self.ser.reset_input_buffer()
        self.get_logger().info(f"Puerto serie abierto: {port} @ {baud} baud")

        # Sondeamos el serial mas rapido que la llegada de datos (1 Hz)
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline()
        except serial.SerialException as e:
            # p.ej. desenchufaron el USB. Avisamos (con throttle para no
            # inundar el log 50 veces por segundo) pero no matamos el nodo.
            self.get_logger().error(
                f"Error leyendo el serial: {e}", throttle_duration_sec=5.0
            )
            return

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            return

        parts = line.split(",")
        if len(parts) != 2:
            return  # log del ESP32, linea partida al arrancar, o basura

        try:
            pos = int(parts[0])
            rpm = float(parts[1])
        except ValueError:
            return  # no era un dato valido, la descartamos

        # Invariante que sostiene todo el diseño: por CADA renglon del serial
        # publicamos exactamente UN mensaje en cada topico, siempre los dos.
        # Ese 1:1 es lo que le permite al monitor volver a emparejar posicion
        # y velocidad del lado del subscriber (ver monitor_subscriber._drain).
        # El orden entre los dos publish() NO importa: el executor de ROS no
        # entrega los callbacks en orden de llegada entre topicos distintos.
        self.pos_pub.publish(Int32(data=pos))
        self.vel_pub.publish(Float32(data=rpm))

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
