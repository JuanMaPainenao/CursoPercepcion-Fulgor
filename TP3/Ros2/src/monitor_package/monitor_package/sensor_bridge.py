#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class SensorBridge(Node):
    """
    Puente hardware-in-the-loop:
      - Escucha los sensores que el ESP32 publica por micro-ROS/WiFi.
      - Convierte sus unidades fisicas (%, grados) a radianes (lo que usa el URDF).
      - Publica /joint_states para que robot_state_publisher arme el arbol de TF.
    """

    # Recorrido del joint del pot: 100 % -> 360 grados (2*pi). Coincide con el
    # 'upper' del joint_pot en el URDF.
    POT_JOINT_MAX = 2.0 * math.pi   # 360 grados en radianes

    def __init__(self):
        super().__init__("sensor_bridge")

        # Ultimos valores conocidos, ya en las unidades del joint (radianes).
        self.pos_pot = 0.0       # joint_pot    [rad]
        self.pos_as5600 = 0.0    # joint_as5600 [rad]

        # Suscripciones a los topics del ESP32 (QoS default: compatible con micro-ROS).
        self.create_subscription(Float32, "pot/position", self.cb_pot, 10)
        self.create_subscription(Float32, "as5600/angle", self.cb_as5600, 10)

        # Publicador que consume robot_state_publisher.
        self.pub = self.create_publisher(JointState, "joint_states", 10)

        # El ESP publica a 10 Hz; republicamos joint_states a 50 Hz para
        # mantener TF fresco y que RViz no "congele" la pose entre muestras.
        self.create_timer(1.0 / 50.0, self.publish_joint_states)

        self.get_logger().info(
            "sensor_bridge listo: pot/position + as5600/angle -> /joint_states")

    def cb_pot(self, msg: Float32):
        # 0..100 %  ->  0..2*pi rad (recorrido del joint revolute)
        pct = max(0.0, min(100.0, msg.data))
        self.pos_pot = (pct / 100.0) * self.POT_JOINT_MAX

    def cb_as5600(self, msg: Float32):
        # 0..360 grados  ->  radianes. Joint continuous: sin recorte.
        self.pos_as5600 = math.radians(msg.data)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint_pot", "joint_as5600"]        # DEBEN coincidir con el URDF
        js.position = [self.pos_pot, self.pos_as5600]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()