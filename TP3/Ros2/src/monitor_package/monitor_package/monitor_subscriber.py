import csv
import math
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")

        self.declare_parameter("csv_path", "angulos_log.csv")
        self.csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        # Ultimos valores conocidos, en grados (0..360).
        self.last_pot_deg = 0.0
        self.last_as_deg = 0.0
        self.lock = threading.Lock()

        self.t0 = self.get_clock().now()

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "t_rel_s", "pot_deg", "as5600_deg"])
        self.get_logger().info(f"Registrando historial en: {self.csv_path}")

        # Pote: 0..100 %  -> grados en el callback. AS5600: ya viene en grados.
        self.create_subscription(Float32, "pot/position", self.cb_pot, 10)
        self.create_subscription(Float32, "as5600/angle", self.cb_as5600, 10)

    def cb_pot(self, msg: Float32):
        # 0..100 %  ->  0..360 grados (mismo mapeo que el sensor_bridge).
        with self.lock:
            self.last_pot_deg = msg.data * 3.6

    def cb_as5600(self, msg: Float32):
        # Usamos el AS5600 como "reloj" del registro: cada fila lleva ambos
        # angulos tomados en el mismo instante.
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        with self.lock:
            self.last_as_deg = msg.data
            pot = self.last_pot_deg
            aded = self.last_as_deg

        ts = datetime.now().isoformat(timespec="milliseconds")
        self.writer.writerow([ts, f"{t:.3f}", f"{pot:.1f}", f"{aded:.1f}"])
        self.csv_file.flush()
        self.get_logger().info(f"pot = {pot:.1f} deg | as5600 = {aded:.1f} deg")

    def close(self):
        try:
            self.csv_file.close()
        except Exception:  # noqa: BLE001
            pass


def make_dial(ax, color):
    """Configura un eje polar como dial: 0 arriba, sentido horario, radio mudo."""
    ax.set_theta_zero_location("N")     # 0 grados arriba
    ax.set_theta_direction(-1)          # sentido horario (poner 1 para antihorario)
    ax.set_ylim(0, 1)
    ax.set_yticklabels([])              # el radio no codifica nada
    ax.set_thetagrids(range(0, 360, 30))
    ax.grid(True, alpha=0.3)

    needle, = ax.plot([0, 0], [0, 1], lw=3, color=color)
    tip, = ax.plot([0], [1], marker="o", markersize=10, color=color)
    return needle, tip


def main(args=None):
    rclpy.init(args=args)
    node = Monitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_pot, ax_as) = plt.subplots(
        1, 2, figsize=(11, 6), subplot_kw={"projection": "polar"})
    fig.suptitle("Angulos en vivo: potenciometro vs encoder magnetico (AS5600)")

    needle_pot, tip_pot = make_dial(ax_pot, "#2a78d6")
    needle_as, tip_as = make_dial(ax_as, "#eb6834")

    def update(_frame):
        with node.lock:
            pot_deg = node.last_pot_deg
            as_deg = node.last_as_deg

        th_pot = math.radians(pot_deg)
        th_as = math.radians(as_deg)

        needle_pot.set_data([0, th_pot], [0, 1])
        tip_pot.set_data([th_pot], [1])
        ax_pot.set_title(f"Potenciometro\n{pot_deg:5.1f}\u00b0", pad=20)

        needle_as.set_data([0, th_as], [0, 1])
        tip_as.set_data([th_as], [1])
        ax_as.set_title(f"AS5600\n{as_deg:5.1f}\u00b0", pad=20)

        return (needle_pot, tip_pot, needle_as, tip_as)

    _ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.close()
        node.destroy_node()


if __name__ == "__main__":
    main()