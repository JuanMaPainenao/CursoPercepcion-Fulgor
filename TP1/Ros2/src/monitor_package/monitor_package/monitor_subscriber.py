import csv
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")

        self.declare_parameter("csv_path", "encoder_log.csv")
        self.csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        self.last_pos = 0
        self.last_rpm = 0.0

        self.t0 = self.get_clock().now()
        self.hist_t = []
        self.hist_pos = []
        self.hist_rpm = []
        self.lock = threading.Lock()

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "t_rel_s", "position_tics", "velocity_rpm"])
        self.get_logger().info(f"Registrando historial en: {self.csv_path}")

        self.create_subscription(Int32, "encoder/position", self.cb_pos, 10)
        self.create_subscription(Float32, "encoder/velocity", self.cb_vel, 10)

    def cb_pos(self, msg: Int32):
        self.last_pos = msg.data

    def cb_vel(self, msg: Float32):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        pos = self.last_pos
        rpm = msg.data

        with self.lock:
            self.hist_t.append(t)
            self.hist_pos.append(pos)
            self.hist_rpm.append(rpm)

        ts = datetime.now().isoformat(timespec="milliseconds")
        self.writer.writerow([ts, f"{t:.3f}", pos, f"{rpm:.2f}"])
        self.csv_file.flush()
        self.get_logger().info(f"pos = {pos} tics | vel = {rpm:.2f} rpm")

    def close(self):
        try:
            self.csv_file.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Monitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    fig.suptitle("Encoder: historial de posicion y velocidad")
    ax_pos.set_ylabel("Posicion [tics]")
    ax_vel.set_ylabel("Velocidad [RPM]")
    ax_vel.set_xlabel("Tiempo [s]")
    for ax in (ax_pos, ax_vel):
        ax.grid(True)

    (line_pos,) = ax_pos.plot([], [], lw=2, color="#2a78d6")
    (line_vel,) = ax_vel.plot([], [], lw=2, color="#eb6834")

    def update(_frame):
        with node.lock:
            t = list(node.hist_t)
            p = list(node.hist_pos)
            v = list(node.hist_rpm)
        if t:
            line_pos.set_data(t, p)
            line_vel.set_data(t, v)
            for ax in (ax_pos, ax_vel):
                ax.relim()
                ax.autoscale_view()
        return (line_pos, line_vel)

    _ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)

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