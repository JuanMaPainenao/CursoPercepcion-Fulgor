import csv
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Monitor(Node):
    _MAX_PENDING = 100

    def __init__(self):
        super().__init__("monitor")

        self.declare_parameter("csv_path", "encoder_log.csv")
        self.csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        # Ultimo par ya emparejado
        self.last_pos = 0
        self.last_rpm = 0.0

        # Colas de emparejamiento (una por topico)
        self.pending_pos = deque(maxlen=self._MAX_PENDING)
        self.pending_vel = deque(maxlen=self._MAX_PENDING)

        self.t0 = self.get_clock().now()
        self.hist_t = []
        self.hist_pos = []
        self.hist_rpm = []
        self.lock = threading.Lock()

        # Archivo de historial con timestamp
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "t_rel_s", "position_tics", "velocity_rpm"])
        self.get_logger().info(f"Registrando historial en: {self.csv_path}")

        self.create_subscription(Int32, "encoder/position", self.cb_pos, 10)
        self.create_subscription(Float32, "encoder/velocity", self.cb_vel, 10)

    def _t_rel(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def cb_pos(self, msg: Int32):
        t = self._t_rel()
        with self.lock:
            self.pending_pos.append((t, msg.data))
            rows = self._drain()
        self._emit(rows)

    def cb_vel(self, msg: Float32):
        with self.lock:
            self.pending_vel.append(msg.data)
            rows = self._drain()
        self._emit(rows)

    def _drain(self):
        """Empareja el i-esimo de cada cola. Se llama con el lock tomado."""
        rows = []
        while self.pending_pos and self.pending_vel:
            t, pos = self.pending_pos.popleft()
            rpm = self.pending_vel.popleft()
            self.last_pos = pos
            self.last_rpm = rpm
            self.hist_t.append(t)
            self.hist_pos.append(pos)
            self.hist_rpm.append(rpm)
            rows.append((t, pos, rpm))
        return rows

    def _emit(self, rows):
        """Loguea y persiste. Va FUERA del lock a proposito: el hilo de
        matplotlib toma el mismo lock 5 veces por segundo y no queremos
        frenarlo con un write()+flush() contra el disco."""
        for t, pos, rpm in rows:
            ts = datetime.now().isoformat(timespec="milliseconds")
            self.writer.writerow([ts, f"{t:.3f}", pos, f"{rpm:.2f}"])
            self.get_logger().info(f"pos = {pos} tics | vel = {rpm:.2f} rpm")
        if rows:
            self.csv_file.flush()

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
        ax.grid(True, color="#e6e5e1", lw=0.8)
        ax.set_axisbelow(True)
        ax.tick_params(colors="#52514e")
        for spine in ax.spines.values():
            spine.set_color("#c9c8c2")

    ax_vel.axhline(0, color="#c9c8c2", lw=1)

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
        plt.show()  # bloquea hasta que cerras la ventana
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()            # hace que rclpy.spin() retorne
        spin_thread.join(timeout=2.0)
        node.close()                # ahora si, nadie mas escribe el CSV
        node.destroy_node()


if __name__ == "__main__":
    main()
