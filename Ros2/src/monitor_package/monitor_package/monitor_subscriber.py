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
    # Tope de las colas de emparejamiento. En regimen tienen 0 o 1 elemento;
    # esto es solo un cinturon de seguridad para que un desbalance no coma RAM.
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

        # Historial (tiempo relativo, posicion, velocidad) para las curvas.
        # Las tres listas se llenan SIEMPRE juntas, en _drain(), asi que el
        # indice i de las tres se refiere a la misma muestra: hist_t[i] sirve
        # de eje x para las dos curvas y no hay que interpolar nada.
        # El lock protege estas estructuras y las colas de arriba, que se tocan
        # desde dos hilos: los callbacks de ROS (hilo de spin) y el redibujado
        # de matplotlib (hilo principal).
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
        # El timestamp lo tomamos con la POSICION, que es la magnitud que
        # grafica la curva.
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
            # flush por muestra: si matamos el nodo con Ctrl-C no perdemos las
            # ultimas filas en el buffer de Python.
            self.csv_file.flush()

    def close(self):
        try:
            self.csv_file.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Monitor()

    # ROS spinea en un hilo aparte; matplotlib se queda en el hilo principal
    # (matplotlib exige correr en el hilo principal para mostrar la ventana).
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Dos paneles apilados, NO dos escalas en un mismo eje: tics y RPM son
    # magnitudes distintas y superponerlas en un eje doble haria que el cruce
    # de las curvas pareciera significar algo. sharex acopla el eje de tiempo,
    # asi que las dos curvas quedan alineadas y el zoom actua sobre las dos.
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    fig.suptitle("Encoder: historial de posicion y velocidad")

    ax_pos.set_ylabel("Posicion [tics]")
    ax_vel.set_ylabel("Velocidad [RPM]")
    ax_vel.set_xlabel("Tiempo [s]")

    # Una sola serie por panel: el ylabel ya la identifica, no hace falta
    # leyenda. Grilla y ejes en gris para que la curva sea lo que resalta.
    for ax in (ax_pos, ax_vel):
        ax.grid(True, color="#e6e5e1", lw=0.8)
        ax.set_axisbelow(True)
        ax.tick_params(colors="#52514e")
        for spine in ax.spines.values():
            spine.set_color("#c9c8c2")

    # Cero de referencia: la velocidad tiene signo, y la linea separa de un
    # vistazo un sentido de giro del otro (y marca el reposo).
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
            # Reescalar los dos: comparten x, pero cada uno tiene su propio y.
            for ax in (ax_pos, ax_vel):
                ax.relim()
                ax.autoscale_view()
        return (line_pos, line_vel)

    # Hay que guardar la referencia: si FuncAnimation se recolecta, la
    # animacion se frena y el grafico queda congelado.
    _ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)

    try:
        plt.show()  # bloquea hasta que cerras la ventana
    except KeyboardInterrupt:
        pass
    finally:
        # ORDEN IMPORTANTE: primero frenamos ROS y esperamos a que el hilo de
        # spin termine; recien ahi cerramos el CSV. Al reves, un callback
        # todavia en vuelo escribiria sobre un archivo ya cerrado y saltaria
        # "ValueError: I/O operation on closed file" en el hilo de spin.
        rclpy.shutdown()            # hace que rclpy.spin() retorne
        spin_thread.join(timeout=2.0)
        node.close()                # ahora si, nadie mas escribe el CSV
        node.destroy_node()


if __name__ == "__main__":
    main()
