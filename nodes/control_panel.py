#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Control panel: take off, automatic, land, emergency stop and live plots.

ROS runs in a separate process so that the plots do not delay the messages.
"""
import multiprocessing as mp
import queue
import signal
import sys
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Pose
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402, F401
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Float64MultiArray, Int32  # noqa: E402

COLORS = ['#e74c3c', '#e67e22', '#2ecc71', '#3498db', '#9b59b6']
N = 5
IDLE, TAKING_OFF, AUTOMATIC, LANDING, STOPPED = range(5)
LABELS = {IDLE: ('IDLE', '#555555'), TAKING_OFF: ('TAKING OFF', '#2471a3'),
          AUTOMATIC: ('AUTOMATIC', '#1e8449'), LANDING: ('LANDING', '#b9770e'),
          STOPPED: ('EMERGENCY STOP', '#c0392b')}


class RosWorker(Node):

    def __init__(self, to_gui, to_ros):
        super().__init__('control_panel')
        self.to_gui, self.to_ros = to_gui, to_ros
        self.state = IDLE
        self.pos = np.full((N, 3), np.nan)
        self.errors = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Int32, '/state', qos)
        self.create_subscription(Int32, '/state', self.on_state, qos)
        for i in range(N):
            self.create_subscription(Pose, f'/model/espdrone{i + 1}/pose',
                                     lambda msg, i=i: self.on_pose(msg, i), 10)
        self.create_subscription(Float64MultiArray, '/formation_error', self.on_error, 10)
        self.create_timer(0.1, self.exchange)

    def on_state(self, msg):
        self.state = msg.data

    def on_pose(self, msg, i):
        self.pos[i] = (msg.position.x, msg.position.y, msg.position.z)

    def on_error(self, msg):
        self.errors.append(list(msg.data))

    def exchange(self):
        try:
            while True:
                self.pub.publish(Int32(data=self.to_ros.get_nowait()))
        except queue.Empty:
            pass
        try:
            self.to_gui.put_nowait((self.state, self.pos.copy(), self.errors[::5]))
        except queue.Full:
            return
        self.errors = []


def ros_main(to_gui, to_ros):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init()
    node = RosWorker(to_gui, to_ros)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    controller = argv[1] if len(argv) > 1 else 'ntsmc'
    global N
    N = int(argv[2]) if len(argv) > 2 else 5
    # The worker starts before Tk exists, so it inherits no window state.
    context = mp.get_context('fork')
    to_gui, to_ros = context.Queue(maxsize=50), context.Queue()
    worker = context.Process(target=ros_main, args=(to_gui, to_ros), daemon=True)
    worker.start()

    root = tk.Tk()
    root.title(f'COMRob 2023 | {controller.upper()}')
    root.geometry('1200x800')
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', font=('Arial', 11), padding=8)
    style.configure('Stop.TButton', font=('Arial', 11, 'bold'), foreground='#c0392b')
    panel = ttk.Frame(root, padding=12, relief='groove')
    panel.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)
    ttk.Label(panel, text='COMRob 2023', font=('Arial', 16, 'bold')).pack()
    ttk.Label(panel, text=f'{controller.upper()} attitude control',
              font=('Arial', 11)).pack(pady=(2, 10))
    status = tk.Label(panel, text='IDLE', font=('Arial', 12, 'bold'))
    status.pack(pady=8)
    for text, state, btn_style in (('Take Off', TAKING_OFF, 'TButton'),
                                   ('Automatic Mode', AUTOMATIC, 'TButton'),
                                   ('Land', LANDING, 'TButton'),
                                   ('Emergency Stop', STOPPED, 'Stop.TButton')):
        ttk.Button(panel, text=text, width=20, style=btn_style,
                   command=lambda s=state: to_ros.put(s)).pack(pady=6, padx=8)

    frame = ttk.Frame(root, padding=4)
    frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)
    fig = Figure(figsize=(9, 8), dpi=100, constrained_layout=True)
    grid = fig.add_gridspec(5, 1)
    ax_3d = fig.add_subplot(grid[:3, 0], projection='3d')
    ax_err = fig.add_subplot(grid[3:, 0])
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    view = {'state': IDLE, 'pos': np.full((N, 3), np.nan), 'trails': [], 'errors': []}

    def draw():
        try:
            while True:
                state, pos, errors = to_gui.get_nowait()
                view['state'], view['pos'] = state, pos
                if not np.isnan(pos).any():
                    view['trails'].append(pos)
                    del view['trails'][:-120]
                if errors and view['errors'] and errors[0][0] < view['errors'][-1][0]:
                    view['errors'].clear()
                view['errors'] += errors
        except queue.Empty:
            pass
        label, color = LABELS.get(view['state'], LABELS[IDLE])
        status.config(text=label, fg=color)
        if view['trails']:
            pos = view['pos']
            elev, azim = ax_3d.elev, ax_3d.azim
            ax_3d.cla()
            ax_3d.view_init(elev=elev, azim=azim)
            ax_3d.set_title('Formation', fontsize=11)
            ax_3d.set_xlabel('x [m]', fontsize=8)
            ax_3d.set_ylabel('y [m]', fontsize=8)
            ax_3d.set_zlabel('z [m]', fontsize=8)
            hist = np.array(view['trails'])
            if N <= len(COLORS):
                for i in range(N):
                    ax_3d.plot(*hist[:, i].T, color=COLORS[i], linewidth=0.8, alpha=0.4)
            ax_3d.scatter(*pos[:N - 1].T, color='#2471a3', s=20 if N > len(COLORS) else 35)
            ax_3d.scatter(*pos[N - 1], color='#c0392b', s=70, marker='s')
            half = max(5.0, float(np.nanmax(np.abs(pos[:, :2]))) + 1.0)
            ax_3d.set_xlim(-half, half)
            ax_3d.set_ylim(-half, half)
            ax_3d.set_zlim(0, max(3.0, float(np.nanmax(pos[:, 2])) + 0.5))
        ax_err.cla()
        ax_err.set_title('Formation error of the followers', fontsize=11)
        ax_err.set_xlabel('time in automatic mode [s]', fontsize=9)
        ax_err.set_ylabel('error [m]', fontsize=9)
        ax_err.grid(True, linewidth=0.5, linestyle='--', alpha=0.5)
        if view['errors']:
            data = np.array(view['errors'])
            if N <= len(COLORS):
                for i in range(N - 1):
                    ax_err.plot(data[:, 0], data[:, i + 1], color=COLORS[i], linewidth=1.2,
                                label=f'R{i + 1}')
            else:
                ax_err.plot(data[:, 0], data[:, 1:].mean(axis=1), color='#2471a3', linewidth=1.2,
                            label='mean')
                ax_err.plot(data[:, 0], data[:, 1:].max(axis=1), color='#c0392b', linewidth=1.2,
                            label='largest')
            ax_err.legend(loc='upper right', fontsize=8, ncol=4)
        canvas.draw_idle()
        root.after(250, draw)

    root.protocol('WM_DELETE_WINDOW', root.quit)
    signal.signal(signal.SIGINT, lambda *_: root.quit())

    def keep_alive():
        root.after(200, keep_alive)
    keep_alive()
    root.after(250, draw)
    try:
        root.mainloop()
    finally:
        worker.terminate()
        worker.join(timeout=2)


if __name__ == '__main__':
    main()
