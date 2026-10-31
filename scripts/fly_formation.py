#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Leader-follower formation of five ESP-drones in Gazebo (COMRob 2023).

Drone 5 follows the trajectory of the paper and the others keep the formation
vectors around it. The attitude controller of all drones is chosen with
--controller: pid, tc, sta, stsmc or ntsmc.

Usage:
    python3 scripts/fly_formation.py --controller sta           # with control panel
    python3 scripts/fly_formation.py --controller sta --auto --headless --output results/sta
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
import numpy as np

REPO = Path(__file__).resolve().parent.parent
N = 5
LEADER = 4
# Formation vectors C_51 ... C_54 of the paper, around the leader.
OFFSETS = np.array([[1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0], [0, 0, 0]], dtype=float)
HOVER_Z = 1.0
TAKEOFF_SEC = 5.0
LAND_SEC = 6.0
COMMAND_HZ = 50.0
MASS = 0.059
WORLD = 'espdrone_multi'
# The trajectory of the paper reaches 1.9 m/s, more than this drone model follows well.
TIME_SCALE = 0.5

ATTITUDE_PID = """
      <plugin name="gz::sim::systems::EspAttitudeController" filename="libEspAttitudeController.so">
        <robotNamespace>{ns}</robotNamespace>
        <comLinkName>body</comLinkName>
        <controlFrequency>500</controlFrequency>
        <thrustBase>26300</thrustBase>
        <rollRate_kp>17.8343</rollRate_kp> <rollRate_ki>0.0</rollRate_ki> <rollRate_kd>0.0</rollRate_kd>
        <pitchRate_kp>17.8343</pitchRate_kp> <pitchRate_ki>0.0</pitchRate_ki> <pitchRate_kd>0.0</pitchRate_kd>
        <yawRate_kp>8.9171</yawRate_kp> <yawRate_ki>0.0</yawRate_ki> <yawRate_kd>0.0</yawRate_kd>
        <roll_kp>6.5417</roll_kp> <roll_ki>0.0</roll_ki> <roll_kd>0.2641</roll_kd>
        <pitch_kp>6.5417</pitch_kp> <pitch_ki>0.0</pitch_ki> <pitch_kd>0.2641</pitch_kd>
        <yaw_kp>3.2708</yaw_kp> <yaw_ki>0.0</yaw_ki> <yaw_kd>0.1320</yaw_kd>
      </plugin>"""
ATTITUDE_SMC = """
      <plugin name="gz::sim::systems::{cls}" filename="lib{cls}.so">
        <robotNamespace>{ns}</robotNamespace>
        <comLinkName>body</comLinkName>
        <controlFrequency>500</controlFrequency>
        <thrustBase>26300</thrustBase>{gains}
      </plugin>"""
SMC_CLASS = {'sta': 'EspStaController', 'tc': 'EspTcController',
             'stsmc': 'EspStsmcController', 'ntsmc': 'EspNtsmcController'}
CONTROLLERS = ('pid', 'tc', 'sta', 'stsmc', 'ntsmc')
# Attitude gains tuned for this drone and formation.
SMC_GAINS = {
    'sta': {'k0_phi': 3.82977, 'k1_phi': 717.31471, 'k2_phi': 26.17799,
            'k0_theta': 3.82977, 'k1_theta': 717.31471, 'k2_theta': 26.17799,
            'nu_max': 15837.01764},
    'tc': {'k0_phi': 37.76499, 'k1_phi': 1036.52803, 'k2_phi': 1714.46675, 'k3_phi': 3110.79182,
           'k0_theta': 37.76499, 'k1_theta': 1036.52803, 'k2_theta': 1714.46675,
           'k3_theta': 3110.79182, 'nu_max': 2219.82598},
    'ntsmc': {'k0_phi': 0.10031, 'k1_phi': 5152.51682, 'k2_phi': 720.59224,
              'k0_theta': 0.10031, 'k1_theta': 5152.51682, 'k2_theta': 720.59224,
              'k0_psi': 0.10031, 'k1_psi': 2576.25841, 'k2_psi': 360.29612,
              'nu_max': 12647.19861},
}


def leader_trajectory(t):
    """Trajectory of the leader in the paper.

    The z_d printed in the paper has a typo: it is tanh, not tan. The drones
    start from hover here, so z_d does not go below the hover height.
    """
    radius = math.atan(15.0) + math.atan(t - 15.0)
    return np.array([radius * math.cos(math.pi * t / 5.0),
                     radius * math.sin(math.pi * t / 5.0),
                     max(HOVER_Z, 1.0 + math.tanh(t - 2.5))])


def make_world(controller, gains=None):
    """Return the path of a world file with five drones and the chosen attitude controller."""
    template = (REPO / 'worlds' / 'espdrone_multi.sdf').read_text()
    head, _, rest = template.partition('    <include>')
    drone = '    <include>' + rest.partition('</include>')[0] + '</include>\n'
    tail = rest.rpartition('</include>')[2]
    head = head.replace(
        '    <plugin filename="gz-sim-physics-system"',
        '    <plugin filename="gz-sim-apply-link-wrench-system"\n'
        '            name="gz::sim::systems::ApplyLinkWrench"/>\n'
        '    <plugin filename="gz-sim-physics-system"', 1)
    parts = [head]
    for i in range(N):
        ns = f'espdrone{i + 1}'
        block = drone.replace('espdrone1', ns)
        x, y, _ = OFFSETS[i]
        block = block.replace('<pose>0.4 0.4 0.3 0 0 0</pose>', f'<pose>{x} {y} 0.05 0 0 0</pose>')
        if controller == 'pid':
            attitude = ATTITUDE_PID.format(ns=ns)
        else:
            tags = {**SMC_GAINS.get(controller, {}), **(gains or {})}
            extra = ''.join(f'\n        <{k}>{v}</{k}>' for k, v in tags.items())
            attitude = ATTITUDE_SMC.format(cls=SMC_CLASS[controller], ns=ns, gains=extra)
        start = block.index('      <plugin name="gz::sim::systems::EspAttitudeController"')
        end = block.index('</plugin>', start) + len('</plugin>')
        block = block[:start].rstrip() + attitude + block[end:]
        # Allow the speed the trajectory of the paper needs.
        block = block.replace(
            '<thrustBase>26300</thrustBase>\n        <posX_kp>',
            '<thrustBase>26300</thrustBase>\n        <xyVelMax>4.0</xyVelMax>\n        <posX_kp>')
        parts.append(block)
    parts.append(tail)
    world = tempfile.NamedTemporaryFile('w', suffix=f'_{controller}.sdf', delete=False)
    world.write(''.join(parts))
    world.close()
    return world.name


class Formation:
    """Gazebo process, commands and telemetry of the formation."""

    IDLE, TAKING_OFF, AUTOMATIC, LANDING, STOPPED = range(5)
    LABELS = {IDLE: ('IDLE', '#555555'), TAKING_OFF: ('TAKING OFF', '#2471a3'),
              AUTOMATIC: ('AUTOMATIC', '#1e8449'), LANDING: ('LANDING', '#b9770e'),
              STOPPED: ('EMERGENCY STOP', '#c0392b')}

    def __init__(self, controller, headless, disturbance, time_scale=TIME_SCALE, gains=None):
        self.controller = controller
        self.time_scale = time_scale
        self.disturbance = disturbance
        self.lock = threading.Lock()
        self.state = self.IDLE
        self.pos = np.full((N, 3), np.nan)
        self.sim_time = 0.0
        self.t_state = 0.0
        self.z_from = HOVER_Z
        self.log = []
        self.world_file = make_world(controller, gains)
        env = dict(os.environ)
        env['GZ_SIM_RESOURCE_PATH'] = f"{REPO / 'models'}:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
            f"{REPO / 'build'}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}")
        cmd = ['gz', 'sim', '-r'] + (['-s'] if headless else []) + [self.world_file]
        self.proc = subprocess.Popen(cmd, env=env, start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.node = Node()
        self.pub_pos = [self.node.advertise(f'/espdrone{i + 1}/cmd_position', Pose) for i in range(N)]
        self.pub_enable = [self.node.advertise(f'/espdrone{i + 1}/motor_enable', Boolean)
                           for i in range(N)]
        self.pub_wrench = self.node.advertise(f'/world/{WORLD}/wrench/persistent', EntityWrench)
        self.pub_clear = self.node.advertise(f'/world/{WORLD}/wrench/clear', Entity)
        self.names = {f'espdrone{i + 1}': i for i in range(N)}
        self.node.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self._on_poses)
        self.running = True
        self.steps = 0

    def _on_poses(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            self.sim_time = stamp
            for pose in msg.pose:
                i = self.names.get(pose.name)
                if i is not None:
                    self.pos[i] = (pose.position.x, pose.position.y, pose.position.z)

    def wait_ready(self, timeout=60.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if not np.isnan(self.pos).any():
                    return True
            time.sleep(0.2)
        return False

    def set_state(self, state):
        with self.lock:
            if state == self.TAKING_OFF:
                self.home = self.pos.copy()
            if state == self.LANDING:
                self.land_from = self.pos.copy()
            self.state = state
            self.t_state = self.sim_time
        self._send_enable()

    def _send_enable(self):
        enable = Boolean()
        enable.data = self.state in (self.TAKING_OFF, self.AUTOMATIC, self.LANDING)
        for pub in self.pub_enable:
            pub.publish(enable)

    def _send(self, targets):
        for pub, target in zip(self.pub_pos, targets):
            msg = Pose()
            msg.position.x, msg.position.y, msg.position.z = (float(v) for v in target)
            msg.orientation.w = 1.0
            pub.publish(msg)

    def _send_disturbance(self, t):
        # Forces of the paper, given there as accelerations, scaled to this mass.
        force = MASS * np.array([0.3 * math.sin(0.4 * t), 0.3 * math.cos(0.4 * t),
                                 0.1 * math.sin(0.4 * t) - 0.1 * math.sin(0.6 * t)
                                 + 0.1 * math.cos(0.8 * t)])
        for i in range(N):
            msg = EntityWrench()
            msg.entity.name = f'espdrone{i + 1}::body'
            msg.entity.type = Entity.LINK
            self.pub_clear.publish(msg.entity)
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(v) for v in force)
            self.pub_wrench.publish(msg)

    def step(self):
        """Send the targets of the current state and record the flight."""
        with self.lock:
            state, now, pos, t_state = self.state, self.sim_time, self.pos.copy(), self.t_state
        dt = now - t_state
        # An enable message sent just after Gazebo starts can be lost, and the
        # motors would never start. Repeat it while flying.
        self.steps += 1
        if self.steps % 10 == 0:
            self._send_enable()
        if state == self.TAKING_OFF:
            s = min(1.0, dt / TAKEOFF_SEC)
            start = leader_trajectory(0.0) + OFFSETS
            start[:, 2] = HOVER_Z * s
            self._send(start)
        elif state == self.AUTOMATIC:
            target = leader_trajectory(dt * self.time_scale)
            targets = pos[LEADER] + OFFSETS
            targets[LEADER] = target
            self._send(targets)
            if self.disturbance:
                self._send_disturbance(dt)
            errors = pos - pos[LEADER] - OFFSETS
            errors[LEADER] = pos[LEADER] - target
            self.log.append((dt, *pos.ravel(), *errors.ravel()))
        elif state == self.LANDING:
            s = max(0.0, 1.0 - dt / LAND_SEC)
            targets = self.land_from.copy()
            targets[:, 2] *= s
            self._send(targets)
            if s == 0.0:
                self.set_state(self.IDLE)

    def run_commands(self):
        while self.running:
            self.step()
            time.sleep(1.0 / COMMAND_HZ)

    def close(self):
        self.running = False
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        os.unlink(self.world_file)


def wait_sim(formation, seconds):
    with formation.lock:
        end = formation.sim_time + seconds
    while True:
        with formation.lock:
            if formation.sim_time >= end:
                return
        if formation.proc.poll() is not None:
            raise RuntimeError('Gazebo stopped')
        time.sleep(0.05)


def save_log(formation, output, settle_sec=20.0):
    """Write the flight and return the RMS formation error after the trajectory settles."""
    os.makedirs(output, exist_ok=True)
    log = np.array(formation.log)
    names = [f'{c}{i + 1}' for i in range(N) for c in 'xyz']
    with open(os.path.join(output, f'{formation.controller}.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t'] + names + [f'e_{n}' for n in names])
        writer.writerows(np.round(log, 4).tolist())
    errors = log[log[:, 0] >= settle_sec, 1 + 3 * N:].reshape(-1, N, 3)
    followers = np.delete(errors, LEADER, axis=1)
    rms = np.sqrt((followers ** 2).mean(axis=(0, 1)))
    leader = np.sqrt((errors[:, LEADER] ** 2).mean(axis=0))
    return rms, leader


def run_auto(args):
    formation = Formation(args.controller, args.headless, args.disturbance, args.time_scale,
                          json.loads(args.gains))
    thread = threading.Thread(target=formation.run_commands, daemon=True)
    thread.start()
    try:
        if not formation.wait_ready():
            raise RuntimeError('Gazebo did not start. Check that the plugins were built.')
        wait_sim(formation, 1.0)
        formation.set_state(Formation.TAKING_OFF)
        wait_sim(formation, TAKEOFF_SEC + 3.0)
        formation.set_state(Formation.AUTOMATIC)
        wait_sim(formation, args.duration)
        rms, leader = save_log(formation, args.output)
        print(f'{args.controller}: follower RMS x,y,z = {np.round(rms, 4).tolist()} m, '
              f'leader RMS x,y,z = {np.round(leader, 4).tolist()} m', flush=True)
    finally:
        formation.close()


def run_panel(args):
    import tkinter as tk
    from tkinter import ttk

    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    formation = Formation(args.controller, args.headless, args.disturbance, args.time_scale,
                          json.loads(args.gains))
    threading.Thread(target=formation.run_commands, daemon=True).start()
    colors = ['#e74c3c', '#e67e22', '#2ecc71', '#3498db', '#9b59b6']

    root = tk.Tk()
    root.title(f'COMRob 2023 | {args.controller.upper()}')
    root.geometry('1200x800')
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', font=('Arial', 11), padding=8)
    style.configure('Stop.TButton', font=('Arial', 11, 'bold'), foreground='#c0392b')
    panel = ttk.Frame(root, padding=12, relief='groove')
    panel.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)
    ttk.Label(panel, text='COMRob 2023', font=('Arial', 16, 'bold')).pack()
    ttk.Label(panel, text=f'{args.controller.upper()} attitude control',
              font=('Arial', 11)).pack(pady=(2, 10))
    status = tk.Label(panel, text='STARTING', font=('Arial', 12, 'bold'))
    status.pack(pady=8)
    for text, state, btn_style in (
            ('Take Off', Formation.TAKING_OFF, 'TButton'),
            ('Automatic Mode', Formation.AUTOMATIC, 'TButton'),
            ('Land', Formation.LANDING, 'TButton'),
            ('Emergency Stop', Formation.STOPPED, 'Stop.TButton')):
        ttk.Button(panel, text=text, width=20, style=btn_style,
                   command=lambda s=state: formation.set_state(s)).pack(pady=6, padx=8)

    frame = ttk.Frame(root, padding=4)
    frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)
    fig = Figure(figsize=(9, 8), dpi=100, constrained_layout=True)
    grid = fig.add_gridspec(5, 1)
    ax_3d = fig.add_subplot(grid[:3, 0], projection='3d')
    ax_err = fig.add_subplot(grid[3:, 0])
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    trails = []

    def draw():
        with formation.lock:
            pos, state = formation.pos.copy(), formation.state
            log = list(formation.log[-1500:])
        label, color = Formation.LABELS[state]
        status.config(text=label, fg=color)
        if not np.isnan(pos).any():
            trails.append(pos)
            del trails[:-100]
            elev, azim = ax_3d.elev, ax_3d.azim
            ax_3d.cla()
            ax_3d.view_init(elev=elev, azim=azim)
            ax_3d.set_title('Formation', fontsize=11)
            ax_3d.set_xlabel('x [m]', fontsize=8)
            ax_3d.set_ylabel('y [m]', fontsize=8)
            ax_3d.set_zlabel('z [m]', fontsize=8)
            hist = np.array(trails)
            for i in range(N):
                ax_3d.plot(*hist[:, i].T, color=colors[i], linewidth=0.8, alpha=0.4)
                ax_3d.scatter(*pos[i], color=colors[i], s=70 if i == LEADER else 35,
                              marker='s' if i == LEADER else 'o')
            ax_3d.set_xlim(-5, 5)
            ax_3d.set_ylim(-5, 5)
            ax_3d.set_zlim(0, 3)
        ax_err.cla()
        ax_err.set_title('Formation error of the followers', fontsize=11)
        ax_err.set_xlabel('time in automatic mode [s]', fontsize=9)
        ax_err.set_ylabel('error [m]', fontsize=9)
        ax_err.grid(True, linewidth=0.5, linestyle='--', alpha=0.5)
        if log:
            data = np.array(log)
            errs = data[:, 1 + 3 * N:].reshape(-1, N, 3)
            for i in range(N - 1):
                ax_err.plot(data[:, 0], np.linalg.norm(errs[:, i], axis=1), color=colors[i],
                            linewidth=1.2, label=f'R{i + 1}')
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
        formation.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', choices=CONTROLLERS, default='sta')
    parser.add_argument('--headless', action='store_true', help='no Gazebo window')
    parser.add_argument('--disturbance', action='store_true', help='forces of the paper')
    parser.add_argument('--auto', action='store_true', help='fly without the control panel')
    parser.add_argument('--duration', type=float, default=50.0, help='automatic mode [s]')
    parser.add_argument('--time-scale', type=float, default=TIME_SCALE,
                        help='speed of the trajectory; 1.0 is the speed of the paper')
    parser.add_argument('--gains', default='{}', help='attitude gains as JSON')
    parser.add_argument('--output', default='results')
    args = parser.parse_args()
    if not shutil.which('gz'):
        raise SystemExit('Gazebo was not found. See the installation steps in README.md.')
    if args.auto:
        run_auto(args)
    else:
        run_panel(args)


if __name__ == '__main__':
    main()
