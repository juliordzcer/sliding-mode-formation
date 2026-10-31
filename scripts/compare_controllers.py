#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Fly the formation with every controller, with and without observer.

Flights run at the same time, each with its own Gazebo partition and discovery
ports, because simulations that share them lose commands.

Usage:
    python3 scripts/compare_controllers.py --output results --disturbance --repeat 3
"""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

N = 5
LEADER = 4
SETTLE_SEC = 20.0
CONTROLLERS = ('pid', 'tc', 'sta', 'stsmc', 'ntsmc')
FLY = str(Path(__file__).resolve().parent / 'fly_formation.py')


def summarize(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    errors = data[data[:, 0] >= SETTLE_SEC, 1 + 3 * N:].reshape(-1, N, 3)
    followers = np.delete(errors, LEADER, axis=1)
    rms = np.sqrt((followers ** 2).mean(axis=(0, 1)))
    return (*rms, float(np.abs(followers).max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results')
    parser.add_argument('--disturbance', action='store_true')
    parser.add_argument('--duration', type=float, default=50.0)
    parser.add_argument('--time-scale', type=float, default=0.5)
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()

    pending = [(c, obs, k) for c in CONTROLLERS for obs in (True, False)
               for k in range(args.repeat)]
    running, rows = [], []
    slot_free = list(range(args.jobs))
    while pending or running:
        while pending and slot_free:
            controller, observer, k = pending.pop(0)
            slot = slot_free.pop()
            out = os.path.join(args.output, f"{'obs' if observer else 'noobs'}_{k}")
            cmd = [sys.executable, FLY, '--controller', controller, '--auto', '--headless',
                   '--duration', str(args.duration), '--time-scale', str(args.time_scale),
                   '--output', out]
            if args.disturbance:
                cmd.append('--disturbance')
            if not observer:
                cmd += ['--obs-pos', '0', '--obs-att', '0']
            env = dict(os.environ, GZ_PARTITION=f'comrob{slot}', GZ_IP='127.0.0.1',
                       GZ_DISCOVERY_MSG_PORT=str(23000 + 10 * slot),
                       GZ_DISCOVERY_SRV_PORT=str(23001 + 10 * slot))
            proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
            running.append((proc, slot, controller, observer, out))
        time.sleep(1.0)
        for item in list(running):
            proc, slot, controller, observer, out = item
            if proc.poll() is None:
                continue
            running.remove(item)
            slot_free.append(slot)
            path = os.path.join(out, f'{controller}.csv')
            if os.path.exists(path):
                rows.append((controller, observer, *summarize(path)))
            else:
                print(f'{controller}: a flight failed')

    with open(os.path.join(args.output, 'summary.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['controller', 'observer', 'follower_x', 'follower_y', 'follower_z', 'peak'])
        writer.writerows([r[0], r[1], *[round(v, 4) for v in r[2:]]] for r in rows)
    print('RMS error of the followers [m] after the first 20 s, mean of the flights')
    print(f"{'controller':>11s}{'observer':>10s}{'x':>9s}{'y':>9s}{'z':>9s}{'worst peak':>12s}")
    for controller in CONTROLLERS:
        for observer in (True, False):
            sel = np.array([r[2:] for r in rows if r[0] == controller and r[1] == observer])
            if len(sel):
                print(f"{controller:>11s}{'yes' if observer else 'no':>10s}"
                      f'{sel[:, 0].mean():9.3f}{sel[:, 1].mean():9.3f}{sel[:, 2].mean():9.3f}'
                      f'{sel[:, 3].max():12.2f}')


if __name__ == '__main__':
    main()
