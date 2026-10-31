#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Fly the formation with every attitude controller and print the comparison.

Usage:
    python3 scripts/compare_controllers.py --output results --disturbance
"""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys

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
    return (*np.sqrt((followers ** 2).mean(axis=(0, 1))),
            *np.sqrt((errors[:, LEADER] ** 2).mean(axis=0)),
            float(np.abs(followers).max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results')
    parser.add_argument('--disturbance', action='store_true')
    parser.add_argument('--duration', type=float, default=50.0)
    parser.add_argument('--time-scale', type=float, default=0.5)
    args = parser.parse_args()
    rows = []
    for controller in CONTROLLERS:
        cmd = [sys.executable, FLY, '--controller', controller, '--auto', '--headless',
               '--duration', str(args.duration), '--time-scale', str(args.time_scale),
               '--output', args.output]
        if args.disturbance:
            cmd.append('--disturbance')
        env = dict(os.environ, GZ_PARTITION=f'comrob_{controller}', GZ_IP='127.0.0.1')
        if subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL).returncode != 0:
            print(f'{controller}: flight failed')
            continue
        rows.append((controller, *summarize(os.path.join(args.output, f'{controller}.csv'))))
    header = ['controller', 'follower_x', 'follower_y', 'follower_z',
              'leader_x', 'leader_y', 'leader_z', 'follower_peak']
    with open(os.path.join(args.output, 'summary.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows([r[0], *[round(v, 4) for v in r[1:]]] for r in rows)
    print('RMS error [m] after the first 20 s')
    print(''.join(f'{h:>14s}' for h in header))
    for row in rows:
        print(f'{row[0]:>14s}' + ''.join(f'{v:14.4f}' for v in row[1:]))


if __name__ == '__main__':
    main()
