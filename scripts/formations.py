# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Formations around the leader: the ones of the papers, pyramids and letters.

A scenario has the offset of each drone from the leader, which is the last drone,
and its pad on the ground. A follower only needs the position of the leader and
its offset, so the same law works with any number of drones.
"""
from dataclasses import dataclass
import math
import typing

import numpy as np
from scipy.optimize import linear_sum_assignment

# Formation vectors C_51 ... C_54 of the paper, around the leader.
PAPER_OFFSETS = np.array([[1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0], [0, 0, 0]], dtype=float)

# Rows of a 5x7 font, top to bottom, with the most significant of 5 bits on the left.
FONT = {
    'A': (14, 17, 17, 31, 17, 17, 17), 'B': (30, 17, 17, 30, 17, 17, 30),
    'C': (14, 17, 16, 16, 16, 17, 14), 'D': (30, 17, 17, 17, 17, 17, 30),
    'E': (31, 16, 16, 30, 16, 16, 31), 'F': (31, 16, 16, 30, 16, 16, 16),
    'G': (14, 17, 16, 23, 17, 17, 15), 'H': (17, 17, 17, 31, 17, 17, 17),
    'I': (14, 4, 4, 4, 4, 4, 14), 'J': (7, 2, 2, 2, 2, 18, 12),
    'K': (17, 18, 20, 24, 20, 18, 17), 'L': (16, 16, 16, 16, 16, 16, 31),
    'M': (17, 27, 21, 21, 17, 17, 17), 'N': (17, 25, 21, 19, 17, 17, 17),
    'O': (14, 17, 17, 17, 17, 17, 14), 'P': (30, 17, 17, 30, 16, 16, 16),
    'Q': (14, 17, 17, 17, 21, 18, 13), 'R': (30, 17, 17, 30, 20, 18, 17),
    'S': (15, 16, 16, 14, 1, 1, 30), 'T': (31, 4, 4, 4, 4, 4, 4),
    'U': (17, 17, 17, 17, 17, 17, 14), 'V': (17, 17, 17, 17, 17, 10, 4),
    'W': (17, 17, 17, 21, 21, 27, 17), 'X': (17, 17, 10, 4, 10, 17, 17),
    'Y': (17, 17, 10, 4, 4, 4, 4), 'Z': (31, 1, 2, 4, 8, 16, 31),
}
PAD_SPACING = 1.5
CNCA_HOVER_Z = 0.5
HOP_HEIGHT = 2.5
CLIMB_SEC = 8.0
MAX_TAKEOFF_SPEED = 0.5


@dataclass
class Scenario:
    offsets: np.ndarray
    pads: np.ndarray
    start: np.ndarray
    climb_sec: float
    travel_sec: float
    trajectory: typing.Optional[typing.Callable] = None
    time_scale: typing.Optional[float] = None
    observer: bool = True

    @property
    def size(self):
        return len(self.offsets)

    @property
    def leader(self):
        return self.size - 1

    @property
    def takeoff_sec(self):
        return self.climb_sec + self.travel_sec

    def _above(self, pads):
        return pads + np.array([0.0, 0.0, HOP_HEIGHT if self.climb_sec > 0.0 else 0.0])

    def takeoff_point(self, elapsed):
        """Straight up first: a drone that moves sideways near the ground tips over."""
        lifted = self._above(self.pads)
        if elapsed < self.climb_sec:
            return self.pads + (elapsed / self.climb_sec) * (lifted - self.pads)
        s = min(1.0, (elapsed - self.climb_sec) / self.travel_sec)
        return lifted + s * (self.start - lifted)

    def landing_goals(self, pos):
        """Straight down with the formation of the paper. In a larger one, the pads are
        moved under the formation and given to the closest drones."""
        if self.climb_sec == 0.0:
            goals = pos.copy()
            goals[:, 2] = 0.0
            return goals
        goals = self.pads + np.array([*(pos[:, :2].mean(axis=0) - self.pads[:, :2].mean(axis=0)), 0.0])
        return assign(pos, goals)

    def landing_point(self, elapsed, origin, goals):
        """The take-off backwards: above the pads first, then straight down."""
        lifted = self._above(goals)
        if elapsed < self.travel_sec:
            return origin + (elapsed / self.travel_sec) * (lifted - origin)
        s = min(1.0, (elapsed - self.travel_sec) / max(self.climb_sec, 1e-6))
        return lifted + s * (goals - lifted)


def pyramid(levels, spacing):
    """Square layers of 1, 4, 9, ... drones, with the largest one at the bottom."""
    points = []
    for layer, side in enumerate(range(levels, 0, -1)):
        for i in range(side):
            for j in range(side):
                points.append(((i - (side - 1) / 2) * spacing, (j - (side - 1) / 2) * spacing,
                               layer * spacing))
    return np.array(points)


def letters(text, spacing):
    """Letters of a 5x7 font on the vertical plane y = 0."""
    points = []
    for k, letter in enumerate(text.upper()):
        if letter not in FONT:
            raise ValueError(f'No letter {letter!r} in the font: use A to Z.')
        for row, bits in enumerate(FONT[letter]):
            for col in range(5):
                if bits >> (4 - col) & 1:
                    points.append(((k * 6 + col) * spacing, 0.0, (6 - row) * spacing))
    points = np.array(points)
    points[:, 0] -= points[:, 0].mean()
    return points


def ground_pads(count, center):
    """Grid of pads on the ground around a point."""
    side = int(np.ceil(np.sqrt(count)))
    grid = [(i, j) for i in range(side) for j in range(side)]
    grid.sort(key=lambda p: (p[0] - (side - 1) / 2) ** 2 + (p[1] - (side - 1) / 2) ** 2)
    pads = np.zeros((count, 3))
    pads[:, :2] = (np.array(grid[:count]) - (side - 1) / 2) * PAD_SPACING + center[:2]
    return pads


def assign(sources, goals):
    """Goals reordered for the sources. With the smallest sum of squared distances,
    drones on straight lines that start and arrive together stay apart."""
    cost = ((sources[:, None] - goals[None]) ** 2).sum(axis=2)
    _, chosen = linear_sum_assignment(cost)
    return goals[chosen]


def paper_scenario(leader_start, takeoff_sec):
    start = PAPER_OFFSETS + leader_start
    pads = PAPER_OFFSETS.copy()
    start[:, :2] = pads[:, :2]
    return Scenario(PAPER_OFFSETS.copy(), pads, start, 0.0, takeoff_sec)


def shape_scenario(points, leader_start):
    """The leader is the drone of the lowest layer closest to the center, the last one."""
    low = points[:, 2].min()
    center = points[:, :2].mean(axis=0)
    candidates = np.flatnonzero(points[:, 2] == low)
    index = candidates[np.linalg.norm(points[candidates, :2] - center, axis=1).argmin()]
    order = [i for i in range(len(points)) if i != index] + [index]
    offsets = points[order] - points[index]
    start = offsets + leader_start
    pads = assign(start, ground_pads(len(start), start.mean(axis=0)))
    lifted = pads + np.array([0.0, 0.0, HOP_HEIGHT])
    travel = max(8.0, float(np.linalg.norm(start - lifted, axis=1).max()) / MAX_TAKEOFF_SPEED)
    return Scenario(offsets, pads, start, CLIMB_SEC, travel)


def cnca_trajectory(t):
    """Trajectory of the CNCA 2023 paper. The paper does not give the height h, so it is
    1 m here, and the drones start from hover, so z_d does not go below it."""
    radius = 0.2 * (math.atan(15.0) + math.atan(t - 15.0))
    return np.array([radius * math.cos(math.pi * t / 5.0) - 0.25,
                     radius * math.sin(math.pi * t / 5.0),
                     max(CNCA_HOVER_Z, 0.5 * (1.0 + math.tanh(t - 2.5)) + 0.05)])


def cnca_scenario(takeoff_sec):
    """The two drones of the CNCA 2023 paper, without the observer of COMRob 2023:
    R1 follows the leader R2 at C_21."""
    offsets = np.array([[0.5, 0.0, 0.1], [0.0, 0.0, 0.0]])
    start = offsets + cnca_trajectory(0.0)
    pads = start.copy()
    pads[:, 2] = 0.0
    return Scenario(offsets, pads, start, 0.0, takeoff_sec, cnca_trajectory, 1.0, False)
