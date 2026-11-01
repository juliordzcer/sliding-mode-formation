#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Commands the formation: take off, leader trajectory, followers and landing.

Subscribes to /state, published by the control panel or by this node with
auto_sequence, and to the poses from Gazebo. It also publishes /formation_error,
the distance of each follower to its place in the formation.
"""
import csv
import math
import os

from comrob2023_formation import INERTIA, MASS, WORLD
from comrob2023_formation import leader_trajectory, make_scenario
from geometry_msgs.msg import Pose, Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import Bool, Float64MultiArray, Int32

IDLE, TAKING_OFF, AUTOMATIC, LANDING, STOPPED = range(5)
PHYSICS_STEP_SEC = 0.002
SETTLE_SEC = 20.0
MAX_TRAVEL_SPEED = 1.0


class FormationNode(Node):

    def __init__(self):
        super().__init__('formation_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.time_scale = self.declare_parameter('time_scale', 0.5).value
        self.disturbance = self.declare_parameter('disturbance', False).value
        self.auto_sequence = self.declare_parameter('auto_sequence', False).value
        self.duration = self.declare_parameter('duration', 50.0).value
        self.output = self.declare_parameter('output', '').value
        self.controller = self.declare_parameter('controller', 'ntsmc').value

        self.scenario = make_scenario(self.declare_parameter('formation', 'paper').value,
                                      self.declare_parameter('levels', 4).value,
                                      self.declare_parameter('text', 'COMROB').value,
                                      self.declare_parameter('spacing', 1.2).value)
        self.size = N = self.scenario.size
        self.trajectory = self.scenario.trajectory or leader_trajectory
        self.time_scale = self.scenario.time_scale or self.time_scale
        self.leader = self.scenario.leader

        self.state = IDLE
        self.t_state = 0.0
        self.pos = np.full((N, 3), np.nan)
        self.land_from = None
        self.log = []
        self.steps = 0
        self.last_step = None
        self.stage = 0
        self.saved = False

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Int32, '/state', state_qos)
        self.create_subscription(Int32, '/state', self.on_state, state_qos)
        for i in range(N):
            self.create_subscription(Pose, f'/model/espdrone{i + 1}/pose',
                                     lambda msg, i=i: self.on_pose(msg, i), 10)
        self.pos_pubs = [self.create_publisher(Pose, f'/espdrone{i + 1}/cmd_position', 10)
                         for i in range(N)]
        self.vel_pubs = [self.create_publisher(Twist, f'/espdrone{i + 1}/cmd_vel_world', 10)
                         for i in range(N)]
        self.enable_pubs = [self.create_publisher(Bool, f'/espdrone{i + 1}/motor_enable', 10)
                            for i in range(N)]
        self.wrench_pub = self.create_publisher(EntityWrench, f'/world/{WORLD}/wrench', 10)
        self.error_pub = self.create_publisher(Float64MultiArray, '/formation_error', 10)
        self.create_timer(0.02, self.step)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        if msg.data == self.state:
            return
        self.state = msg.data
        self.t_state = self.now_sec()
        if self.state == LANDING:
            self.land_from = self.pos.copy()
            self.land_goals = self.scenario.landing_goals(self.land_from)
        self.send_enable()

    def on_pose(self, msg, i):
        self.pos[i] = (msg.position.x, msg.position.y, msg.position.z)

    def send_enable(self):
        msg = Bool(data=self.state in (TAKING_OFF, AUTOMATIC, LANDING))
        for pub in self.enable_pubs:
            pub.publish(msg)

    def send_positions(self, targets):
        for pub, target in zip(self.pos_pubs, targets):
            msg = Pose()
            msg.position.x, msg.position.y, msg.position.z = (float(v) for v in target)
            msg.orientation.w = 1.0
            pub.publish(msg)

    def go_to(self, point):
        # On the way to a large formation, the velocity loop is smoother than following
        # a moving position.
        velocities = np.clip(point - self.pos, -MAX_TRAVEL_SPEED, MAX_TRAVEL_SPEED)
        for pub, velocity in zip(self.vel_pubs, velocities):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = (float(v) for v in velocity)
            pub.publish(msg)

    def send_disturbance(self, t, dt):
        # Disturbances of the paper, given there as accelerations. Each message acts
        # during one physics step, so it carries the impulse of the whole period.
        force = (dt / PHYSICS_STEP_SEC) * MASS * np.array([
            0.3 * math.sin(0.4 * t), 0.3 * math.cos(0.4 * t),
            -0.5 + 0.1 * math.sin(0.4 * t) - 0.1 * math.sin(0.6 * t) + 0.1 * math.cos(0.8 * t)])
        torque = (dt / PHYSICS_STEP_SEC) * INERTIA * np.array([
            -0.5 + 0.2 * math.sin(0.4 * t) - 0.2 * math.sin(0.6 * t) + 0.2 * math.cos(0.6 * t),
            -0.5 + 0.2 * math.cos(0.8 * t) - 0.2 * math.sin(0.4 * t) + 0.2 * math.cos(0.8 * t),
            -0.5 + 0.2 * math.cos(0.8 * t) - 0.2 * math.sin(0.6 * t) + 0.2 * math.cos(0.8 * t)])
        for i in range(self.size):
            msg = EntityWrench()
            msg.entity.name = f'espdrone{i + 1}::body'
            msg.entity.type = Entity.LINK
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(v) for v in force)
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (float(v) for v in torque)
            self.wrench_pub.publish(msg)

    def publish_state(self, state):
        self.state_pub.publish(Int32(data=state))

    def run_sequence(self, now):
        ready = not np.isnan(self.pos).any()
        if self.stage == 0 and ready and now > 2.0:
            self.publish_state(TAKING_OFF)
            self.stage = 1
        elif self.stage == 1 and self.state == TAKING_OFF and now - self.t_state > self.scenario.takeoff_sec + 3.0:
            self.publish_state(AUTOMATIC)
            self.stage = 2
        elif self.stage == 2 and self.state == AUTOMATIC and now - self.t_state > self.duration:
            self.save_log()
            self.publish_state(LANDING)
            self.stage = 3

    def step(self):
        now = self.now_sec()
        if now <= 0.0:
            return
        dt = now - self.last_step if self.last_step is not None else 0.0
        self.last_step = now
        if dt <= 0.0:
            return
        if self.auto_sequence:
            self.run_sequence(now)
        # An enable message sent just after Gazebo starts can be lost, and the
        # motors would never start. Repeat it while flying.
        self.steps += 1
        if self.steps % 10 == 0:
            self.send_enable()
        elapsed = now - self.t_state
        scenario = self.scenario
        if self.state == TAKING_OFF:
            point = scenario.takeoff_point(elapsed)
            if 0.0 < scenario.climb_sec <= elapsed:
                self.go_to(point)
            else:
                self.send_positions(point)
        elif self.state == AUTOMATIC and not np.isnan(self.pos).any():
            target = self.trajectory(elapsed * self.time_scale)
            targets = self.pos[self.leader] + scenario.offsets
            targets[self.leader] = target
            self.send_positions(targets)
            if self.disturbance:
                self.send_disturbance(elapsed, dt)
            errors = self.pos - self.pos[self.leader] - scenario.offsets
            errors[self.leader] = self.pos[self.leader] - target
            self.log.append((elapsed, *self.pos.ravel(), *errors.ravel()))
            self.error_pub.publish(Float64MultiArray(
                data=[elapsed] + np.linalg.norm(errors[:self.leader], axis=1).tolist()))
        elif self.state == LANDING and self.land_from is not None:
            point = scenario.landing_point(elapsed, self.land_from, self.land_goals)
            if scenario.climb_sec > 0.0 and elapsed < scenario.travel_sec:
                self.go_to(point)
            else:
                self.send_positions(point)
            if elapsed >= scenario.takeoff_sec:
                self.publish_state(IDLE)

    def save_log(self):
        if not self.output or self.saved or not self.log:
            return
        self.saved = True
        os.makedirs(self.output, exist_ok=True)
        log = np.array(self.log)
        N = self.size
        names = [f'{c}{i + 1}' for i in range(N) for c in 'xyz']
        with open(os.path.join(self.output, f'{self.controller}.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t'] + names + [f'e_{n}' for n in names])
            writer.writerows(np.round(log, 4).tolist())
        errors = log[log[:, 0] >= SETTLE_SEC, 1 + 3 * N:].reshape(-1, N, 3)
        rms = np.sqrt((np.delete(errors, self.leader, axis=1) ** 2).mean(axis=(0, 1)))
        self.get_logger().info(
            f'{self.controller}: follower RMS x,y,z = {np.round(rms, 4).tolist()} m')


def main():
    rclpy.init()
    node = FormationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
