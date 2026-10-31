# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Formation definition, trajectory and world generation (COMRob 2023)."""
import math
from pathlib import Path
import tempfile

import numpy as np

N = 5
LEADER = 4
# Formation vectors C_51 ... C_54 of the paper, around the leader.
OFFSETS = np.array([[1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0], [0, 0, 0]], dtype=float)
HOVER_Z = 1.0
TAKEOFF_SEC = 5.0
LAND_SEC = 6.0
COMMAND_HZ = 50.0
MASS = 0.063
INERTIA = np.array([1.594e-05, 1.4535e-05, 2.8468e-05])
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
POSE_PUBLISHER = """
      <plugin name="gz::sim::systems::PosePublisher" filename="gz-sim-pose-publisher-system">
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>100</update_frequency>
      </plugin>"""
SMC_CLASS = {'sta': 'EspStaController', 'tc': 'EspTcController',
             'stsmc': 'EspStsmcController', 'ntsmc': 'EspNtsmcController'}
CONTROLLERS = ('pid', 'tc', 'sta', 'stsmc', 'ntsmc')
# Attitude gains tuned for this drone and formation.
SMC_GAINS = {
    'tc': {'k0_phi': 6264.97819, 'k1_phi': 3758.98692, 'k2_phi': 127.93203, 'k3_phi': 308.70168, 'k0_theta': 6264.97819, 'k1_theta': 3758.98692, 'k2_theta': 127.93203, 'k3_theta': 308.70168, 'k0_psi': 920.30595, 'k1_psi': 552.18357, 'k2_psi': 18.79282, 'k3_psi': 45.34732, 'nu_max': 3035.80767, 'lpf_cutoff_hz': 187.29657, 'rate_lpf_cutoff_hz': 51.34855},
    'sta': {'k0_phi': 0.84696, 'k1_phi': 743.98088, 'k2_phi': 93080708.05085, 'k0_theta': 0.84696, 'k1_theta': 743.98088, 'k2_theta': 93080708.05085, 'k0_psi': 0.84696, 'k1_psi': 301.58201, 'k2_psi': 37731436.34382, 'nu_max': 771.03436, 'lpf_cutoff_hz': 87.57061, 'rate_lpf_cutoff_hz': 64.49251},
    'stsmc': {'k0_phi': 0.20036, 'k0_theta': 0.20036, 'k1_phi': 4744.60681, 'k1_theta': 4744.60681, 'k2_phi': 1108.52578, 'k2_theta': 1108.52578, 'nu_max': 5016.8184},
    'ntsmc': {'k0_phi': 0.27354, 'k1_phi': 8221.99787, 'k2_phi': 2055.93238, 'k0_theta': 0.27354, 'k1_theta': 8221.99787, 'k2_theta': 2055.93238, 'k0_psi': 0.10031, 'k1_psi': 2576.25841, 'k2_psi': 360.29612, 'nu_max': 4592.4458},
}
# Disturbance bounds of the observers per controller, position in m/s^2 and
# attitude in rad/s^2, tuned with the gains above. 0 turns an observer off.
OBSERVER_BOUNDS = {
    'pid': (1.1112, 0.0507),
    'tc': (0.5448, 0.1617),
    'sta': (1.1589, 4.9213),
    'stsmc': (2.5935, 0.6387),
    'ntsmc': (0.2759, 0.6503),
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


def make_world(template_path, controller, gains=None, obs_pos=None, obs_att=None):
    """Return the path of a world file with five drones and the chosen attitude controller."""
    obs_pos = OBSERVER_BOUNDS[controller][0] if obs_pos is None else obs_pos
    obs_att = OBSERVER_BOUNDS[controller][1] if obs_att is None else obs_att
    template = Path(template_path).read_text()
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
            attitude = ATTITUDE_PID.format(ns=ns).replace(
                '</plugin>', f'  <ftsmo_D_att>{obs_att}</ftsmo_D_att>\n      </plugin>')
        else:
            tags = {**SMC_GAINS.get(controller, {}), **(gains or {})}
            tags['ftsmo_D_att'] = obs_att
            extra = ''.join(f'\n        <{k}>{v}</{k}>' for k, v in tags.items())
            attitude = ATTITUDE_SMC.format(cls=SMC_CLASS[controller], ns=ns, gains=extra)
        start = block.index('      <plugin name="gz::sim::systems::EspAttitudeController"')
        end = block.index('</plugin>', start) + len('</plugin>')
        block = block[:start].rstrip() + attitude + block[end:]
        # Allow the speed the trajectory of the paper needs.
        block = block.replace(
            '<thrustBase>26300</thrustBase>\n        <posX_kp>',
            f'<thrustBase>26300</thrustBase>\n        <xyVelMax>4.0</xyVelMax>\n        '
            f'<ftsmo_D_pos>{obs_pos}</ftsmo_D_pos>\n        <posX_kp>')
        # Publishes the pose of each drone for the ROS bridge.
        block = block.replace('</include>', POSE_PUBLISHER + '\n    </include>')
        parts.append(block)
    parts.append(tail)
    world = tempfile.NamedTemporaryFile('w', suffix=f'_{controller}.sdf', delete=False)
    world.write(''.join(parts))
    world.close()
    return world.name
