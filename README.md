# Sliding Mode Formation Control of Quadrotors: Gazebo Simulation

Gazebo simulation based on the paper *Diseño de Control de Formación basado en Modos Deslizantes para un Grupo de Quad-Rotors* (COMRob 2023). Five ESP-drones fly in a leader-follower formation. Drone R5 is the leader and follows a trajectory, and the other four keep a fixed position relative to it. The attitude of the drones can be controlled by five different controllers, so they can be compared on the same flight:

| Name | Controller |
|---|---|
| `pid` | PID |
| `tc` | Continuous Twisting |
| `sta` | Super-Twisting Algorithm |
| `stsmc` | Continuous Singular Terminal Sliding Mode |
| `ntsmc` | Continuous Non-Singular Terminal Sliding Mode |

![Trajectory of the formation](media/trajectory.gif)

This repository is based on the simulation of the paper, but it uses another drone and other code. See [Differences from the paper](#differences-from-the-paper).

## This branch

This is the **ROS 2** version of the repository: a ROS node commands the drones and a bridge connects it to Gazebo. The `main` branch has the same simulation without ROS, with a simpler installation.

## What you need

- A computer with **Ubuntu 24.04**, because ROS 2 Jazzy needs it.
- About 10 GB of free disk space. No special GPU is needed, because the drones have no cameras here.

You do not need to know ROS or Python to run it. Copy each block into a terminal (`Ctrl+Alt+T` opens one) and press Enter.

## Installation

**1. Download this branch**

```bash
sudo apt update && sudo apt install -y git
mkdir -p ~/ws_comrob2023_ros/src
cd ~/ws_comrob2023_ros/src
git clone -b ros2 https://github.com/juliordzcer/comrob2023-sliding-mode-formation.git
```

**2. Install ROS 2, Gazebo and the other dependencies**

This asks for your password and takes a while.

```bash
cd ~/ws_comrob2023_ros/src/comrob2023-sliding-mode-formation
./install_deps.sh
```

**3. Build**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_comrob2023_ros
colcon build --symlink-install
```

It should end with `1 package finished`.

**4. Prepare each new terminal**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_comrob2023_ros/install/setup.bash
```

## Running the simulation

```bash
ros2 launch comrob2023_formation formation.launch.py controller:=ntsmc
```

Gazebo and the control panel open after a few seconds. In the Gazebo window, press the view button or scroll to find the drones. Then:

1. Press **Take Off** and wait until the drones hover at 1 m, about 8 s.
2. Press **Automatic Mode**. The leader starts the trajectory and the panel shows the formation error.
3. Press **Land** to finish.

**Emergency Stop** turns the motors off, so the drones fall. To close everything, press `Ctrl+C` in the terminal.

### Options

| Option | Values | Default | What it does |
|---|---|---|---|
| `controller` | `pid`, `tc`, `sta`, `stsmc`, `ntsmc` | `ntsmc` | Attitude controller |
| `headless` | `true`, `false` | `false` | Run Gazebo without its window |
| `run_gui` | `true`, `false` | `true` | Open the control panel |
| `disturbance` | `true`, `false` | `false` | Apply the disturbance forces and torques of the paper |
| `time_scale` | number | `0.5` | Speed of the trajectory. `1.0` is the speed of the paper |
| `auto_sequence` | `true`, `false` | `false` | Take off, fly and land without the panel |
| `duration` | seconds | `50.0` | Automatic mode with `auto_sequence` |
| `output` | folder | empty | Save the flight as a CSV with `auto_sequence` |

### Topics

| Topic | Type | Content |
|---|---|---|
| `/state` | `std_msgs/Int32` | 0 idle, 1 automatic, 2 take off, 3 land, 4 stop |
| `/model/espdroneN/pose` | `geometry_msgs/Pose` | Pose of drone N |
| `/espdroneN/cmd_position` | `geometry_msgs/Pose` | Position command of drone N |
| `/espdroneN/motor_enable` | `std_msgs/Bool` | Motors of drone N |
| `/formation_error` | `std_msgs/Float64MultiArray` | Time, then the error of each follower in meters |

## Reproducing the results

Run one flight for each controller, changing `controller` and the folder:

```bash
ros2 launch comrob2023_formation formation.launch.py controller:=ntsmc headless:=true run_gui:=false auto_sequence:=true disturbance:=true output:=results
```

The node prints the error when the automatic mode ends and saves `results/ntsmc.csv`. Press `Ctrl+C` once the drones have landed.

## Results

Flights of the `main` branch, with the disturbance forces and torques of the paper, at half the speed of the paper, with and without the disturbance observer. The errors are the RMS of the followers after the first 20 s, averaged over 3 flights. The peak is the largest error of a follower on any axis in those flights.

| Controller | Observer | x [m] | y [m] | z [m] | Worst peak [m] |
|---|---|---|---|---|---|
| PID | Yes | 0.190 | 0.154 | 0.022 | 0.62 |
| PID | No | 0.180 | 0.163 | 0.023 | 0.46 |
| STSMC | Yes | 0.189 | 0.208 | 0.022 | 0.38 |
| STSMC | No | 0.182 | 0.215 | 0.023 | 0.39 |
| NTSMC | Yes | 0.194 | 0.205 | 0.022 | 0.39 |
| NTSMC | No | 0.187 | 0.213 | 0.023 | 0.41 |
| STA | Yes | 0.195 | 0.203 | 0.023 | 0.38 |
| STA | No | 0.186 | 0.213 | 0.023 | 0.39 |
| TC | Yes | 0.192 | 0.205 | 0.022 | 0.39 |
| TC | No | 0.188 | 0.209 | 0.023 | 0.39 |

Most of the error comes from the leader, which lags behind its trajectory, and the followers inherit that lag, so the five controllers end up close. PID has the lowest error in y and also the largest peak. With every controller, the observer lowers the error in y a little and raises it in x a little, and it only changes the peak clearly with PID, where it is larger. These do not reproduce the ranking of the paper, where NTSMC has the lowest error.

These results are for this drone and these gains, so they do not replace the ones in the paper.

## Differences from the paper

| | Paper | This repository |
|---|---|---|
| Simulator | Numerical simulation | Gazebo, with propellers and motor dynamics |
| Drone | Crazyflie, 32 g | ESP-drone model, 59 g |
| Position control | Virtual controls of the paper | Cascade of position and velocity loops of the drone firmware |
| Disturbance observer | FT-SMO | The same observer, with its estimate applied through the attitude and thrust commands of the firmware cascade |
| Disturbances | Forces and torques | The same, scaled to the mass and inertia of this drone |
| Trajectory | As in the paper | Half the speed by default, because the paper's speed reaches 1.9 m/s |
| Height | `z_d` printed with `tan` | `tanh`, starting from hover at 1 m |
| Gains | Table of the paper | Tuned in flight for this drone |

The attitude laws of the plugins follow the equations of the paper.

## TODO

- Add larger formations with the same strategy, such as a pyramid with many drones or figures like letters, to show that it scales to many more agents.

## Known limitations

- **TC is not reliable with this drone.** Even with the tuned gains, 3 of 4 test flights lost a drone. Do not use its results to compare it with the other controllers.
- **Speed.** At the speed of the paper (`--time-scale 1.0`), the controllers were not tuned and the tracking error is much larger.
- **Windows.** The panel was tested on a virtual display, and flights were tested with Gazebo in headless mode.

## Common problems

**`Package 'comrob2023_formation' not found`**: run the two `source` commands of step 4 in that terminal.

**A change has no effect, or plugins are missing**: build again with the command of step 3.

**The drones stay on the ground**: close everything with `Ctrl+C`, check that no earlier simulation is still running with `pgrep -a ruby`, and start again.

## Repository layout

| Folder | Content |
|---|---|
| `plugins` | Motor model, position controller and attitude controllers for Gazebo |
| `models/espdrone` | ESP-drone model |
| `worlds` | Gazebo world |
| `nodes` | ROS nodes: formation commands and control panel |
| `launch` | Launch file with Gazebo and the bridge |
| `comrob2023_formation` | Formation, trajectory, gains and world generation |

## How to cite

If you use this code, please cite both the paper and this repository.

```bibtex
@inproceedings{rodriguez2023formacion,
  author    = {Rodríguez-Cervantes, J. C. and Dzul, A. and Ríos, H.},
  title     = {Diseño de Control de Formación basado en Modos Deslizantes para un Grupo de {Quad-Rotors}},
  booktitle = {XXV Congreso Mexicano de Robótica (COMRob)},
  address   = {Xalapa, México},
  year      = {2023}
}
```

```bibtex
@software{rodriguez2026comrob2023sim,
  author = {Rodriguez, Julio Cesar},
  title  = {COMRob 2023 Sliding Mode Formation Simulation},
  year   = {2026},
  url    = {https://github.com/juliordzcer/comrob2023-sliding-mode-formation}
}
```

## License

The code is under the [Apache License 2.0](LICENSE). Keep the [NOTICE](NOTICE) file in copies and derived works.

The ESP-drone model in `models/espdrone` was designed by Julio Cesar Rodriguez, who also measured its parameters on the real drone. It is under the [Creative Commons Attribution 4.0](models/espdrone/LICENSE) license, so credit him if you use or adapt it.

Contact: Julio Cesar Rodriguez, contact@juliordz.com
