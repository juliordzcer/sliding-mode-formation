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

## What you need

- A computer with **Ubuntu 24.04**.
- About 5 GB of free disk space. No special GPU is needed, because the drones have no cameras here.

You do not need to know ROS or Python to run it, and ROS is not used. Copy each block into a terminal (`Ctrl+Alt+T` opens one) and press Enter.

## Installation

**1. Download the repository**

```bash
sudo apt update && sudo apt install -y git
cd ~
git clone https://github.com/juliordzcer/comrob2023-sliding-mode-formation.git
cd comrob2023-sliding-mode-formation
```

**2. Install Gazebo and the other dependencies**

This asks for your password and takes a few minutes.

```bash
./install_deps.sh
```

**3. Build the controllers**

```bash
cmake -S . -B build
cmake --build build -j4
```

It should end with `Built target` lines and no errors.

## Running the simulation

From the repository folder, choose a controller:

```bash
python3 scripts/fly_formation.py --controller ntsmc
```

Gazebo and the control panel open after a few seconds. In the Gazebo window, press the view button or scroll to find the drones. Then:

1. Press **Take Off** and wait until the drones hover at 1 m, about 8 s.
2. Press **Automatic Mode**. The leader starts the trajectory and the panel shows the formation error.
3. Press **Land** to finish.

**Emergency Stop** turns the motors off, so the drones fall. Close the panel window to end the simulation.

### Options

| Option | What it does |
|---|---|
| `--controller` | `pid`, `tc`, `sta`, `stsmc` or `ntsmc` |
| `--headless` | Run Gazebo without its window |
| `--disturbance` | Apply the disturbance forces and torques of the paper |
| `--obs-pos`, `--obs-att` | Disturbance bounds of the observers; `0` turns one off |
| `--time-scale` | Speed of the trajectory. `1.0` is the speed of the paper, and the default is `0.5` |
| `--auto` | Fly without the panel: take off, follow the trajectory, save the data and exit |
| `--duration` | Seconds of automatic mode with `--auto` |
| `--output` | Folder for the flight data with `--auto` |

## Reproducing the results

This flies all the controllers 3 times, with and without observer, and takes about 10 minutes:

```bash
python3 scripts/compare_controllers.py --output results --disturbance --repeat 3
```

The flights are saved in `results/`, one folder per repetition, with the summary in `results/summary.csv`.

## Results

Flights with the disturbance forces and torques of the paper, at half the speed of the paper, with and without the disturbance observer. The errors are the RMS of the followers after the first 20 s, averaged over 3 flights. The peak is the largest error of a follower on any axis in those flights.

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

- **TC is not reliable with this drone.** Even with tuned gains, some flights lose a drone: in the table, 2 of the 3 flights with observer. STA stays in the air but with larger errors. Do not use their results to rank them against the others.
- **Speed.** At the speed of the paper (`--time-scale 1.0`), the controllers were not tuned and the tracking error is much larger.
- **Windows.** The panel was tested on a virtual display, and flights were tested with Gazebo in headless mode.

## Common problems

**`Gazebo did not start. Check that the plugins were built.`**: run the two commands of step 3 of the installation, from the repository folder.

**`No module named 'gz'`**: run `./install_deps.sh` again.

**The drones stay on the ground**: close everything, check that no earlier simulation is still running with `pgrep -a ruby`, and start again.

## Repository layout

| Folder | Content |
|---|---|
| `plugins` | Motor model, position controller and attitude controllers for Gazebo |
| `models/espdrone` | ESP-drone model |
| `worlds` | Gazebo world |
| `scripts` | Flight, control panel and comparison |

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
