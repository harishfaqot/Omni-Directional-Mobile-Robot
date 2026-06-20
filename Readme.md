# RobotPadi — Swerve-Drive Paddy Field Robot (ROS 2)

This repository is a ROS 2 workspace `src/` folder containing two packages for **RobotPadi**, a 4‑wheel **swerve‑drive** field robot for paddy/rice‑field operations:

| Package | Type | Purpose |
|---|---|---|
| [`robotpadi_description`](#robotpadi_description) | `ament_python` | URDF/Xacro model, meshes, `ros2_control` + Gazebo bindings, RViz configs, Gazebo worlds & models, launch files |
| [`robotpadi_controller`](#robotpadi_controller) | `ament_python` | Swerve‑drive kinematics + a click‑to‑navigate node (`swerve_nav_goal`) driven from RViz's 2D Nav Goal, plus two standalone Tkinter teleop GUIs |

> Built and tested against **ROS 2 Jazzy** + **Gazebo Harmonic** (`ros_gz`, `gz_ros2_control`). Other ROS 2 / Gazebo combinations may work but are not verified by this README.

---

## Table of contents

- [Robot overview](#robot-overview)
- [Repository layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [⚠️ Required fix before first run](#️-required-fix-before-first-run)
- [Running the simulation](#running-the-simulation)
- [`robotpadi_description`](#robotpadi_description)
- [`robotpadi_controller`](#robotpadi_controller)
- [Topics & ROS interfaces](#topics--ros-interfaces)
- [Controllers](#controllers)
- [Driving the robot](#driving-the-robot)
- [Worlds](#worlds)
- [Regenerating the paddy field world](#regenerating-the-paddy-field-world)
- [Known issues / things to check before relying on this](#known-issues--things-to-check-before-relying-on-this)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Robot overview

RobotPadi is a 4‑module **swerve drive** (independent steer + drive per corner) platform with a sensor/equipment panel and an arm/boom mount on top of the chassis. The model was exported from a Fusion 360 assembly using the `fusion2urdf_ros2` exporter, then hand‑edited to add `ros2_control` and Gazebo bindings.

| Module | Steering joint | Wheel (drive) joint |
|---|---|---|
| Front‑right | `Revolute 1` | `Revolute 5` |
| Back‑right  | `Revolute 2` | `Revolute 6` |
| Front‑left  | `Revolute 3` | `Revolute 7` |
| Back‑left   | `Revolute 4` | `Revolute 8` |

- **Steering joints** (`Revolute 1‑4`): continuous, **position‑controlled**.
- **Wheel joints** (`Revolute 5‑8`): continuous, **velocity‑controlled**.
- Two fixed bodies: `panel_1` (top panel) and `arm_1` (boom/arm), both rigidly mounted to the chassis (`base_robot`).
- All visual/collision geometry is loaded from STL meshes (units in millimeters, scaled `0.001` in the URDF).
- No sensors (lidar/camera/IMU) are defined in the URDF — the RViz config includes a `LaserScan` display placeholder, but no laser plugin is wired up.

## Repository layout

```
src/
├── robotpadi_description/      # Robot model, sim worlds, launch files (see below)
└── robotpadi_controller/       # Swerve kinematics + nav/teleop nodes
    ├── robotpadi_controller/
    │   ├── swerve_nav_goal.py      # console_script: swerve_nav_goal (registered, used by robot.launch.py)
    │   ├── swerve_controller.py    # standalone node + Tkinter GUI with proper swerve kinematics (not a console_script)
    │   └── gui.py                  # earlier/simpler Tkinter teleop GUI — same speed/angle on all 4 wheels (not a console_script)
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

## Prerequisites

- **Ubuntu 24.04** (recommended for ROS 2 Jazzy)
- **ROS 2 Jazzy** (or a distro with `ros_gz` + `gz_ros2_control` support)
- **Gazebo Harmonic** (`gz-sim`)
- `python3-tk` (Tkinter) — required only if you plan to run `swerve_controller.py` or `gui.py` directly
- ROS 2 / system packages:
  ```bash
  sudo apt install \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-velocity-controllers \
    python3-tk
  ```

## Installation

```bash
# 1. Create / go to your workspace
mkdir -p ~/ros2_ws
cd ~/ros2_ws

# 2. Put the contents of this repo's src/ folder here, so you end up with:
#      ~/ros2_ws/src/robotpadi_description
#      ~/ros2_ws/src/robotpadi_controller

# 3. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build both packages
colcon build --packages-select robotpadi_description robotpadi_controller

# 5. Source the workspace
source install/setup.bash
```

> Both packages use `ament_python`. `robotpadi_description`'s `setup.py` copies `urdf/`, `meshes/`, `config/`, `launch/`, `worlds/`, and `models/` into `share/robotpadi_description/` at build time. `robotpadi_controller`'s `setup.py` only registers `swerve_nav_goal` as an installed console script (see [`robotpadi_controller`](#robotpadi_controller) below for the other two scripts, which are run differently).

## ⚠️ Required fix before first run

The default world used by `robot.launch.py` (`robotpadi_description/worlds/paddy_generated.world`, and also `farm.sdf`/`generated.world`) references the maize/mud‑box models with an **absolute path hardcoded to the original author's machine**:

```
/home/ubuntu/github/Omni-Directional-Mobile-Robot/src/robotpadi_description/models/...
```

This path almost certainly does **not** exist on your machine, so Gazebo will fail to load those models. Fix it before launching:

**Option A — Regenerate the world for your workspace (recommended)**
See [Regenerating the paddy field world](#regenerating-the-paddy-field-world) below.

**Option B — Quick find‑and‑replace**
```bash
cd ~/ros2_ws/src/robotpadi_description/worlds
sed -i "s|/home/ubuntu/github/Omni-Directional-Mobile-Robot/src/robotpadi_description|$(realpath ..)|g" \
  paddy_generated.world generated.world farm.sdf
```
This points the `<uri>` tags at your source tree's `models/` folder directly. If you rebuild with `colcon build`, repeat this on the **installed** copy under `install/robotpadi_description/share/robotpadi_description/worlds/`, or re‑run on the source and rebuild.

## Running the simulation

```bash
ros2 launch robotpadi_description robot.launch.py
```

This is the main entry point. It brings up Gazebo with the paddy field world, spawns the robot, starts `ros2_control`, the bridge, RViz, **and the `swerve_nav_goal` node from `robotpadi_controller`** (this is why both packages need to be built — see [Installation](#installation)).

What it does, in order:

1. Sets `GZ_SIM_RESOURCE_PATH` so Gazebo can resolve `model://` URIs and relative texture paths under `robotpadi_description`'s `models/` and `worlds/` folders.
2. Starts `robot_state_publisher` with the processed `robotpadi.xacro` description.
3. Launches Gazebo (`gz_sim.launch.py`) with `worlds/paddy_generated.world` (`-r` run‑on‑start, `-v 4` verbose, `--render-engine ogre`).
4. After **5 s**, spawns the robot (`ros_gz_sim create`) from `/robot_description` at pose `(x=-5.0, y=-5.0, z=0.35)`.
5. Starts the `ros_gz_bridge` parameter bridge (clock / odom / tf — see [Topics](#topics--ros-interfaces)).
6. After **8 s**, spawns the `joint_state_broadcaster` controller.
7. After **10 s**, spawns `steering_controller` and `wheel_controller`.
8. After **6 s**, opens RViz2 with `config/gazebo.rviz`.
9. After **12 s**, starts `swerve_nav_goal` (from `robotpadi_controller`) — listens for RViz's **2D Nav Goal** tool on `/goal_pose` and drives the robot there.

The staggered `TimerAction` delays exist so Gazebo, the bridge, and `ros2_control` are fully up before controllers are spawned and commands sent. On slower machines, increase these delays if nodes fail to find their dependencies on first try.

Two other launch files are included in `robotpadi_description` for lighter‑weight use:

```bash
# Gazebo + ros2_control only, empty world, no RViz/no nav node
ros2 launch robotpadi_description gazebo.launch.py

# RViz‑only kinematic viewer with joint_state_publisher_gui sliders, no Gazebo
ros2 launch robotpadi_description display.launch.py
ros2 launch robotpadi_description display.launch.py gui:=false   # use non-GUI joint_state_publisher
```

---

## `robotpadi_description`

| Path | Purpose |
|---|---|
| `urdf/robotpadi.xacro` | Main robot description (links, joints, meshes, materials) |
| `urdf/robotpadi.ros2control` | `ros2_control` hardware interface definitions (8 joints) |
| `urdf/robotpadi.gazebo` | Gazebo plugin bindings: odometry publisher + `gz_ros2_control` |
| `urdf/materials.xacro` | Simple RGBA material definitions |
| `meshes/*.stl` | Visual/collision meshes for every link |
| `config/controllers.yaml` | `controller_manager` + steering/wheel controller config |
| `config/ros_gz_bridge_gazebo.yaml` | Gazebo ⇄ ROS 2 topic bridge map (clock, odom, tf) |
| `config/display.rviz`, `config/gazebo.rviz` | RViz2 perspectives for model‑viewer and full‑sim use |
| `launch/robot.launch.py` | **Main launch file** — full Gazebo + ROS 2 control + RViz + nav node |
| `launch/gazebo.launch.py` | Gazebo + control stack only, spawns into an empty world |
| `launch/display.launch.py` | RViz‑only viewer with `joint_state_publisher_gui` (no Gazebo) |
| `worlds/` | Gazebo SDF worlds (paddy field, maize field, generic farm) |
| `worlds/generate_world.py` | Script that procedurally generates the paddy/maize field layout |
| `models/` | Gazebo models used by the worlds (maize plants, mud box, sand heightmap, paddy textures) |

## `robotpadi_controller`

This package holds the swerve‑drive kinematics and three different ways to drive the robot. Only one of them is wired into `robot.launch.py`.

| File | Console script? | Run with | What it does |
|---|---|---|---|
| `swerve_nav_goal.py` | ✅ `swerve_nav_goal` (registered in `setup.py`) | `ros2 run robotpadi_controller swerve_nav_goal` (or launched automatically by `robot.launch.py`) | Subscribes to `/odom` and `/goal_pose`. When you click **2D Nav Goal** in RViz, it drives the robot through a `ROTATE → DRIVE → ALIGN → DONE` state machine using proportional control, converting the body‑frame velocity command into 4 independent wheel speed + steering angle pairs via swerve kinematics. Also publishes a goal `Marker`, the traveled `/robot_path`, and a straight‑line `/desired_path` for visualization in RViz. |
| `swerve_controller.py` | ❌ not registered | `python3 src/robotpadi_controller/robotpadi_controller/swerve_controller.py` (run the file directly; needs `ros2_ws` sourced and `rclpy` importable) | Standalone node + a polished Tkinter GUI with **proper independent swerve kinematics** (vx/vy/omega sliders, WASD+QE keyboard control, live per‑wheel speed/angle diagram). This is the more "correct" of the two manual GUIs but isn't installed as a `ros2 run`‑able entry point. |
| `gui.py` | ❌ not registered | `python3 src/robotpadi_controller/robotpadi_controller/gui.py` | An earlier, simpler Tkinter teleop GUI. **Not true swerve drive** — it sends the *same* speed to all 4 wheels and the *same* angle to all 4 steering joints (more like a basic skid/ackermann tester than independent‑corner swerve). Useful as a minimal sanity check that the controllers and topics are wired up correctly. |

If you want `swerve_controller.py` or `gui.py` runnable via `ros2 run`, add them to `entry_points['console_scripts']` in `robotpadi_controller/setup.py`, e.g.:
```python
entry_points={
    'console_scripts': [
        'swerve_nav_goal = robotpadi_controller.swerve_nav_goal:main',
        'swerve_controller = robotpadi_controller.swerve_controller:main',
        'robot_gui = robotpadi_controller.gui:main',
    ],
},
```
then rebuild (`colcon build --packages-select robotpadi_controller`) and re‑source.

### Swerve kinematics (shared logic)

Both `swerve_nav_goal.py` and `swerve_controller.py` implement the same inverse kinematics:

```
Lx = HALF_TRACK = 0.950 / 2   # half distance between left/right wheels
Ly = HALF_BASE  = 1.090 / 2   # half distance between front/rear axles
WHEEL_RADIUS    = 0.2 m
```

For a desired body‑frame velocity `(vx, vy, ω)`, each corner's wheel velocity is:

```
vwx = vx − ω·cy
vwy = vy + ω·cx
speed  = hypot(vwx, vwy) / WHEEL_RADIUS
angle  = atan2(vwy, vwx)
```

> **Note on geometry constants:** `HALF_TRACK`/`HALF_BASE` (0.475 m / 0.545 m) are independent values hand‑set in the controller code and are **not derived from the URDF**. Cross‑check them against the actual joint offsets in `robotpadi.xacro` (e.g. steering joint origins at `±0.386`/`±0.548` m from `base_robot`) if you change the CAD model, or the kinematics will be slightly off from the physical/simulated geometry.

## Topics & ROS interfaces

Bridged from Gazebo to ROS 2 via `ros_gz_bridge` (`robotpadi_description/config/ros_gz_bridge_gazebo.yaml`):

| ROS 2 topic | Gazebo topic | Type | Direction |
|---|---|---|---|
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | GZ → ROS |
| `/odom` | `/model/robotpadi/odometry` | `nav_msgs/msg/Odometry` | GZ → ROS |
| `/tf` | `/model/robotpadi/tf` | `tf2_msgs/msg/TFMessage` | GZ → ROS |

Odometry is produced in Gazebo by the `gz::sim::systems::OdometryPublisher` plugin (`base_link` robot frame, `odom` odom frame, 50 Hz) defined in `urdf/robotpadi.gazebo`.

Published/consumed by `robotpadi_controller`:

| Topic | Type | Direction (relative to `swerve_nav_goal`) | Purpose |
|---|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Subscribed | Set via RViz's **2D Nav Goal** tool |
| `/odom` | `nav_msgs/msg/Odometry` | Subscribed | Robot pose feedback |
| `/wheel_controller/commands` | `std_msgs/msg/Float64MultiArray` | Published | Per‑wheel velocity commands |
| `/steering_controller/commands` | `std_msgs/msg/Float64MultiArray` | Published | Per‑steering‑joint position commands |
| `/goal_marker` | `visualization_msgs/msg/Marker` | Published | Arrow marker at the current goal, for RViz |
| `/robot_path` | `nav_msgs/msg/Path` | Published | Trace of the robot's actual traveled path (`odom` frame) |
| `/desired_path` | `nav_msgs/msg/Path` | Published | Straight‑line path from start to goal, for RViz comparison |

Other relevant interfaces:
- `/robot_description` — published by `robot_state_publisher`, used by `ros_gz_sim create` to spawn the model.
- `/joint_states` — published by the `joint_state_broadcaster` controller.

## Controllers

Defined in `robotpadi_description/config/controllers.yaml`, loaded via the `gz_ros2_control` plugin. `controller_manager` runs at **100 Hz**.

| Controller | Type | Joints (in order) |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all |
| `steering_controller` | `position_controllers/JointGroupPositionController` | `Revolute 1, 2, 3, 4` → FR, BR, FL, BL steering |
| `wheel_controller` | `velocity_controllers/JointGroupVelocityController` | `Revolute 5, 6, 7, 8` → BR, FL, BL, FR wheels |

## Driving the robot

**1. Click‑to‑navigate (default, via `robot.launch.py`)**
After launch, in RViz click the **2D Nav Goal** tool, then click‑and‑drag on the map to set a target position + heading. `swerve_nav_goal` will rotate to face it, drive there, then rotate to match the goal heading.

**2. Manual command‑line teleop**
```bash
# Steer all 4 modules to a given angle (radians)
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.3, 0.3, 0.3, 0.3]}"

# Drive all 4 wheels at a given angular velocity (rad/s)
ros2 topic pub /wheel_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [2.0, 2.0, 2.0, 2.0]}"
```

**3. Tkinter GUI / keyboard teleop (proper swerve kinematics)**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robotpadi_controller/robotpadi_controller/swerve_controller.py
```
Gives you `vx`/`vy`/`ω` sliders, WASD+QE keyboard control, and a live 4‑wheel speed/angle diagram. Requires `python3-tk` and Gazebo/`ros2_control` already running (e.g. via `gazebo.launch.py`).

## Worlds

| File | Used by | Description |
|---|---|---|
| `worlds/paddy_generated.world` | `robot.launch.py` (default) | Procedurally generated paddy/maize field on a heightmap terrain, 5 rows × 20 plants. **Contains hardcoded absolute model paths — see warning above.** |
| `worlds/generated.world` | — | Earlier/alternate generated field layout. Same path caveat applies. |
| `worlds/paddy.world` | — | Hand‑authored larger paddy world. |
| `worlds/farm.sdf` | — | Minimal world including `mud_box` + a single `maize_01` model. **Hardcoded absolute paths.** |
| `worlds/worlds.sdf` | — | Minimal world using the `sand_heightmap` model via `model://` URI (relies on `GZ_SIM_RESOURCE_PATH`). |

To launch the robot in a different world, edit the `world_file` variable in `launch/robot.launch.py` (or `launch/gazebo.launch.py` for the empty‑world variant) and point it at one of the files above.

## Regenerating the paddy field world

`worlds/generate_world.py` procedurally builds `paddy_generated.world` (5 groups × 10 plants per row, alternating `maize_01`/`maize_02` models on a heightmap).

```bash
cd ~/ros2_ws/src/robotpadi_description/worlds

# Edit BASE_PATH in generate_world.py to point at your models/ directory, e.g.:
#   BASE_PATH = "/home/<you>/ros2_ws/src/robotpadi_description/models"

python3 generate_world.py > paddy_generated.world
```

Layout parameters you can tune at the top of the script:

| Variable | Meaning |
|---|---|
| `N_GROUPS` | Number of plant rows/groups (Y direction) |
| `N_PLANTS` | Plants per row |
| `X_OFFSET` / `Y_OFFSET` | Shifts the whole field relative to world origin |
| `BASE_PATH` | Absolute path to the `models/` directory containing `maize_01`/`maize_02` |

After regenerating, rebuild (`colcon build`) so the updated world is copied into the install space.

## Known issues / things to check before relying on this

These are present in the code as‑is and are worth verifying/fixing for your own use rather than assumed correct:

- **Wheel/steering ordering mismatch.** `swerve_kinematics()` in both `swerve_nav_goal.py` and `swerve_controller.py` builds results in the order `[FL, RL, FR, RR]` per its `corners` list (though the docstring in `swerve_nav_goal.py` says `[RR, FL, RL, FR]` — the two comments disagree). `controllers.yaml`, however, defines `steering_controller` joints as `[Revolute 1, 2, 3, 4]` = `[FR, BR, FL, BL]` and `wheel_controller` as `[Revolute 5, 6, 7, 8]` = `[BR‑wheel, FL‑wheel, BL‑wheel, FR‑wheel]`. As written, the 4‑element command arrays published by the controller nodes are **not guaranteed to line up corner‑for‑corner** with the physical/URDF joint order. Verify this in simulation (watch which wheel steers/spins for a given command) before trusting autonomous navigation, and reorder the `corners` list or the published array indices to match if needed.
- **Axis convention is hardcoded and commented as a guess.** `swerve_nav_goal.py`'s `_publish()` has a `robot_vx, robot_vy = vx, vy` line labeled "CASE A: forward = Y axis" permanently active, with a "CASE B" alternative commented out. The accompanying comment tells you to check which axis is forward in RViz — this hasn't been confirmed for this specific URDF/mesh export. The 90° yaw offset added in `_odom_cb` (`+ math.pi / 2.0`) is a related manual correction for the same issue.
- **Geometry constants are independent of the URDF.** `HALF_TRACK`/`HALF_BASE` in the controller code are hand‑set values, not computed from `robotpadi.xacro`'s joint origins. If the CAD model changes, update both places.
- **`swerve_controller.py` and `gui.py` are not `ros2 run`‑able** — they're only registered as plain Python files, not console scripts (see [`robotpadi_controller`](#robotpadi_controller) above for how to add them, or run them directly with `python3`).
- **`gui.py` is not true swerve drive** — it sends identical speed/angle to all 4 corners.
- **World files use absolute, machine‑specific paths** — see [⚠️ Required fix before first run](#️-required-fix-before-first-run).

## Troubleshooting

**Gazebo loads but the field is empty / "Unable to find uri" errors**
You hit the hardcoded absolute path issue — see [⚠️ Required fix before first run](#️-required-fix-before-first-run).

**`ros2 launch` fails with "package 'robotpadi_controller' not found"**
Make sure you built and sourced `robotpadi_controller` too (`colcon build --packages-select robotpadi_controller`, then re‑source `install/setup.bash`) — `robot.launch.py` depends on its `swerve_nav_goal` console script.

**Robot spawns but doesn't move / controllers fail to spawn**
Controller spawning is timer‑based (8 s / 10 s delays) and assumes Gazebo + the bridge are fully initialized by then. On slower machines, increase the `period` values in the `TimerAction`s in `robot.launch.py`/`gazebo.launch.py`.

**Robot drives sideways, spins the wrong way, or the wrong wheels turn for a given command**
See [Known issues](#known-issues--things-to-check-before-relying-on-this) above — this is most likely the corner‑ordering mismatch or the forward‑axis convention, not a bug in `ros2_control` itself.

**`ModuleNotFoundError: No module named 'tkinter'` when running `swerve_controller.py` or `gui.py`**
Install Tk support: `sudo apt install python3-tk`.

**Robot model is squashed, distorted, or mis‑scaled in RViz/Gazebo**
Meshes are authored in millimeters and scaled by `0.001` in `robotpadi.xacro`. If you regenerate meshes from CAD, keep that scale factor or update it to match your export units.

**No laser/camera data even though RViz has a `LaserScan` display**
There are no sensors defined in this version of the URDF — the RViz display is an unused placeholder.

## License

Both `package.xml` files currently declare `TODO: License declaration`. Update these fields and add a `LICENSE` file before publishing/distributing the packages.