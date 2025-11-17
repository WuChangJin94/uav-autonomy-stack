# UAV Autonomy Stack

End-to-end drone autonomy stack combining:

- **ROS Noetic** for perception, control, and middleware
- **MOOS-IvP** for high-level mission planning and behaviors
- **ArduPilot** (Copter) for low-level flight control and EKF
- **NVIDIA Jetson Orin Nano (JetPack)** for onboard deployment
- Support for **GPS / UWB / RTK** localization in **SITL**, **HITL**, and **real flight**

---

## Features

1. **End-to-End Autonomy with ROS + MOOS**
   - ROS Noetic catkin workspace with `uav_autonomy` package
   - MOOS-IvP missions and IvP behaviors for UAV
   - `moos-ros-bridge` plumbing ROS topics ↔ MOOS variables

2. **Simulation: SITL & HITL**
   - ArduPilot SITL connection via MAVROS (UDP)
   - Optional Gazebo / simple 3D visualization
   - Simulated localization with:
     - GPS only
     - UWB / ExternalNav (simulated Pozyx)
     - RTK (simulated high-accuracy GPS)

3. **Deployment on Real UAV (Jetson Orin Nano)**
   - JetPack-based container / environment
   - Same ROS + MOOS + MAVROS stack used in simulation
   - ExternalNav from Pozyx UWB or other systems via MAVROS
   - Compatible with ArduPilot-based flight controllers (Pixhawk, Cube, etc.)

---

## High-Level Architecture

### Components

- **ArduPilot (Copter)**  
  - Runs on flight controller
  - EKF3 with multiple source lanes:
    - Lane 1: GPS
    - Lane 2: ExternalNav (UWB / vision)
  - Exposes MAVLink over serial/UDP

- **ROS Noetic (Companion Computer / Dev PC)**
  - `mavros`: MAVLink ↔ ROS bridge
  - `uav_autonomy`:
    - `pozyx_to_mavros_vision.py`: UWB → ExternalNav (VISION_POSITION_ESTIMATE)
    - `position_controller.py`: position-control using `/mavros/setpoint_position/local`
    - `state_bridge_moos_ros.py`: ROS ↔ MOOS variable conversion
  - Optional perception / mapping nodes

- **MOOS-IvP**
  - `MOOSDB` central database
  - `pHelmIvP` for behavior-based mission execution
  - Mission files in `moos/missions/`
  - IvP behaviors in `moos/behaviors/`
  - Uses UAV position (from ROS) and publishes desired waypoints / loiter points

- **NVIDIA Jetson Orin Nano**
  - Runs JetPack (Ubuntu)
  - Runs the same ROS + MOOS stack for real hardware
  - Communicates with flight controller over serial (e.g., USB/TELEM)

---

## Getting Started

### 1. Clone

```bash
git clone https://github.com/<your-user>/uav-autonomy-stack.git
cd uav-autonomy-stack
git submodule add https://github.com/ArduPilot/ardupilot.git ardupilot
```


### 2. Build ROS Workspace (PC / x86)

```bash
cd ros_ws
rosws_path=$(pwd)
mkdir -p src
cd src

# (Optional) clone mavros & moos-ros-bridge here if not installed globally
# git clone https://github.com/mavlink/mavros.git
# git clone https://github.com/udecrobotics/moos-ros-bridge.git

cd "$rosws_path"
catkin_make
source devel/setup.bash
```

### 3. Install Dependencies

- ROS Noetic (desktop-full or base + required packages)
- MAVROS + geographiclib datasets
- MOOS-IvP (built from source or system packages)
- Pozyx Python library (pypozyx) and Pozyx ROS node (for real UWB)

---

### Running in SITL
1. Start ArduPilot SITL (example):
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```
2. In another terminal, bring up ROS stack:
```bash
cd uav-autonomy-stack/ros_ws
source devel/setup.bash
roslaunch uav_autonomy sitl_bringup.launch
```
3. Start MOOS mission:
```bash
cd uav-autonomy-stack/moos/missions
pAntler sim_uav_sitl.moos
```
MOOS will publish desired setpoints → ROS → MAVROS → ArduPilot SITL.

### Running on Jetson Orin Nano (Real Drone)
1. Flash JetPack and bring up Ubuntu (Jetson Orin Nano).
2. Clone this repo on the Jetson.
3. Install ROS Noetic (or use Docker via docker/Dockerfile.jetson_orin).
4. Build ros_ws as above.
5. Connect flight controller via USB and configure serial (e.g., /dev/ttyACM0).
6. Launch hardware stack:
```bash
cd uav-autonomy-stack/ros_ws
source devel/setup.bash
roslaunch uav_autonomy jetson_bringup.launch
```
7. Start MOOS mission for hardware:
```bash
cd uav-autonomy-stack/moos/missions
pAntler hw_uav_jetson.moos
```
