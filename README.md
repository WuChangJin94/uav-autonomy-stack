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

