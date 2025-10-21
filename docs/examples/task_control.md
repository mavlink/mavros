# MAVROS Example — Flight Task Control
## Overview
The **`flight_task_control.py`** script demonstrates how to perform **basic UAV flight operations** u
It serves as an introductory example for users who are just starting with MAVROS and want to understa
This example covers the most fundamental flight control commands — including **arming**, **mode switc
It is especially helpful for new users who want to:
- Understand the structure of MAVROS services.
- Learn how to send flight commands programmatically.
- Explore how ArduPilot or PX4 handle basic flight tasks through ROS 2.
---
## Services Used
### 1. `/mavros/cmd/arming` — Arm or Disarm the UAV
**Service Type:** `mavros_msgs/srv/CommandBool`
**Purpose:** Enables or disables the motors of the UAV.
**Usage in Script:**
This service is called to arm the drone before takeoff and to disarm it after landing.
**Request Fields:**
```python
value (bool):
True → Arm the vehicle (enable motors)
False → Disarm the vehicle (disable motors)
```
**Example Call:**
```python
self.arm_vehicle(True)
```
**Behavior:**
- The autopilot performs safety checks (e.g., GPS lock, pre-arm conditions).
- If successful, the drone’s motors are powered, and the UAV becomes ready to take off.
---
### 2. `/mavros/set_mode` — Change Flight Mode
**Service Type:** `mavros_msgs/srv/SetMode`
**Purpose:** Switches the vehicle’s current flight mode (e.g., from `STABILIZE` to `GUIDED`, `LAND`,
**Usage in Script:**
```python
self.set_mode("GUIDED")
```
---
### 3. `/mavros/cmd/takeoff` — Command Autonomous Takeoff
**Service Type:** `mavros_msgs/srv/CommandTOL`
**Purpose:** Commands the UAV to autonomously take off to a specified altitude.
**Usage in Script:**
```python
self.takeoff(altitude=5.0)
```
---
### 4. `/mavros/cmd/land` — Command Autonomous Landing
**Service Type:** `mavros_msgs/srv/CommandTOL`
**Purpose:** Commands the UAV to perform an autonomous landing sequence.
**Usage in Script:**
```python
self.land()
```
---
### 5. `/mavros/cmd/set_home` — Set Home Position
**Service Type:** `mavros_msgs/srv/CommandHome`
**Purpose:** Sets or updates the vehicle’s “home” position, used for Return-to-Launch (RTL) operation
**Usage in Script:**
```python
self.set_home(use_current_gps=True)
```
---
## Execution Flow
1. Waits for MAVROS services to become available.
2. Sets the home position using `/mavros/cmd/set_home`.
3. Switches flight mode to `GUIDED`.
4. Arms the vehicle.
5. Takes off to the desired altitude.
6. Waits briefly in hover.
7. Initiates landing.
8. Disarms automatically after landing.
---
## How to Run
### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select mavros_examples
source install/setup.bash
```
### 2. Launch MAVROS with SITL (ArduPilot Example)
```bash
ros2 launch mavros apm.launch.py fcu_url:=udp://:14540@
```
### 3. Run the Example Node
```bash
ros2 run mavros_examples flight_task_control.py
```
---
