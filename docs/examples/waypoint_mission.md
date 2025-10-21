# waypoint_mission_manager

## Overview
The `waypoint_mission_manager.py` example demonstrates **mission management** using **MAVROS** in **ROS 2**.  
It shows how to interact with **MAVLink mission services and topics** through MAVROS to manage missions for **copters, planes, and VTOLs**.

This example helps users understand how to upload, download, clear, and activate waypoint missions, as well as how to switch flight modes for autonomous flight operations.

> **Note:** It is assumed that the copter, plane, or VTOL is already **airborne and stable** before executing mission-related operations.

---

## Features

This example provides complete control over mission management using MAVROS services and topics:
- **Push Mission:** Upload a mission plan (set of waypoints) to the flight controller.
- **Pull Mission:** Retrieve the current mission from the flight controller.
- **Clear Mission:** Delete all waypoints from the mission list.
- **Set Current Waypoint:** Select which waypoint should be active.
- **Flight Mode Control:** Switch between modes like `AUTO`, `GUIDED`, or `STABILIZE`.
- **Mission Feedback:** Monitor active waypoints and track mission progress.

---

## MAVROS Services Used

### `/mavros/mission/push`
**Service Type:** `mavros_msgs/srv/WaypointPush`  
**Description:**  
Uploads a list of mission waypoints from the ROS 2 node to the flight controller.

**Purpose:**  
Sends a complete mission plan to the autopilot. Each waypoint contains:
- Latitude and longitude (in degrees)
- Altitude (in meters)
- MAVLink command type (e.g., `WAYPOINT`, `TAKEOFF`, `LAND`)

**Usage:**  
Called before switching to `AUTO` mode so the copter, plane, or VTOL can begin autonomous mission execution.

**Defined in Code:**
```python
self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')
```

---

### `/mavros/mission/pull`
**Service Type:** `mavros_msgs/srv/WaypointPull`  
**Description:**  
Retrieves all mission waypoints currently stored in the flight controller.

**Purpose:**  
Used to verify or review mission items already loaded on the autopilot.  
Helpful for confirming mission integrity or debugging upload results.

**Defined in Code:**
```python
self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
```

---

### `/mavros/mission/clear`
**Service Type:** `mavros_msgs/srv/WaypointClear`  
**Description:**  
Removes all stored waypoints from the flight controller.

**Purpose:**  
Ensures a clean state before uploading a new mission.  
Avoids mixing new mission items with previously stored ones.

**Defined in Code:**
```python
self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
```

---

### `/mavros/mission/set_current`
**Service Type:** `mavros_msgs/srv/WaypointSetCurrent`  
**Description:**  
Sets the currently active mission item (by index).

**Purpose:**  
Allows resuming missions from a specific waypoint instead of restarting from the first one.  
Commonly used in partially completed missions or re-flight scenarios.

**Defined in Code:**
```python
self.set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
```

---

### `/mavros/set_mode`
**Service Type:** `mavros_msgs/srv/SetMode`  
**Description:**  
Changes the current flight mode (e.g., AUTO, GUIDED, STABILIZE, etc.).

**Purpose:**  
Used to set the aircraft into AUTO mode after mission upload, enabling autonomous navigation.

**Defined in Code:**
```python
self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
```

---

## MAVROS Topics Used

### `/mavros/mission/waypoints`
**Message Type:** `mavros_msgs/msg/WaypointList`  
**Description:**  
Publishes all mission waypoints currently stored on the flight controller.

**Purpose:**  
Used to monitor mission updates in real-time.  
Triggered when new missions are pushed or existing ones are cleared.

**Defined in Code:**
```python
self.waypoints_sub = self.create_subscription(
    WaypointList,
    '/mavros/mission/waypoints',
    self.waypoints_callback,
    qos_profile
)
```

---

### `/mavros/mission/reached`
**Message Type:** `mavros_msgs/msg/WaypointReached`  
**Description:**  
Publishes a message each time the aircraft reaches a waypoint.

**Purpose:**  
Used for live feedback on mission progress.  
Helps confirm the sequence and timing of waypoint completion.

**Defined in Code:**
```python
self.reached_sub = self.create_subscription(
    WaypointReached,
    '/mavros/mission/reached',
    self.waypoint_reached_callback,
    qos_profile
)
```

---

## Example Workflow

### 1. Run the Node
```bash
ros2 run mavros_examples waypoint_mission_manager
```
Ensure that MAVROS is running and connected to the flight controller.

### 2. Clear Existing Mission
The script first clears all existing mission items:
```python
manager.clear_waypoints()
```

### 3. Create and Push New Mission
A sample mission (square flight path) is generated and uploaded:
```python
sample_mission = manager.create_sample_mission()
manager.push_waypoints(sample_mission)
```

### 4. Pull Mission to Verify
```python
manager.pull_waypoints()
```

### 5. Set a Specific Waypoint as Active
```python
manager.set_current_waypoint(2)
```

### 6. Change Flight Mode to AUTO
```python
manager.set_mode('AUTO')
```

### 7. Monitor Mission Progress
```bash
ros2 topic echo /mavros/mission/reached
```

---

## Sample Mission Pattern

The `create_sample_mission()` function generates a square-shaped flight path:

| Waypoint | Description      | Command   |
|-----------|------------------|-----------|
| 0         | Home Position    | WAYPOINT  |
| 1         | Takeoff to 10m   | TAKEOFF   |
| 2         | Move North       | WAYPOINT  |
| 3         | Move North-East  | WAYPOINT  |
| 4         | Move South-East  | WAYPOINT  |
| 5         | Return to Home   | WAYPOINT  |
| 6         | Land             | LAND      |


This pattern is suitable for both SITL simulation and real hardware tests.