MAVROS Example — Waypoint Mission Manager
=========================================


<br><br>


Overview
--------

The `waypoint_mission_manager.py` example demonstrates mission management using MAVROS in ROS 2. It shows how to interact with **MAVLink mission services and topics** through MAVROS to manage missions for copters, planes, and VTOLs. This example helps users understand how to upload, download, clear, and activate waypoint missions, as well as how to switch flight modes for autonomous flight operations.

!!! note

    It is assumed that the copter, plane, or VTOL is already **airborne and stable** before executing mission-related operations.


Features
--------

This example provides complete control over mission management using MAVROS services and topics:
- **Push Mission:** Upload a mission plan (set of waypoints) to the flight controller. 
- **Pull Mission:** Retrieve the current mission from the flight controller. 
- **Clear Mission:** Delete all waypoints from the mission list. 
- **Set Current Waypoint:** Select which waypoint should be active. 
- **Flight Mode Control:** Switch between modes like `AUTO`, `GUIDED`, or `STABILIZE`. 
- **Mission Feedback:** Monitor active waypoints and track mission progress.


<br>


MAVROS Services Used
--------------------


### `/mavros/mission/push`

<dl>
  <dt><b>Service Type:</b></dt>
  <dd><code>mavros_msgs/srv/WaypointPush</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Uploads a list of mission waypoints from the ROS 2 node to the flight controller.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Sends a complete mission plan to the autopilot. Each waypoint contains latitude, longitude, altitude, and a MAVLink command type (e.g., <code>WAYPOINT</code>, <code>TAKEOFF</code>, <code>LAND</code>).</dd>

  <dt><b>Usage:</b></dt>
  <dd>Called before switching to <code>AUTO</code> mode so the copter, plane, or VTOL can begin autonomous mission execution.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')</code></pre>
  </dd>
</dl>


---

### `/mavros/mission/pull`

<dl>
  <dt><b>Service Type:</b></dt>
  <dd><code>mavros_msgs/srv/WaypointPull</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Retrieves all mission waypoints currently stored in the flight controller.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Used to verify or review mission items already loaded on the autopilot. Helpful for confirming mission integrity or debugging upload results.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')</code></pre>
  </dd>
</dl>


---

### `/mavros/mission/clear`

<dl>
  <dt><b>Service Type:</b></dt>
  <dd><code>mavros_msgs/srv/WaypointClear</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Removes all stored waypoints from the flight controller.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Ensures a clean state before uploading a new mission. Avoids mixing new mission items with previously stored ones.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')</code></pre>
  </dd>
</dl>


---

### `/mavros/mission/set_current`

<dl>
  <dt><b>Service Type:</b></dt>
  <dd><code>mavros_msgs/srv/WaypointSetCurrent</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Sets the currently active mission item by index.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Allows resuming missions from a specific waypoint instead of restarting from the first one. Useful for partially completed missions or re-flight scenarios.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')</code></pre>
  </dd>
</dl>


---

### `/mavros/set_mode`

<dl>
  <dt><b>Service Type:</b></dt>
  <dd><code>mavros_msgs/srv/SetMode</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Changes the current flight mode (e.g., <code>AUTO</code>, <code>GUIDED</code>, <code>STABILIZE</code>).</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Used to set the aircraft into <code>AUTO</code> mode after mission upload, enabling autonomous navigation.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')</code></pre>
  </dd>
</dl>


<br>


MAVROS Topics Used
------------------


### `/mavros/mission/waypoints`

<dl>
  <dt><b>Message Type:</b></dt>
  <dd><code>mavros_msgs/msg/WaypointList</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Publishes all mission waypoints currently stored on the flight controller.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Used to monitor mission updates in real time. Triggered when new missions are pushed or existing ones are cleared.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.waypoints_sub = self.create_subscription(
    WaypointList,
    '/mavros/mission/waypoints',
    self.waypoints_callback,
    qos_profile
)</code></pre>
  </dd>
</dl>


---

### `/mavros/mission/reached`

<dl>
  <dt><b>Message Type:</b></dt>
  <dd><code>mavros_msgs/msg/WaypointReached</code></dd>

  <dt><b>Description:</b></dt>
  <dd>Publishes a message each time the aircraft reaches a waypoint.</dd>

  <dt><b>Purpose:</b></dt>
  <dd>Used for live feedback on mission progress and to confirm the sequence of waypoint completion.</dd>

  <dt><b>Defined in Code:</b></dt>
  <dd>
  <pre><code>self.reached_sub = self.create_subscription(
    WaypointReached,
    '/mavros/mission/reached',
    self.waypoint_reached_callback,
    qos_profile
)</code></pre>
  </dd>
</dl>


<br><br>


Example Workflow
----------------

<dl>
  <dt><b>1. Run the Node:</b></dt>
  <dd>
  <pre><code>ros2 run mavros_examples waypoint_mission_manager</code></pre>
  Ensure that MAVROS is running and connected to the flight controller.
  </dd>

  <dt><b>2. Clear Existing Mission:</b></dt>
  <dd>
  <pre><code>manager.clear_waypoints()</code></pre>
  </dd>

  <dt><b>3. Create and Push New Mission:</b></dt>
  <dd>
  <pre><code>sample_mission = manager.create_sample_mission()
manager.push_waypoints(sample_mission)</code></pre>
  </dd>

  <dt><b>4. Pull Mission to Verify:</b></dt>
  <dd>
  <pre><code>manager.pull_waypoints()</code></pre>
  </dd>

  <dt><b>5. Set a Specific Waypoint as Active:</b></dt>
  <dd>
  <pre><code>manager.set_current_waypoint(2)</code></pre>
  </dd>

  <dt><b>6. Change Flight Mode to AUTO:</b></dt>
  <dd>
  <pre><code>manager.set_mode('AUTO')</code></pre>
  </dd>

  <dt><b>7. Monitor Mission Progress:</b></dt>
  <dd>
  <pre><code>ros2 topic echo /mavros/mission/reached</code></pre>
  </dd>
</dl>


<br>


Sample Mission Pattern
----------------------

The `create_sample_mission()` function generates a **square-shaped flight path**:

| WP | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|----|---|---|---|---|---|---|---|
| **Description** | Home | Takeoff 10 m | Move N | Move NE | Move SE | Return Home | Land |
| **Command**     | WAYPOINT | TAKEOFF | WAYPOINT | WAYPOINT | WAYPOINT | WAYPOINT | LAND |

This mission pattern works well for **SITL simulations** and **real hardware testing**.


<br><br>
