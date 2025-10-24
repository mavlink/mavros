MAVROS Example — Flight Task Control
====================================


<br><br>


Overview
--------

The `flight_task_control.py` script demonstrates how to perform basic UAV flight operations. It serves as an introductory example for users who are just starting with MAVROS and want to understand how to communicate with copters, planes, or VTOLs through MAVROS services. This example covers the most fundamental flight control commands — including **arming**, **mode switching**, **takeoff**, **landing**, and **setting home position**.



Services Used
-------------
### 1. `/mavros/cmd/arming` — Arm or Disarm the UAV
<dl>
  <dt><strong>Service Type:</strong></dt>
  <dd><code>mavros_msgs/srv/CommandBool</code></dd>

  <dt><strong>Purpose:</strong></dt>
  <dd>Enables or disables the motors of the UAV.</dd>

  <dt><strong>Usage in Script:</strong></dt>
  <dd>This service is called to arm the drone before takeoff and to disarm it after landing.</dd>

  <dt><strong>Request Fields:</strong></dt>
  <dd>
  <pre><code>value (bool):
True → Arm the vehicle (enable motors)
False → Disarm the vehicle (disable motors)
  </code></pre>
  </dd>

  <dt><strong>Example Call:</strong></dt>
  <dd><pre><code>self.arm_vehicle(True)</code></pre></dd>

  <dt><strong>Behavior:</strong></dt>
  <dd>
    <ul>
      <li>The autopilot performs safety checks (e.g., GPS lock, pre-arm conditions).</li>
      <li>If successful, the drone’s motors are powered, and the UAV becomes ready to take off.</li>
    </ul>
  </dd>
</dl>

---

### 2. `/mavros/set_mode` — Change Flight Mode
<dl>
  <dt><strong>Service Type:</strong></dt>
  <dd><code>mavros_msgs/srv/SetMode</code></dd>

  <dt><strong>Purpose:</strong></dt>
  <dd>Switches the vehicle’s current flight mode (e.g., from <code>STABILIZE</code> to <code>GUIDED</code>, <code>LAND</code>, etc.).</dd>

  <dt><strong>Usage in Script:</strong></dt>
  <dd><pre><code>self.set_mode("GUIDED")</code></pre></dd>
</dl>

---

### 3. `/mavros/cmd/takeoff` — Command Autonomous Takeoff
<dl>
  <dt><strong>Service Type:</strong></dt>
  <dd><code>mavros_msgs/srv/CommandTOL</code></dd>

  <dt><strong>Purpose:</strong></dt>
  <dd>Commands the UAV to autonomously take off to a specified altitude.</dd>

  <dt><strong>Usage in Script:</strong></dt>
  <dd><pre><code>self.takeoff(altitude=5.0)</code></pre></dd>
</dl>

---

### 4. `/mavros/cmd/land` — Command Autonomous Landing
<dl>
  <dt><strong>Service Type:</strong></dt>
  <dd><code>mavros_msgs/srv/CommandTOL</code></dd>

  <dt><strong>Purpose:</strong></dt>
  <dd>Commands the UAV to perform an autonomous landing sequence.</dd>

  <dt><strong>Usage in Script:</strong></dt>
  <dd><pre><code>self.land()</code></pre></dd>
</dl>

---

### 5. `/mavros/cmd/set_home` — Set Home Position
<dl>
  <dt><strong>Service Type:</strong></dt>
  <dd><code>mavros_msgs/srv/CommandHome</code></dd>

  <dt><strong>Purpose:</strong></dt>
  <dd>Sets or updates the vehicle’s home position, used for Return-to-Launch (RTL) operations.</dd>

  <dt><strong>Usage in Script:</strong></dt>
  <dd><pre><code>self.set_home(use_current_gps=True)</code></pre></dd>
</dl>

---

Execution Flow
--------------
1. Waits for MAVROS services to become available.  
2. Sets the home position using `/mavros/cmd/set_home`.  
3. Switches flight mode to `GUIDED`.  
4. Arms the vehicle.  
5. Takes off to the desired altitude.  
6. Hovers briefly.  
7. Initiates landing.  
8. Disarms automatically after landing.



How to Run
----------
<dl>
  <dt><strong>Step 1:</strong> Build the Package</dt>
  <dd>
  <pre><code>cd ~/ros2_ws
colcon build --packages-select mavros_examples
source install/setup.bash
  </code></pre>
  </dd>

  <dt><strong>Step 2:</strong> Launch MAVROS with SITL (ArduPilot Example)</dt>
  <dd>
  <pre><code>ros2 launch mavros apm.launch.py fcu_url:=udp://:14540@</code></pre>
  </dd>

  <dt><strong>Step 3:</strong> Run the Example Node</dt>
  <dd>
  <pre><code>ros2 run mavros_examples flight_task_control.py</code></pre>
  </dd>
</dl>

---
