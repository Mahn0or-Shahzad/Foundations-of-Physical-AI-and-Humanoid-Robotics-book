---
id: gazebo-simulation
title: Module 2A - Digital Twin Simulation with Gazebo
sidebar_position: 4
description: Creating high-fidelity digital twins of humanoid robots using Gazebo simulation for physics-accurate testing.
keywords: [Gazebo, digital twin, simulation, physics engine, SDF, URDF, sensor simulation]
---

# Module 2A: Digital Twin Simulation with Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the role of simulation in Physical AI development and the digital twin paradigm
2. Set up Gazebo Fortress/Classic for humanoid robot simulation
3. Convert URDF models to SDF (Simulation Description Format) for Gazebo
4. Configure physics engines (ODE, Bullet, DART) and understand their trade-offs
5. Create Gazebo world files with realistic environments (terrain, obstacles, lighting)
6. Simulate sensors (IMU, LiDAR, RGB-D cameras) with realistic noise models
7. Launch complete humanoid simulations with sensor data publishing to ROS 2 topics
8. Validate simulation fidelity and debug common physics issues

---

## Why Simulation? The Digital Twin Paradigm

Physical AI systems face a fundamental challenge: **learning from physical interaction is slow and risky**. Training a humanoid to walk by trial-and-error on hardware would require thousands of attempts—each fall risking damage to expensive actuators and sensors. Physical experimentation is:

- **Slow**: Real-time only (no speedup)
- **Expensive**: Hardware wear, component failure
- **Dangerous**: Falls, collisions, uncontrolled behaviors
- **Limited**: Cannot easily test edge cases (extreme terrain, sensor failures)

**Digital twins**—high-fidelity virtual replicas of physical robots—solve this by enabling:

- **Rapid Prototyping**: Test algorithms in minutes instead of months
- **Parallel Exploration**: Run hundreds of simulations simultaneously (via GPU parallelization)
- **Safe Failure**: Robots can fall, collide, or break without consequence
- **Systematic Validation**: Test edge cases (steep slopes, low-light conditions, sensor occlusions) impractical to reproduce physically
- **Reproducibility**: Exact same initial conditions, deterministic physics (if configured)

The **simulation-to-real (Sim2Real)** workflow has become standard in robotics: develop and validate algorithms in simulation, then transfer to physical hardware with domain randomization and fine-tuning (Zhao et al., 2020). For humanoid robotics, where bipedal balance and dexterous manipulation are contact-rich and difficult to model analytically, simulation is not optional—it is **essential**.

---

## Gazebo: Physics-Accurate Robotic Simulation

**Gazebo** (now called Gazebo Sim or Ignition Gazebo for newer versions) is an open-source 3D robotics simulator with accurate physics, diverse sensor models, and native ROS 2 integration (Koenig & Howard, 2004). Unlike game engines (Unity, Unreal) optimized for visual fidelity, Gazebo prioritizes **physics accuracy** and **sensor realism**—critical for algorithm validation.

### Gazebo Architecture

Gazebo separates concerns into modular components:

1. **Physics Engine**: Simulates rigid body dynamics, collisions, contact forces
   - **ODE** (Open Dynamics Engine): Default, stable, moderate accuracy
   - **Bullet**: Higher accuracy, better collision detection, GPU acceleration available
   - **DART** (Dynamic Animation and Robotics Toolkit): Most accurate, supports contacts and constraints

2. **Rendering Engine**: Visualizes environments using OGRE (3D graphics)
   - Not photorealistic (unlike Unity/Unreal)
   - Sufficient for algorithm development and debugging

3. **Sensor Models**: Simulate LiDAR (ray-casting), cameras (rendering), IMUs (physics integration)
   - Include realistic noise, latency, and artifacts

4. **Plugin System**: Extend functionality (custom sensors, controllers, world behaviors)
   - Gazebo-ROS 2 plugins bridge simulation to ROS 2 topics

5. **World Files (SDF)**: Define environments, lighting, gravity, objects

### Installing Gazebo for ROS 2 Humble

```bash
# Gazebo Fortress (recommended for Humble)
sudo apt install ros-humble-gazebo-ros-pkgs

# Or Gazebo Classic 11 (legacy but stable)
sudo apt install ros-humble-gazebo-ros

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x (Classic)
# Or: Gazebo Sim, version 6.x (Fortress)
```

**Launch Gazebo**:
```bash
# Empty world
gazebo

# Or via ROS 2 launch
ros2 launch gazebo_ros gazebo.launch.py
```

---

## From URDF to SDF: Simulation Description Format

While URDF describes robot structure, **SDF (Simulation Description Format)** extends this for complete simulation environments. SDF supports:

- **Multiple robots** in one world
- **Worlds**: Ground planes, lighting, physics configuration
- **Plugins**: Advanced sensor models, environmental effects
- **Richer physics**: More detailed contact properties, friction models

### Converting URDF to SDF

Gazebo can load URDF directly, but converting to SDF provides more control:

```bash
# Convert URDF to SDF
gz sdf -p simple_humanoid.urdf > simple_humanoid.sdf

# Inspect generated SDF
cat simple_humanoid.sdf
```

**Key Differences**:

| Feature | URDF | SDF |
|---------|------|-----|
| **Format** | ROS-specific XML | Gazebo-native XML |
| **Worlds** | Robot only | Robot + environment |
| **Physics** | Basic inertial properties | Detailed contact, friction, damping |
| **Multiple Models** | One robot per file | Multiple robots + objects per world |
| **Plugins** | Limited | Extensive (sensors, actuators, custom behaviors) |

**Best Practice**: Develop in URDF (for ROS 2 compatibility), convert to SDF for simulation (for physics richness).

---

## Creating a Gazebo World for Humanoid Testing

A Gazebo **world file** defines the environment where your robot operates. Let's create a simple indoor environment:

Create `worlds/humanoid_test_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_test_world">

    <!-- Physics engine configuration -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>  <!-- Z-up -->
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>  <!-- Coefficient of friction (concrete-like) -->
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Table (obstacle and manipulation target) -->
    <model name="table">
      <static>true</static>
      <pose>2.0 0 0 0 0 0</pose>  <!-- 2m in front of robot -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.8 0.8 0.75</size>  <!-- 80cm × 80cm × 75cm table -->
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.8 0.8 0.75</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>  <!-- Brown wood color -->
          </material>
        </visual>
      </link>
    </model>

    <!-- Include humanoid robot model -->
    <include>
      <uri>model://simple_humanoid</uri>
      <pose>0 0 1.0 0 0 0</pose>  <!-- Spawn 1m above ground -->
    </include>

  </world>
</sdf>
```

**World File Components**:

1. **`<physics>`**: Timestep, solver, real-time factor
2. **`<light>`**: Directional sunlight, point lights, spotlights
3. **`<model>`**: Static objects (ground, tables, walls) or dynamic objects (boxes, tools)
4. **`<include>`**: Import external models (robots, props)

### Launching Gazebo with Custom World

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/humanoid_test_world.sdf
```

---

## Physics Simulation: Making Digital Twins Realistic

### Gravity and Rigid Body Dynamics

Gazebo simulates Newtonian mechanics:

- **Gravity**: Default `-9.81 m/s²` in Z direction (configurable per world)
- **Inertia**: Computed from link mass and inertia tensor in URDF
- **Forces**: Applied to links (motor torques, external disturbances, collisions)

**Equations of Motion** (simplified):

For each link, Gazebo solves:

```
τ = I·α + ω × (I·ω)  (Euler's rotation equation)
F = m·a              (Newton's second law)
```

Where:
- `τ` = applied torque (from motors, gravity, contacts)
- `I` = inertia tensor (from URDF `<inertial>`)
- `α` = angular acceleration
- `ω` = angular velocity
- `F` = applied force
- `m` = link mass
- `a` = linear acceleration

**Practical Implication**: If your URDF has incorrect inertial properties (e.g., head is too heavy), the simulated humanoid will tip forward unrealistically. Accurate `<inertial>` blocks are critical for believable simulation.

### Collision Detection and Contact Forces

Gazebo computes contact forces when geometries intersect:

```xml
<collision name="collision">
  <geometry>
    <box size="0.3 0.2 0.5"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>   <!-- Friction coefficient (0=frictionless, 1=high friction) -->
        <mu2>0.8</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <soft_cfm>0.01</soft_cfm>  <!-- Contact softness (compliance) -->
        <soft_erp>0.2</soft_erp>    <!-- Error reduction parameter -->
      </ode>
    </contact>
  </surface>
</collision>
```

**Contact Parameters**:
- **`<mu>` / `<mu2>`**: Friction coefficients (prevent slipping)
- **`<soft_cfm>`**: Constraint Force Mixing (makes contacts "soft" vs rigid)
- **`<soft_erp>`**: Error Reduction Parameter (how quickly interpenetrations are resolved)

**Humanoid Foot Contact**: For bipedal walking, realistic foot-ground friction is essential. Use `mu=0.8` for concrete-like surfaces. Too low (`mu=0.1`) causes slipping; too high (`mu=2.0`) can cause numerical instabilities.

### Physics Engine Selection

Gazebo supports multiple physics engines:

| Engine | Accuracy | Speed | Best For |
|--------|----------|-------|----------|
| **ODE** | Medium | Fast | General-purpose, default choice |
| **Bullet** | High | Medium | Accurate collisions, contact-rich tasks (grasping) |
| **DART** | Very High | Slow | Research, precise dynamics, constraint-based control |

**Switching engines** (in world SDF):
```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Recommendation for Humanoids**: Start with **ODE** for fast iteration. Use **Bullet** when validating balance controllers or manipulation grasps. Use **DART** for research on contact-rich locomotion.

---

## Sensor Simulation in Gazebo

Gazebo's sensor plugins generate data mimicking real hardware:

### IMU Simulation

**What it simulates**:
- **Linear Acceleration**: Gravity + robot acceleration (in robot frame)
- **Angular Velocity**: Rotation rates (gyroscope)
- **Orientation**: Estimated via physics integration (quaternion)

**Realism features**:
- **Gaussian noise**: Simulates sensor inaccuracy (`<noise>` tag)
- **Bias drift**: IMUs drift over time (optional, advanced)
- **Update rate**: Configurable (100-200 Hz typical)

**Example with noise**:
```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="torso_imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>  <!-- 0.01 rad/s noise -->
          </noise>
        </x>
        <!-- Repeat for y, z -->
      </angular_velocity>
      <linear_acceleration>
        <!-- Similar noise configuration -->
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR (Ray-Based Laser Scanner)

**How it works**: Gazebo casts rays from the sensor origin, detecting intersections with collision geometries.

**Configuration**:
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>       <!-- Number of rays -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>   <!-- Minimum detection distance (m) -->
        <max>30.0</max>  <!-- Maximum detection distance (m) -->
        <resolution>0.01</resolution>  <!-- Range precision (cm) -->
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>  <!-- 2cm measurement noise -->
      </noise>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=/scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Verification**:
```bash
# Launch Gazebo with humanoid
ros2 launch humanoid_gazebo spawn_humanoid.launch.py

# Check LiDAR topic
ros2 topic hz /scan
# Expected: ~10 Hz

# Visualize in RViz
rviz2
# Add LaserScan display, topic=/scan
# Should see point cloud representing obstacles
```

### RGB-D Camera Simulation

**RGB-D cameras** (e.g., Intel RealSense, Kinect) provide both color images and depth maps:

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="rgbd_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.3</near>   <!-- Minimum depth (m) -->
        <far>10.0</far>    <!-- Maximum depth (m) -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Depth noise (matches real sensors ~7mm) -->
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
        <remapping>depth/image_raw:=depth</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Published Topics**:
- `/camera/image_raw` (sensor_msgs/Image) - RGB image
- `/camera/depth` (sensor_msgs/Image, encoding=32FC1) - Depth map in meters
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Calibration parameters

**Use Cases**:
- **RGB**: Object detection (YOLO, DOPE)
- **Depth**: 3D localization of objects, obstacle avoidance
- **Combined**: Visual SLAM, 3D reconstruction (nvblox)

---

## Complete Humanoid Simulation Workflow

### Step 1: Prepare the Humanoid URDF

Starting from Module 1B's URDF, ensure:

1. ✅ All links have valid `<inertial>` blocks (mass, CoM, inertia tensor)
2. ✅ All joints have `<limit>` tags (effort, velocity, position limits)
3. ✅ Sensor links defined (camera_link, lidar_link, imu_link)
4. ✅ Gazebo plugins added for all sensors (camera, LiDAR, IMU)
5. ✅ Collision geometries simplified (use primitives: box, sphere, cylinder)

**Validate**:
```bash
check_urdf simple_humanoid.urdf
# Expected: "Successfully parsed urdf file"
```

### Step 2: Create Launch File

Create `launch/gazebo_humanoid.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'simple_humanoid.urdf'
    )

    # Read URDF contents
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
                '/gazebo.launch.py'
            ]),
            launch_arguments={'world': 'worlds/humanoid_test_world.sdf'}.items()
        ),

        # Spawn humanoid robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid', '-topic', '/robot_description'],
            output='screen'
        ),

        # Publish robot description to /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # Publish joint states (from Gazebo to ROS 2)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

**Key Components**:

1. **Gazebo Launch**: Starts Gazebo server and client with specified world
2. **Spawn Entity**: Inserts robot into running Gazebo simulation
3. **Robot State Publisher**: Computes TF transforms from URDF and `/joint_states`
4. **Joint State Publisher**: Bridges Gazebo's joint states to ROS 2 topic

**Launch**:
```bash
ros2 launch humanoid_gazebo gazebo_humanoid.launch.py
```

**Expected Result**: Gazebo opens with the test world (ground plane, table), humanoid spawns at origin, sensors begin publishing to ROS 2 topics.

### Step 3: Verify Sensor Data

```bash
# List topics
ros2 topic list
# Expected: /camera/image_raw, /camera/depth, /scan, /imu/data, /joint_states, /tf, /tf_static

# Check publication rates
ros2 topic hz /camera/image_raw  # ~30 Hz
ros2 topic hz /scan              # ~10 Hz
ros2 topic hz /imu/data          # ~100 Hz

# Visualize in RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "base_link"
# 2. Add RobotModel display (reads /robot_description)
# 3. Add Camera display (topic=/camera/image_raw)
# 4. Add LaserScan display (topic=/scan)
# 5. Add TF display to visualize coordinate frames
```

### Step 4: Control the Humanoid

Send joint commands to move the robot:

```bash
# Publish joint command (move neck yaw)
ros2 topic pub --once /joint_commands sensor_msgs/JointState \
  "{name: ['neck_yaw'], position: [0.5]}"

# Humanoid's head should turn 0.5 radians (~28°)
```

For programmatic control, create a Python node publishing to `/joint_commands` at 50 Hz (see Module 1B examples).

---

## Common Simulation Issues and Solutions

### Issue 1: Robot Falls Through Ground

**Symptom**: Humanoid spawns and immediately falls through the ground plane.

**Causes**:
- Missing or incorrect collision geometry on ground plane
- Robot spawned below ground (`<pose>` Z coordinate negative)
- Insufficient physics solver iterations

**Solution**:
```xml
<!-- Ensure ground has collision -->
<collision name="collision">
  <geometry>
    <plane><normal>0 0 1</normal></plane>
  </geometry>
</collision>

<!-- Spawn robot above ground (Z ≥ 1.0 for humanoid) -->
<pose>0 0 1.0 0 0 0</pose>

<!-- Increase solver iterations if needed -->
<physics type="ode">
  <ode>
    <solver>
      <iterations>50</iterations>  <!-- Default: 50, increase if unstable -->
    </solver>
  </ode>
</physics>
```

### Issue 2: Robot is Unstable / Vibrates

**Symptom**: Humanoid shakes or oscillates when standing.

**Causes**:
- Timestep too large (physics not converging)
- Inertia tensors incorrect (links too light or heavy)
- Contact parameters too stiff

**Solution**:
```xml
<!-- Reduce timestep -->
<max_step_size>0.0005</max_step_size>  <!-- 0.5ms instead of 1ms -->

<!-- Tune contact parameters -->
<soft_cfm>0.01</soft_cfm>  <!-- Increase for softer contacts -->
<soft_erp>0.2</soft_erp>   <!-- Decrease for slower error correction -->
```

**Validate inertia**: Use CAD software (SolidWorks, Fusion 360) to compute accurate inertia tensors, or use approximations:

```
For uniform box: Ixx = (1/12) * m * (h² + d²)
For sphere: I = (2/5) * m * r²
For cylinder: Izz = (1/2) * m * r²
```

### Issue 3: Sensors Publish No Data

**Symptom**: Topics exist but no messages received.

**Causes**:
- Plugin not loaded (Gazebo errors in terminal)
- Incorrect `<frame_name>` (sensor frame doesn't exist in URDF)
- Gazebo-ROS bridge not functioning

**Solution**:
```bash
# Check Gazebo logs for plugin errors
# Common: "[Err] [Plugin.hh:178] Failed to load plugin..."

# Verify sensor frame exists in URDF
grep -i "camera_link" simple_humanoid.urdf

# Ensure use_sim_time is true
ros2 param get /robot_state_publisher use_sim_time
# Expected: Boolean value is: True
```

---

## Practical Exercises: Gazebo Simulation Mastery

### Exercise 1: Create a Gazebo World with Obstacles

**Objective**: Build an environment for humanoid navigation testing.

**Tasks**:
1. Create `indoor_world.sdf` with ground plane, 4 walls, 2 tables, 3 boxes
2. Configure realistic lighting (ambient + directional)
3. Set physics to ODE with 0.001s timestep
4. Launch Gazebo and verify obstacles render correctly

**Success Criteria**:
- World loads without errors
- Robot can be spawned and remains stable
- Obstacles have collision properties (robot doesn't pass through walls)

### Exercise 2: Add Force-Torque Sensors to Feet

**Objective**: Simulate ground reaction forces for balance control.

**Tasks**:
1. Create `left_foot_link` and `right_foot_link` (if not already in URDF)
2. Add Gazebo force-torque sensor plugin to both feet
3. Launch simulation and apply external forces (push robot with Gazebo GUI)
4. Subscribe to `/left_foot/force` and `/right_foot/force` topics
5. Observe force values change as robot shifts weight

**Success Criteria**:
- Force sensors publish at configured rate (over 50 Hz)
- Standing on both feet: ~50% weight per foot
- Leaning: Weight shifts to one foot (force increases)

### Exercise 3: Tune Physics for Stable Humanoid Standing

**Objective**: Achieve stable standing pose without oscillation.

**Tasks**:
1. Spawn humanoid in Gazebo (should stand upright initially)
2. Observe behavior: Does it remain stable or fall/vibrate?
3. If unstable, adjust:
   - Inertial properties (check CoM locations, inertia tensors)
   - Contact parameters (`soft_cfm`, `soft_erp`)
   - Timestep (`max_step_size`)
4. Iterate until robot stands stably for 10+ seconds

**Success Criteria**:
- Robot stands without falling for 10 seconds
- No excessive vibration (less than 1cm oscillation)
- Joints remain within limits

### Exercise 4: Simulate Multi-Modal Sensor Suite

**Objective**: Integrate camera, LiDAR, and IMU on humanoid.

**Tasks**:
1. Add all three sensors to URDF with Gazebo plugins
2. Launch in Gazebo world with objects (table, boxes)
3. Verify all sensor topics publish:
   - `/camera/image_raw` (RGB image)
   - `/camera/depth` (depth map)
   - `/scan` (2D LiDAR)
   - `/imu/data` (orientation, acceleration)
4. Visualize all sensors simultaneously in RViz

**Success Criteria**:
- All 4 sensor topics publish at expected rates
- Camera shows objects in scene
- LiDAR detects obstacles (visualize point cloud)
- IMU shows correct orientation (tilt robot → IMU values change)

---

## Performance Considerations

### Real-Time Factor

**Real-time factor (RTF)** measures simulation speed relative to real time:

- **RTF = 1.0**: Simulation runs at real-time speed (1 simulated second = 1 real second)
- **RTF = 0.5**: Simulation runs at half speed (common for complex scenes)
- **RTF = 2.0**: Simulation runs at double speed (simplified physics)

**Check RTF** in Gazebo GUI (bottom-right corner) or via topic:
```bash
ros2 topic echo /clock
# Compare sim_time to real_time
```

**Improving RTF**:
- Simplify collision geometries (fewer vertices)
- Reduce sensor update rates (30 Hz → 10 Hz if acceptable)
- Use ODE instead of Bullet/DART (faster but less accurate)
- Disable shadows and advanced rendering

**Trade-off**: Faster simulation (higher RTF) vs more accurate physics. For humanoid balance validation, prioritize accuracy (RTF ≥ 0.8). For navigation testing in large environments, prioritize speed (RTF ≥ 1.0).

### Gazebo Performance Tips

1. **Simplify Meshes**: Use collision primitives (box, sphere) instead of complex STL meshes
2. **Batch Simulations**: Use `headless` mode (no GUI) for faster execution:
   ```bash
   gazebo --verbose -s libgazebo_ros_factory.so --headless
   ```
3. **Limit Sensor Range**: LiDAR with 30m range is faster than 100m range
4. **Use LOD**: Level-of-detail models (high-poly visual, low-poly collision)

---

## Comparing Simulation to Reality: The Reality Gap

Even the best simulators have limitations. Key differences between Gazebo and physical robots:

| Aspect | Simulation (Gazebo) | Physical Reality |
|--------|---------------------|------------------|
| **Physics** | Approximate (ODE, Bullet) | Exact (governed by nature) |
| **Contacts** | Soft constraints, can interpenetrate slightly | Hard constraints, no interpenetration |
| **Friction** | Simplified Coulomb friction | Complex (static/dynamic, anisotropic) |
| **Sensors** | Ideal ray-casting or rendering | Noise, occlusions, calibration errors |
| **Timing** | Deterministic (if configured) | Asynchronous, jitter, latency |
| **Environment** | Controlled, repeatable | Unpredictable, unmodeled factors |

**Simulation-to-Real Transfer Strategies** (covered in Module 3):
- **Domain Randomization**: Randomize friction, mass, sensor noise to cover reality gap
- **System Identification**: Measure real robot parameters, adjust simulation to match
- **Residual Policies**: Train in simulation, fine-tune on hardware

**Pragmatic Expectation**: Algorithms achieving 95% success in simulation may achieve 70-85% on real hardware without additional tuning. This gap is expected and manageable through Sim2Real techniques.

---

## Summary and Next Steps

You now understand:

✅ **Digital twin paradigm**: Virtual replicas for safe, rapid prototyping
✅ **Gazebo architecture**: Physics engine, rendering, sensors, plugins
✅ **URDF to SDF conversion**: Leveraging both formats
✅ **Physics simulation**: Gravity, collisions, contact forces, solver configuration
✅ **Sensor simulation**: IMU, LiDAR, RGB-D cameras with realistic noise
✅ **Complete workflow**: URDF preparation → world creation → launch file → sensor verification
✅ **Debugging**: Common issues (robot falls, instability, sensor failures) and solutions
✅ **Performance**: Real-time factor, optimization strategies

**In Module 2B (next chapter)**, we'll explore **Unity** for high-fidelity visualization and human-robot interaction. Unity's photorealistic rendering complements Gazebo's physics accuracy, providing visual realism for demonstrations and VR/AR applications.

**Module 2 Validation Checkpoint**: You should be able to create a Gazebo world, spawn a humanoid with sensors, verify sensor topics publish correctly, and achieve stable standing simulation. If you can do this, you're ready for Unity integration!

---

## References

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *Proceedings of the 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2149-2154. https://doi.org/10.1109/IROS.2004.1389727

Collins, J., Chand, S., Vanderkop, A., & Howard, D. (2021). A review of physics simulators for robotic applications. *IEEE Access*, 9, 51416-51431. https://doi.org/10.1109/ACCESS.2021.3068769

Gazebo Documentation (2023). *SDF Format Specification*. Open Robotics. http://sdformat.org/

Zhao, W., Queralta, J. P., & Westerlund, T. (2020). Sim-to-real transfer in deep reinforcement learning for robotics: A survey. *IEEE Symposium Series on Computational Intelligence (SSCI)*, 737-744. https://doi.org/10.1109/SSCI47803.2020.9308468

ROS 2 Documentation (2023). *URDF in Gazebo*. Open Robotics. https://classic.gazebosim.org/tutorials?tut=ros_urdf
