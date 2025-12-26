---
id: ros2-architecture
title: ROS 2 Fundamentals - The Robotic Nervous System
sidebar_position: 2
description: Understanding ROS 2 architecture, communication patterns, and the computational graph for humanoid robotics.
keywords: [ROS 2, nodes, topics, services, actions, middleware, DDS, computational graph]
---

# Module 1A: ROS 2 Fundamentals — The Robotic Nervous System

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the role of ROS 2 as middleware for distributed robotic systems
2. Distinguish between nodes, topics, services, and actions with appropriate use cases
3. Diagram the computational graph for a humanoid robot system
4. Understand the Data Distribution Service (DDS) underlying ROS 2 communication
5. Apply ROS 2 communication patterns to sensor data flows and actuator commands
6. Identify when to use publish-subscribe (topics) vs request-reply (services) vs goal-based (actions) communication

---

## What is ROS 2?

**Robot Operating System 2 (ROS 2)** is an open-source middleware framework providing the communication infrastructure, tools, and libraries necessary to build complex robotic systems. Despite its name, ROS 2 is not an operating system in the traditional sense (like Linux or Windows); rather, it operates **above** the OS, providing abstractions that enable distributed computation across heterogeneous hardware components (Macenski et al., 2022).

### Why ROS 2 is Essential for Robotics

Modern robots are not monolithic systems but rather **distributed networks** of computational processes:

- **Perception modules** process camera images at 30 Hz
- **IMU drivers** publish orientation data at 100 Hz
- **Motor controllers** execute joint commands at 1000 Hz
- **Planning algorithms** compute paths every few seconds
- **LLM-based cognitive planners** may take 5-10 seconds for complex reasoning

These heterogeneous components operate at different frequencies, run on different hardware (CPU, GPU, embedded processors), and may be written in different programming languages (Python, C++, Rust). **ROS 2 provides the nervous system** that enables these distributed processes to communicate, coordinate, and function as a unified robotic intelligence.

### ROS 2 vs ROS 1: Key Improvements

ROS 2 (first released in 2017, LTS versions from 2020 onward) addresses critical limitations of the original ROS:

| Feature | ROS 1 | ROS 2 | Impact on Humanoids |
|---------|-------|-------|---------------------|
| **Communication** | Custom TCP/UDP protocol | DDS (Data Distribution Service) | Improved real-time performance, QoS control |
| **Real-Time Support** | Limited | Built-in (with RT Linux kernel) | Critical for balance control, motor coordination |
| **Multi-Robot** | Requires workarounds | Native support | Enables humanoid swarms, collaborative tasks |
| **Security** | Minimal | Authentication, encryption | Essential for deployed humanoid systems |
| **Platform Support** | Linux only | Linux, Windows, macOS, embedded | Broader development ecosystem |
| **Lifecycle Management** | Manual | Managed lifecycle nodes | Graceful startup/shutdown of complex systems |

For humanoid robotics—where real-time motor control, multi-sensor fusion, and safety-critical operation are paramount—ROS 2's improvements are not merely convenient but **essential** (Macenski et al., 2022).

---

## ROS 2 Architecture: The Computational Graph

At its core, ROS 2 organizes computation as a **directed graph** where:

- **Nodes** are vertices (computational processes)
- **Topics, Services, and Actions** are edges (communication channels)

This graph model provides flexibility: nodes can be added, removed, or reconfigured without recompiling the entire system. A humanoid robot might have dozens of nodes running concurrently—some on the robot's onboard computer, others on remote workstations for computationally intensive tasks like LLM inference.

### Nodes: The Computational Units

A **node** is an independent process that performs a specific function: reading a sensor, processing images, planning paths, or controlling motors. Nodes encapsulate functionality and communicate with other nodes exclusively through ROS 2's communication primitives (topics, services, actions).

**Design Principle**: Each node should have a **single, well-defined responsibility**. This modularity enables:

- **Independent Development**: Different teams can develop perception, planning, and control nodes concurrently
- **Reusability**: A camera driver node works identically whether on a humanoid, quadruped, or manipulator arm
- **Fault Isolation**: If the LLM planner node crashes, the robot can continue operating with degraded capability rather than total failure

**Example Nodes in a Humanoid System**:

- `camera_driver`: Publishes RGB images from head-mounted camera
- `imu_driver`: Publishes orientation and acceleration data from torso IMU
- `joint_state_publisher`: Publishes current positions of all 15 humanoid joints
- `balance_controller`: Computes Zero Moment Point to maintain upright posture
- `navigation_planner`: Plans collision-free paths using Nav2
- `llm_task_planner`: Decomposes voice commands into action sequences
- `manipulation_controller`: Executes arm trajectories for grasping

Each node runs as a separate process (potentially on different CPU cores or even different machines), communicating via ROS 2's middleware layer.

### Topics: Publish-Subscribe Communication

**Topics** implement the publish-subscribe pattern for **continuous, many-to-many data streams**. Nodes publish messages to named topics; other nodes subscribe to receive those messages. Publishers and subscribers are decoupled: neither knows how many or which nodes are on the other end.

**Use Cases for Topics**:
- Sensor data streams (cameras, LiDAR, IMU) - continuous, high-frequency
- Joint state updates (current positions, velocities, efforts)
- Odometry (robot pose and velocity estimates)
- Costmaps (obstacle information for navigation)

**Example: IMU Data Flow**

```
[IMU Hardware] → [imu_driver node] --publishes--> /imu/data (sensor_msgs/Imu)
                                                        ↓
                              [balance_controller] --subscribes--> processes orientation data
                              [state_estimator] --subscribes--> fuses with other sensors
                              [visualization (RViz)] --subscribes--> displays orientation
```

**Key Properties**:
- **Asynchronous**: Publishers send data without waiting for subscribers
- **Best-effort by default**: Messages may be dropped if network congested (configurable via QoS)
- **Type-safe**: Messages have defined schemas (e.g., `sensor_msgs/Imu` specifies accelerometer and gyroscope fields)
- **Many-to-many**: Multiple publishers and subscribers can share a topic

**Quality of Service (QoS)**: ROS 2 allows fine-grained control over reliability, durability, and latency through QoS policies. For humanoid balance control, you might use **reliable** QoS (guaranteed delivery) for joint commands but **best-effort** QoS (lower latency) for camera images where occasional frame drops are acceptable.

### Services: Request-Reply Communication

**Services** implement synchronous **request-reply** patterns for **one-time operations** where the client needs a response. The client sends a request and **blocks** until the server responds.

**Use Cases for Services**:
- Configuration queries ("What is the current controller gain?")
- One-time operations ("Reset the simulation", "Switch to standing mode")
- Stateless computations ("Compute inverse kinematics for this pose")

**Example: Inverse Kinematics Service**

```
[Manipulation Planner] --calls service--> /compute_ik (moveit_msgs/GetPositionIK)
                                                ↓
                         [IK Solver Service] processes request: desired end-effector pose
                                                ↓
                         Returns joint angles [q1, q2, ..., q7] for arm configuration
                                                ↓
      [Manipulation Planner] receives solution and sends joint commands
```

**Key Properties**:
- **Synchronous**: Client waits for server response (blocking call)
- **One-to-one**: Each request handled by a single server instance
- **Atomic**: Service calls either complete successfully or fail (no partial results)

**When to Use Services vs Topics**: If you need **continuous data** (sensor readings), use topics. If you need **one-time computation with a response** (IK calculation), use services. The distinction ensures appropriate communication patterns for different data flows.

### Actions: Goal-Based Communication

**Actions** extend services to support **long-running, goal-oriented tasks** with **feedback** and **preemption** (cancellation). Actions are ideal for behaviors that take significant time and benefit from progress updates.

**Use Cases for Actions**:
- Navigation ("Navigate to pose (x, y, θ)")
- Manipulation ("Grasp object at pose")
- Multi-step behaviors ("Execute pick-and-place sequence")

**Example: Navigation Action**

```
[Task Planner] --sends goal--> /navigate_to_pose (nav2_msgs/NavigateToPose)
                                        ↓
                  [Nav2 Controller] accepts goal and begins path execution
                                        ↓
                  --sends feedback--> current_pose, distance_remaining (every 0.5s)
                                        ↓
    [Task Planner] monitors progress, can cancel if user interrupts
                                        ↓
                  --sends result--> SUCCESS (reached goal) or FAILED (stuck, timeout)
```

**Key Properties**:
- **Asynchronous with feedback**: Client receives periodic progress updates
- **Preemptable**: Client can cancel goal mid-execution (robot stops gracefully)
- **Three-phase protocol**: Goal (sent once), Feedback (continuous), Result (sent once at completion)

**Why Actions Matter for Humanoids**: Bipedal navigation might take 30-60 seconds; grasping a cup might take 5-10 seconds. During these long-running tasks, the system needs **continuous feedback** (current position, gripper force) to monitor progress and **preemption capability** to abort safely if the environment changes or the user intervenes.

---

## The ROS 2 Communication Model: DDS

ROS 2's communication is built on the **Data Distribution Service (DDS)** standard, a mature middleware used in aerospace, defense, and industrial automation (Object Management Group, 2015). DDS provides:

1. **Discovery**: Nodes automatically discover each other on the network without centralized broker (unlike ROS 1's roscore)
2. **Quality of Service (QoS)**: Fine-grained control over reliability, latency, and bandwidth trade-offs
3. **Real-Time Capabilities**: Bounded latency and deterministic behavior (when paired with RT Linux kernel)
4. **Security**: Authentication, access control, and encryption (DDS-Security)

**Practical Impact**: When you launch a new camera node on a humanoid, all nodes subscribing to `/camera/image_raw` automatically discover and connect to it—no manual configuration required. If the camera node crashes and restarts, subscribers reconnect automatically. This **self-healing** property is critical for robust robotic systems.

---

## Humanoid Robot Data Flow Example

Let's examine how ROS 2 coordinates a humanoid robot maintaining balance while processing visual information:

### Scenario: Humanoid Standing on One Leg

**Perception Layer** (Topics, High Frequency):
```
[IMU Driver] --30 Hz--> /imu/data (orientation, angular velocity)
[Force Sensors] --100 Hz--> /left_foot/force, /right_foot/force
[Joint Encoders] --50 Hz--> /joint_states (all 15 joint positions/velocities)
```

**State Estimation** (Topic Processing):
```
[State Estimator Node]
  Subscribes to: /imu/data, /joint_states, /left_foot/force, /right_foot/force
  Processes: Fuses sensor data, estimates Center of Mass (CoM), computes Zero Moment Point (ZMP)
  Publishes to: /robot/state (custom state estimate)
```

**Control Layer** (Topic to Action):
```
[Balance Controller Node]
  Subscribes to: /robot/state
  Computes: Joint torque commands to keep ZMP within support polygon
  Publishes to: /joint_commands (target positions for each joint)

[Joint Controller]
  Subscribes to: /joint_commands
  Sends: Low-level motor commands to actuators (via hardware interface)
```

**High-Level Planning** (Service):
```
User requests via service: /set_balance_mode (service)
  Request: "STAND_ON_LEFT_LEG"
  Response: SUCCESS (controller transitions to single-leg support mode)
```

**Navigation Integration** (Action):
```
[Navigation Planner] --sends action goal--> /navigate_to_pose
  Goal: Move to position (x: 2.0, y: 1.0) while maintaining balance
  Feedback: Current pose, distance remaining (updated every 0.5s)
  Result: REACHED_GOAL or STUCK (after 60 seconds)
```

This example demonstrates **multi-timescale coordination**: force sensors publish at 100 Hz for rapid balance adjustments, while navigation goals operate at timescales of tens of seconds. ROS 2's topic/service/action model enables this heterogeneity while maintaining system coherence.

---

## The Computational Graph Visualized

Consider a simplified humanoid robot with the following nodes:

**Perception Nodes**:
- `camera_driver` → publishes to `/camera/image_raw`
- `lidar_driver` → publishes to `/scan`
- `imu_driver` → publishes to `/imu/data`

**Processing Nodes**:
- `object_detector` → subscribes to `/camera/image_raw`, publishes to `/detected_objects`
- `slam_node` → subscribes to `/scan` and `/imu/data`, publishes to `/map` and TF transforms

**Control Nodes**:
- `balance_controller` → subscribes to `/imu/data` and `/joint_states`, publishes to `/joint_commands`
- `navigation_controller` → action server for `/navigate_to_pose`

**Textual Representation of Computational Graph**:

```
         Camera → /camera/image_raw → Object Detector → /detected_objects
                                                               ↓
         LiDAR → /scan ────────┐                     Navigation Planner
                               ├──→ SLAM Node → /map         ↓
         IMU → /imu/data ──────┤         ↓              /navigate_to_pose (action)
                               │        /tf                   ↓
                               └──→ Balance Controller → /joint_commands
                                        ↑                      ↓
         Joint Encoders → /joint_states ┘             Joint Controllers → Motors
```

**Interpretation**: Sensor data (camera, LiDAR, IMU, joint states) flows into processing nodes (object detection, SLAM, balance control) which produce higher-level outputs (detected objects, maps, motor commands). The navigation controller coordinates these via actions to achieve goal-directed behaviors.

---

## Communication Pattern Selection Guide

### When to Use Topics (Publish-Subscribe)

✅ **Use topics when**:
- Data is **continuous** and **streaming** (sensor readings, robot state)
- You need **many-to-many** communication (multiple subscribers)
- **Latency is critical** over guaranteed delivery
- Data has a natural **update rate** (e.g., 30 Hz for cameras)

**Examples**:
- `/camera/image_raw` (sensor_msgs/Image) - Camera images at 30 Hz
- `/joint_states` (sensor_msgs/JointState) - Joint positions/velocities at 50 Hz
- `/odom` (nav_msgs/Odometry) - Robot pose estimates at 50 Hz

### When to Use Services (Request-Reply)

✅ **Use services when**:
- Operation is **one-time** or **infrequent**
- You need a **response** before proceeding
- Operation is **stateless** (same input always gives same output)
- **Computation is fast** (less than 100ms typical)

**Examples**:
- `/compute_ik` (moveit_msgs/GetPositionIK) - Compute joint angles for target pose
- `/reset_simulation` (std_srvs/Empty) - Reset robot to initial state
- `/get_planning_scene` (moveit_msgs/GetPlanningScene) - Query current environment obstacles

### When to Use Actions (Goal-Based)

✅ **Use actions when**:
- Task is **long-running** (seconds to minutes)
- You need **progress feedback** during execution
- Task can be **canceled or preempted** (user changes mind)
- Failure modes are important to distinguish (stuck, timeout, collision)

**Examples**:
- `/navigate_to_pose` (nav2_msgs/NavigateToPose) - Navigate to goal (10-60 seconds)
- `/grasp_object` (custom action) - Approach and grasp object (5-15 seconds)
- `/follow_trajectory` (control_msgs/FollowJointTrajectory) - Execute arm motion (2-10 seconds)

**Rule of Thumb**: If it's data → **topic**. If it's a quick query → **service**. If it's a goal you monitor and might cancel → **action**.

---

## ROS 2 Under the Hood: DDS and QoS

ROS 2's communication layer is built on the **Data Distribution Service (DDS)** standard, which provides publish-subscribe infrastructure with rich Quality of Service (QoS) policies.

### Quality of Service (QoS) Policies

QoS policies allow you to trade off **reliability** vs **latency** vs **bandwidth**:

**Key QoS Parameters**:

1. **Reliability**:
   - **Reliable**: Guaranteed delivery (messages retransmitted if lost) - Use for motor commands
   - **Best Effort**: Fire-and-forget (occasional drops acceptable) - Use for high-rate sensor data

2. **Durability**:
   - **Transient Local**: New subscribers receive last published message (useful for robot state)
   - **Volatile**: Subscribers only receive messages published after subscription

3. **History**:
   - **Keep Last (N)**: Buffer only the last N messages (typical for sensors)
   - **Keep All**: Buffer all messages until delivered (for critical commands)

4. **Deadline**: Publisher guarantees message rate (e.g., "Camera must publish at ≥20 Hz")

**Practical Example**: For `/joint_commands`, you would use:
- **Reliable** (motor commands must not be dropped)
- **Transient Local** (if controller restarts, it gets the last command)
- **Keep Last 1** (only the most recent command matters)

For `/camera/image_raw`, you might use:
- **Best Effort** (occasional frame drops acceptable for lower latency)
- **Volatile** (old images not useful)
- **Keep Last 3** (small buffer for transient network delays)

### Discovery and Lifecycle Management

**Automatic Discovery**: When a node starts, it announces its publishers, subscribers, services, and actions via DDS discovery protocol. All other nodes automatically learn about it—no manual configuration required.

**Lifecycle Nodes**: ROS 2 introduces **managed lifecycle nodes** with explicit states:
1. **Unconfigured**: Node loaded but not initialized
2. **Inactive**: Node configured but not processing data
3. **Active**: Node fully operational
4. **Finalized**: Node shutting down gracefully

For humanoid systems with multiple coordinated nodes (e.g., ensure balance controller is active before navigation starts), lifecycle management ensures **graceful startup and shutdown sequences**.

---

## Practical Implications for Humanoid Robotics

### Multi-Rate Coordination

Humanoid robots exhibit **hierarchical control** with different timescales:

- **Low-level control** (1000 Hz): Joint PID loops maintaining target positions
- **Mid-level control** (50-100 Hz): Balance controller computing ZMP corrections
- **High-level planning** (1-10 Hz): Path planning, grasp planning, task sequencing
- **Cognitive reasoning** (0.1-1 Hz): LLM-based task decomposition

ROS 2's topic system enables this by allowing nodes to operate at their natural frequencies while sharing data asynchronously. The balance controller doesn't wait for the LLM planner—it continuously stabilizes the robot regardless of high-level decision-making latency.

### Distributed Computation

Computationally expensive tasks (LLM inference, object detection with deep neural networks) can run on **remote workstations with GPUs**, while low-latency control runs on the **onboard robot computer**. ROS 2's network transparency makes this seamless—nodes communicate identically whether on the same machine or across a network.

**Example Distribution**:
- **Onboard Computer** (robot CPU): IMU driver, joint controllers, balance controller
- **Remote Workstation** (GPU): Object detection (YOLO), LLM planner (GPT-4/Qwen), visual SLAM

DDS automatically routes messages between machines. The balance controller doesn't need to know whether the navigation planner is local or remote—it simply subscribes to `/navigation_commands` and receives data transparently.

### Fault Tolerance

When a node crashes, ROS 2's decoupled architecture prevents cascading failures:

- If the camera driver crashes, object detection fails gracefully but balance control continues
- If the LLM planner fails, the robot can continue executing pre-planned behaviors
- If network connectivity is lost, local nodes continue operating with last known state

This **graceful degradation** is essential for humanoid robots operating in unstructured environments where component failures are inevitable.

---

## Comparison: Humanoid Control in ROS 2 vs Traditional Approach

### Without ROS 2 (Monolithic System)

A humanoid robot programmed traditionally might have:
- A single main loop reading all sensors, computing all control outputs, and sending all motor commands
- Tight coupling between perception, planning, and control (changing one requires understanding all)
- Difficult to parallelize (single-threaded bottleneck)
- Hard to test components in isolation (need full robot)

**Code structure**:
```python
while True:
    imu_data = read_imu()
    camera_image = read_camera()
    joint_states = read_joints()

    # All processing in one loop - tightly coupled
    detected_objects = detect_objects(camera_image)
    balance_correction = compute_balance(imu_data, joint_states)
    navigation_command = plan_path(detected_objects)

    send_joint_commands(balance_correction + navigation_command)
    # What happens if object detection takes 500ms? Balance control delayed!
```

### With ROS 2 (Distributed System)

The same robot with ROS 2:
- Each subsystem (perception, planning, control) is an **independent node**
- Communication via well-defined topics/services/actions
- Components run at appropriate frequencies (balance at 100 Hz, planning at 1 Hz)
- Easy to test nodes individually (mock sensor data via topic publishing)

**Architecture**:
```
Perception Nodes (independent processes):
  camera_driver.py → publishes at 30 Hz
  imu_driver.py → publishes at 100 Hz

Processing Nodes:
  object_detector.py → subscribes to camera, publishes detections at 10 Hz

Control Nodes:
  balance_controller.py → subscribes to IMU at 100 Hz, computes corrections, publishes commands

Planning Nodes:
  navigation_planner.py → action server, computes paths at 1 Hz, integrates with balance controller
```

Each node operates independently at its natural frequency. The balance controller maintains stability at 100 Hz even if object detection temporarily stalls. This **temporal decoupling** is ROS 2's core strength.

---

## Getting Started: Your First ROS 2 System

In the next chapter, we'll transition from concepts to practice:

- **Setting up a ROS 2 workspace** with proper package structure
- **Writing your first nodes** in Python using `rclpy`
- **Creating publishers and subscribers** for sensor simulation
- **Building a simple controller** that commands humanoid joints
- **Visualizing the computational graph** using ROS 2 introspection tools

By Module 1's end, you'll have a functioning ROS 2 system with a simulated humanoid robot responding to commands—your first embodied AI agent.

---

## Key Takeaways

1. **ROS 2 is middleware**, not an OS—it provides communication infrastructure above the operating system
2. **Topics** (publish-subscribe) for continuous data streams, **services** (request-reply) for one-time operations, **actions** (goal-based) for long-running tasks
3. **DDS** provides automatic discovery, QoS control, and real-time capabilities
4. **Computational graphs** enable distributed, modular robotics systems
5. **Decoupling** allows nodes to operate at different frequencies and fail independently
6. **For humanoids**, ROS 2 enables multi-rate control (1000 Hz motor control + 1 Hz cognitive planning) with graceful degradation

Understanding these fundamentals is essential before proceeding to implementation. Next, we'll apply this knowledge to build a working ROS 2 system for a humanoid robot.

---

## References

Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074

Object Management Group (2015). *Data Distribution Service (DDS) Version 1.4*. OMG Document formal/2015-04-10. https://www.omg.org/spec/DDS/1.4

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

ROS 2 Design Documentation (2023). *ROS 2 Design Principles*. Open Robotics. https://design.ros2.org/

REP 103: Standard Units of Measure and Coordinate Conventions (2010). https://www.ros.org/reps/rep-0103.html
