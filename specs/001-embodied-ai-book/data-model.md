# Data Model & Book Structure

**Project**: Embodied AI & Humanoid Robotics Book
**Branch**: `001-embodied-ai-book`
**Date**: 2025-12-19
**Phase**: Phase 1 - Data Models & Contracts

## Book Structure Model

### Module Organization

**Total Modules**: 5 core modules + Introduction + Appendices

| Module ID | Title | Word Count Target | Chapters | Code Examples | Diagrams | Min References |
|-----------|-------|-------------------|----------|---------------|----------|----------------|
| **Intro** | Introduction to Physical AI & Embodied Intelligence | 1,000-1,500 | 1 | 0 | 2 | 3 |
| **M1** | ROS 2 as the Robotic Nervous System | 2,000-3,000 | 4 | 10-15 | 4 | 5 |
| **M2** | Digital Twin Simulation (Gazebo + Unity) | 2,000-3,000 | 4 | 10-15 | 5 | 5 |
| **M3** | NVIDIA Isaac AI Robotics Platform | 2,500-3,500 | 5 | 12-18 | 6 | 5 |
| **M4** | Vision-Language-Action Systems | 3,000-4,000 | 5 | 15-20 | 6 | 7 |
| **M5** | Capstone: End-to-End Autonomous Humanoid | 3,500-5,000 | 6 | 20-25 | 8 | 5 |
| **Appendices** | Python Refresher, Linear Algebra, Hardware Guide, Glossary | 1,500-2,000 | 4 | 5-10 | 2 | 5 |
| **TOTAL** | | **13,500-21,000** | **29** | **72-103** | **33** | **35** |

**Note**: Total word count target is 10,000-15,000 excluding code blocks and appendices. Midpoint estimate: ~14,000 words.

### Chapter Breakdown

#### Introduction (Intro)
1. **intro.md** - What is Physical AI? Why Humanoid Robotics? Book Structure

#### Module 1: ROS 2 Robotic Nervous System
1. **index.md** - Module overview and learning objectives
2. **ros2-architecture.md** - Nodes, topics, services, actions, computational graph
3. **rclpy-development.md** - Python development with rclpy, workspace setup
4. **urdf-modeling.md** - URDF for humanoid robots, joint/link definitions
5. **exercises.md** - Hands-on exercises and validation checkpoints

#### Module 2: Digital Twin Simulation
1. **index.md** - Module overview, digital twin concepts
2. **gazebo-simulation.md** - Gazebo fundamentals, world files, SDF format
3. **unity-integration.md** - Unity visualization, ROS-TCP-Connector setup
4. **sensor-simulation.md** - LiDAR, RGB-D cameras, IMU simulation
5. **exercises.md** - Comparative exercises (Gazebo vs Unity)

#### Module 3: NVIDIA Isaac AI Platform
1. **index.md** - Module overview, Isaac ecosystem introduction
2. **isaac-sim.md** - Isaac Sim setup, USD scenes, photorealistic rendering
3. **isaac-ros.md** - Hardware-accelerated perception (nvblox, cuVSLAM, DOPE)
4. **nav2-integration.md** - Nav2 for humanoid navigation, path planning
5. **rl-locomotion.md** - Reinforcement learning with Isaac Gym, PPO training
6. **exercises.md** - Perception and navigation validation

#### Module 4: Vision-Language-Action
1. **index.md** - Module overview, VLA architecture introduction
2. **whisper-integration.md** - Speech recognition with Whisper, ROS 2 integration
3. **llm-planning.md** - Cognitive planning with GPT-4/Qwen, prompt engineering
4. **vision-language.md** - CLIP, LLaVA for object grounding and scene understanding
5. **action-grounding.md** - Translating plans to ROS 2 actions, safety validation
6. **exercises.md** - End-to-end VLA pipeline testing

#### Module 5: Capstone
1. **index.md** - Capstone overview, system scenario
2. **architecture.md** - System design, node graph, data flow
3. **integration.md** - Module integration strategies, interface definitions
4. **state-machine.md** - Orchestrator design, state transitions
5. **debugging.md** - Troubleshooting guide, common integration issues
6. **demo-guide.md** - Demo preparation, presentation guidelines
7. **exercises.md** - Final validation and assessment rubric

#### Appendices
1. **python-refresher.md** - Python basics, virtual environments, package management
2. **linear-algebra.md** - Vectors, matrices, transformations, quaternions for robotics
3. **hardware-guide.md** - Hardware recommendations, setup guides, budget options
4. **glossary.md** - Technical terminology and acronyms

#### References
1. **references.md** - Complete bibliography (APA 7th edition, 35+ sources)

---

## Humanoid Robot URDF Model Specification

### Model Name: `simple_humanoid`

**Purpose**: Educational humanoid model used across all modules for consistency.

### Joint Hierarchy (Kinematic Tree)

```
base_link (torso)
├── neck_yaw → head_link
│   └── camera_link (fixed)
├── left_shoulder_pitch → left_upper_arm_link
│   └── left_elbow_pitch → left_forearm_link
│       └── left_wrist_pitch → left_hand_link
├── right_shoulder_pitch → right_upper_arm_link
│   └── right_elbow_pitch → right_forearm_link
│       └── right_wrist_pitch → right_hand_link
├── left_hip_yaw → left_thigh_link
│   └── left_knee_pitch → left_shin_link
│       └── left_ankle_pitch → left_foot_link
└── right_hip_yaw → right_thigh_link
    └── right_knee_pitch → right_shin_link
        └── right_ankle_pitch → right_foot_link
```

**Total Degrees of Freedom (DOF)**: 15
- Head: 1 DOF (yaw)
- Arms: 3 DOF × 2 = 6 DOF (shoulder pitch, elbow pitch, wrist pitch per arm)
- Legs: 3 DOF × 2 = 6 DOF (hip yaw, knee pitch, ankle pitch per leg)
- Hands: 2 fixed grippers (simplified, no finger articulation)

### Link Properties

**Sample: Torso (base_link)**
```xml
<link name="base_link">
  <inertial>
    <mass value="15.0"/>  <!-- kg -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- CoM at torso center -->
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>  <!-- 30cm × 20cm × 50cm torso -->
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

### Sensor Configuration

**Sensors Attached**:
1. **RGB Camera** (head-mounted):
   - Type: `camera` plugin
   - Resolution: 640×480
   - FOV: 80 degrees
   - Frame: `camera_link`
   - ROS 2 Topic: `/camera/image_raw` (sensor_msgs/Image)

2. **Depth Camera** (head-mounted):
   - Type: `depth_camera` plugin
   - Resolution: 640×480
   - Range: 0.3m - 10m
   - ROS 2 Topic: `/camera/depth` (sensor_msgs/Image)

3. **LiDAR** (torso-mounted):
   - Type: `ray` plugin (2D or 3D)
   - Range: 0.1m - 30m
   - Scan rate: 10 Hz
   - ROS 2 Topic: `/scan` (sensor_msgs/LaserScan)

4. **IMU** (torso center):
   - Type: `imu` plugin
   - Outputs: Linear acceleration, angular velocity
   - ROS 2 Topic: `/imu/data` (sensor_msgs/Imu)

5. **Force-Torque Sensors** (optional, feet):
   - Type: `force_torque` plugin
   - Location: Ankle joints
   - ROS 2 Topic: `/left_foot/ft`, `/right_foot/ft`

### Actuator Configuration

**Joint Control**:
- **Type**: Position control (default for simulation)
- **Effort Limits**: ~100 N·m for legs, ~50 N·m for arms, ~20 N·m for wrists
- **Velocity Limits**: ~2 rad/s (safe simulation speeds)
- **Control Interface**: ROS 2 topics `/joint_commands` (sensor_msgs/JointState)

---

## ROS 2 Message Schema

### Standard ROS 2 Messages Used

**Sensor Data**:
- `sensor_msgs/Image` - RGB and depth camera images
- `sensor_msgs/LaserScan` - 2D LiDAR scans
- `sensor_msgs/PointCloud2` - 3D LiDAR point clouds
- `sensor_msgs/Imu` - IMU data (accelerometer, gyroscope)
- `sensor_msgs/JointState` - Joint positions, velocities, efforts

**Navigation**:
- `nav_msgs/Odometry` - Robot pose and velocity
- `nav_msgs/OccupancyGrid` - 2D costmaps
- `nav_msgs/Path` - Planned paths from Nav2
- `geometry_msgs/PoseStamped` - Goal poses
- `geometry_msgs/Twist` - Velocity commands

**Perception**:
- `vision_msgs/Detection3DArray` - 3D object detections with bounding boxes
- `vision_msgs/ObjectHypothesisWithPose` - Object pose estimates
- `tf2_msgs/TFMessage` - Transform tree updates

**Custom Messages** (for VLA system):

**TaskPlan.msg**:
```
# High-level task plan from LLM
string task_id
string task_description
TaskStep[] steps
```

**TaskStep.msg**:
```
# Individual action in a task plan
string action_type  # "navigate_to", "detect_object", "grasp_object", "place_object"
string target_object
geometry_msgs/Pose target_pose
float32 timeout
```

**TaskStatus.msg**:
```
# Task execution status
string task_id
string current_state  # "IDLE", "PLANNING", "NAVIGATING", "DETECTING", "GRASPING", etc.
float32 progress  # 0.0 to 1.0
string status_message
bool success
```

---

## Code Example Catalog

### Module 1: ROS 2 Examples (10-15 examples)

1. **hello_publisher** - Basic ROS 2 publisher node
2. **hello_subscriber** - Basic ROS 2 subscriber node
3. **joint_state_publisher** - Publish humanoid joint states
4. **sensor_simulator** - Simulate IMU/camera data
5. **service_demo** - ROS 2 service client/server
6. **action_demo** - ROS 2 action client/server
7. **parameter_demo** - Dynamic parameter configuration
8. **urdf_loader** - Load and visualize URDF in RViz
9. **simple_controller** - Basic joint position controller
10. **launch_demo** - Comprehensive launch file example

### Module 2: Digital Twin Examples (10-15 examples)

1. **gazebo_world_basic** - Simple Gazebo world with humanoid
2. **gazebo_sensors** - LiDAR, camera, IMU simulation in Gazebo
3. **urdf_to_sdf** - Conversion script and validation
4. **unity_scene_basic** - Unity project with humanoid import
5. **ros_tcp_connector_setup** - Unity-ROS 2 bridge configuration
6. **physics_comparison** - Side-by-side Gazebo vs Unity physics test
7. **sensor_visualization** - RViz visualization of simulated sensors
8. **collision_testing** - Collision detection validation
9. **balance_test** - Apply forces and test humanoid stability
10. **performance_benchmark** - FPS and physics timestep measurement

### Module 3: Isaac Examples (12-18 examples)

1. **isaac_sim_hello_world** - Basic Isaac Sim scene with robot
2. **isaac_urdf_import** - Import humanoid URDF into Isaac Sim
3. **synthetic_data_gen** - Domain randomization and dataset export
4. **isaac_ros_vslam** - Launch nvblox or cuVSLAM
5. **isaac_ros_object_detection** - DOPE or CenterPose object detection
6. **nav2_config_humanoid** - Nav2 parameter files for bipedal robot
7. **nav2_launch** - Launch Nav2 stack with humanoid
8. **rl_env_setup** - Isaac Gym environment for locomotion
9. **ppo_training** - Train bipedal walking policy with PPO
10. **policy_evaluation** - Test trained policy in Isaac Sim
11. **sim2real_checklist** - Validation steps for reality gap assessment
12. **isaac_performance_tuning** - GPU optimization and multi-sensor fusion

### Module 4: VLA Examples (15-20 examples)

1. **whisper_ros2_node** - Whisper ASR integrated with ROS 2
2. **llm_planner_node** - LLM API client publishing task plans
3. **prompt_templates** - Robotics-specific prompt engineering examples
4. **action_grounding** - Parse JSON plan → ROS 2 action goals
5. **clip_object_detection** - CLIP zero-shot object recognition
6. **llava_scene_understanding** - LLaVA visual question answering
7. **safety_validator** - Filter and validate LLM-generated commands
8. **task_status_reporter** - Publish and visualize task execution status
9. **multi_modal_fusion** - Combine vision, language, and proprioception
10. **gpt4_integration** - OpenAI API integration example
11. **qwen_local_integration** - Local Qwen model deployment
12. **object_grounding_3d** - Language → 3D object localization
13. **conversation_manager** - Handle multi-turn interactions
14. **error_recovery** - Handle perception and planning failures
15. **cost_optimization** - Token usage and API cost reduction strategies

### Module 5: Capstone Examples (20-25 examples)

1. **capstone_workspace** - Complete ROS 2 workspace structure
2. **system_launch** - Master launch file orchestrating all nodes
3. **state_machine_orchestrator** - SMACH or BT-based coordination
4. **voice_interface_node** - Complete Whisper integration
5. **cognitive_planner_node** - LLM planner with scene context
6. **perception_pipeline_node** - Multi-sensor fusion and object detection
7. **navigation_client_node** - Nav2 action client wrapper
8. **manipulation_server_node** - Grasp planning and execution server
9. **feedback_publisher_node** - Status reporting and user communication
10. **system_config** - All parameter files and configurations
11. **unit_test_suite** - pytest tests for individual nodes
12. **integration_test_suite** - Test subsystem interactions
13. **system_test** - Full end-to-end pipeline test
14. **demo_scenario_1** - "Fetch object from table" scenario
15. **demo_scenario_2** - "Bring object to user" scenario
16. **demo_scenario_3** - "Rearrange objects" scenario
17. **performance_profiler** - Latency and resource usage measurement
18. **docker_environment** - Complete Docker setup for reproducibility
19. **debugging_tools** - Custom debugging utilities and visualizations
20. **deployment_guide** - Step-by-step deployment instructions

---

## Diagram Specifications

### System Architecture Diagrams (8 diagrams)

1. **ROS 2 Computational Graph** (Module 1):
   - Nodes: publisher_node, subscriber_node, service_server, action_server
   - Topics: /sensor_data, /joint_commands
   - Services: /get_config, /reset_sim
   - Actions: /navigate_to, /grasp_object
   - Tool: Mermaid or draw.io → SVG export

2. **Humanoid URDF Structure** (Module 1):
   - Kinematic tree showing joint hierarchy
   - Sensor attachment points
   - Tool: URDF visualizer or custom Python script

3. **Digital Twin Pipeline** (Module 2):
   - Data flow: URDF → Gazebo (physics) ⇄ Unity (visualization) → ROS 2 topics
   - Tool: Mermaid flowchart

4. **Gazebo vs Unity Comparison** (Module 2):
   - Side-by-side screenshots showing same robot in both platforms
   - Tool: Screenshots + annotation

5. **Sensor Simulation Architecture** (Module 2):
   - LiDAR ray-casting, camera rendering, IMU simulation
   - Tool: Block diagram (Mermaid or draw.io)

6. **Isaac Perception Pipeline** (Module 3):
   - Sensor → Isaac ROS (GPU) → VSLAM/Detection → Nav2/Manipulation
   - Tool: Mermaid flowchart with GPU acceleration highlights

7. **Nav2 Architecture for Humanoid** (Module 3):
   - Global planner, local planner, costmaps, recovery behaviors
   - Bipedal-specific modifications highlighted
   - Tool: Nav2 documentation diagram adapted for humanoid

8. **VLA System Architecture** (Module 4):
   - Speech → Whisper → LLM → Action Grounding → ROS 2 Actions
   - Perception feedback loop
   - Tool: Detailed Mermaid diagram

9. **Vision-Language Grounding** (Module 4):
   - Image + query → CLIP/LLaVA → object localization → 3D coordinates
   - Tool: Workflow diagram with example inputs/outputs

10. **Capstone System Diagram** (Module 5):
    - Complete node graph showing all 7 nodes and their connections
    - Tool: ROS 2 rqt_graph export or custom diagram

11. **Capstone Data Flow** (Module 5):
    - End-to-end data flow from voice input to action completion
    - Tool: Sequence diagram (Mermaid)

12. **Capstone State Machine** (Module 5):
    - States: IDLE → LISTENING → PLANNING → NAVIGATING → DETECTING → GRASPING → PLACING → REPORTING
    - Transitions and error handling
    - Tool: State diagram (Mermaid)

---

## Code Repository Structure (Companion Repo)

### Directory Conventions

**Module Pattern**:
```
module<N>-<name>/
├── examples/
│   ├── example1/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── <package_name>/
│   │   │   ├── __init__.py
│   │   │   └── <node_name>.py
│   │   ├── launch/
│   │   │   └── <name>_launch.py
│   │   └── README.md
│   └── example2/...
├── exercises/
│   ├── exercise1/
│   │   ├── instructions.md
│   │   ├── starter_code/
│   │   └── solution/
│   └── exercise2/...
└── README.md
```

**Testing Structure**:
```
tests/
├── module1/
│   ├── test_ros2_basics.py
│   └── test_urdf_validation.py
├── module2/
│   ├── test_gazebo_launch.py
│   └── test_sensor_simulation.py
├── module3/
│   ├── test_isaac_sim_setup.py
│   └── test_nav2_integration.py
├── module4/
│   ├── test_whisper_integration.py
│   └── test_llm_action_grounding.py
└── capstone/
    ├── test_system_integration.py
    └── test_end_to_end.py
```

**Docker Environments**:
```
docker/
├── Dockerfile.ros2          # ROS 2 Humble + Gazebo + basic dependencies
├── Dockerfile.isaac         # Isaac Sim + Isaac ROS + Nav2
├── Dockerfile.development   # Full development environment with all tools
├── docker-compose.yml       # Multi-container orchestration
└── README.md                # Docker usage instructions
```

---

## Validation Criteria per Module

### Module 1 Validation Checklist
- [ ] All ROS 2 code examples execute without errors on Ubuntu 22.04 + Humble
- [ ] URDF model loads in RViz and Gazebo without warnings
- [ ] Computational graph diagrams accurately represent node communication
- [ ] Minimum 5 ROS 2 authoritative sources cited
- [ ] Learning objectives tested via exercises

### Module 2 Validation Checklist
- [ ] Gazebo world launches with humanoid robot
- [ ] Unity scene loads with imported URDF
- [ ] Sensor data publishes to ROS 2 topics in both platforms
- [ ] Comparison table completed with quantitative metrics
- [ ] Minimum 5 simulation sources cited

### Module 3 Validation Checklist
- [ ] Isaac Sim scene loads with humanoid and sensors
- [ ] Isaac ROS perception packages launch successfully
- [ ] Nav2 generates valid paths for bipedal robot
- [ ] RL training completes (even if policy not optimal)
- [ ] Minimum 5 Isaac/perception sources cited

### Module 4 Validation Checklist
- [ ] Whisper transcribes speech and publishes to ROS 2 topic
- [ ] LLM generates valid action plans from natural language
- [ ] Action grounding converts plans to ROS 2 actions
- [ ] Vision-language model grounds objects in 3D space
- [ ] Safety validator rejects invalid commands
- [ ] Minimum 7 VLA/LLM sources cited

### Module 5 Validation Checklist
- [ ] Complete system launches with all nodes communicating
- [ ] State machine transitions through all states successfully
- [ ] End-to-end voice command executes task (>70% success rate)
- [ ] Performance metrics documented (latencies, success rates)
- [ ] Video demonstration recorded and embedded
- [ ] Debugging guide covers common integration issues

---

## Next Steps

Phase 1 deliverables to create:
1. ✅ **data-model.md** - This file
2. ⏳ **contracts/module-interfaces.md** - Module dependency contracts
3. ⏳ **contracts/ros2-message-contracts.md** - ROS 2 message specifications
4. ⏳ **quickstart.md** - Reader environment setup guide

After Phase 1 completion, proceed to `/sp.tasks` for actionable task generation.
