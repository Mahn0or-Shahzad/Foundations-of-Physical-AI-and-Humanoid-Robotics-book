# ROS 2 Message Contracts

**Project**: Embodied AI & Humanoid Robotics Book
**Branch**: `001-embodied-ai-book`
**Date**: 2025-12-19
**Phase**: Phase 1 - Contracts

## Standard ROS 2 Messages

### Sensor Data Topics

**Camera Image Topic**: `/camera/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **Frame ID**: `camera_link`
- **Encoding**: `rgb8` or `bgr8`
- **Frequency**: 30 Hz
- **Resolution**: 640×480 (configurable)

**Depth Camera Topic**: `/camera/depth`
- **Message Type**: `sensor_msgs/Image`
- **Frame ID**: `camera_link`
- **Encoding**: `32FC1` (float32 depth in meters)
- **Frequency**: 30 Hz
- **Range**: 0.3m - 10m

**LiDAR Scan Topic**: `/scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **Frame ID**: `lidar_link`
- **Frequency**: 10 Hz
- **Range**: 0.1m - 30m
- **Angle**: -180° to +180° (360° scan)

**IMU Data Topic**: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Frame ID**: `imu_link`
- **Frequency**: 100 Hz
- **Data**: Linear acceleration (m/s²), angular velocity (rad/s), orientation (quaternion)

**Joint States Topic**: `/joint_states`
- **Message Type**: `sensor_msgs/JointState`
- **Frequency**: 50 Hz
- **Data**: Joint positions (rad), velocities (rad/s), efforts (N·m)

### Navigation Topics

**Odometry Topic**: `/odom`
- **Message Type**: `nav_msgs/Odometry`
- **Frame ID**: `odom` (child: `base_link`)
- **Frequency**: 50 Hz
- **Data**: Pose (position + orientation), twist (linear + angular velocity)

**AMCL Pose Topic**: `/amcl_pose`
- **Message Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Frame ID**: `map`
- **Frequency**: Event-driven (~1-10 Hz)
- **Data**: Localized pose with covariance

**Global Costmap Topic**: `/global_costmap/costmap`
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Frame ID**: `map`
- **Resolution**: 0.05m (5cm per cell)
- **Update Frequency**: 1 Hz

**Local Costmap Topic**: `/local_costmap/costmap`
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Frame ID**: `odom`
- **Resolution**: 0.05m
- **Update Frequency**: 5 Hz

**Planned Path Topic**: `/plan`
- **Message Type**: `nav_msgs/Path`
- **Frame ID**: `map`
- **Published by**: Nav2 global planner

---

## Custom Messages for VLA System

### Speech Command (String Topic)

**Topic Name**: `/speech_command`
**Message Type**: `std_msgs/String`
**Publisher**: Whisper ASR node
**Subscriber**: LLM Planner node
**Example Data**: `"go to the table and pick up the red box"`

### Task Plan (Custom Message)

**Topic Name**: `/task_plan`
**Message Type**: `task_msgs/TaskPlan` (custom)
**Publisher**: LLM Planner node
**Subscriber**: Orchestrator node

**Message Definition** (`TaskPlan.msg`):
```
# TaskPlan.msg
std_msgs/Header header
string task_id          # Unique identifier (UUID)
string task_description # Natural language description
TaskStep[] steps        # Sequence of actions
float32 estimated_duration  # Seconds
```

**Message Definition** (`TaskStep.msg`):
```
# TaskStep.msg
string step_id          # Unique step identifier
string action_type      # "navigate_to", "detect_object", "grasp_object", "place_object", "report_status"
string target_object    # Object name or ID
geometry_msgs/Pose target_pose  # Target pose for navigation or placement
string[] parameters     # Additional action-specific parameters (JSON string)
float32 timeout         # Maximum time for this step (seconds)
```

**Example JSON Representation**:
```json
{
  "header": {"stamp": {"sec": 1234567890, "nanosec": 0}, "frame_id": "map"},
  "task_id": "abc123",
  "task_description": "Pick up red box from table and place on shelf",
  "steps": [
    {
      "step_id": "step1",
      "action_type": "navigate_to",
      "target_object": "table",
      "target_pose": {"position": {"x": 2.0, "y": 1.0, "z": 0.0}, "orientation": {...}},
      "parameters": [],
      "timeout": 30.0
    },
    {
      "step_id": "step2",
      "action_type": "detect_object",
      "target_object": "red box",
      "target_pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {...}},
      "parameters": ["color:red", "shape:box"],
      "timeout": 10.0
    },
    {
      "step_id": "step3",
      "action_type": "grasp_object",
      "target_object": "red box",
      "target_pose": {"position": {"x": 2.0, "y": 1.0, "z": 0.8}, "orientation": {...}},
      "parameters": ["grasp_type:top_grasp"],
      "timeout": 15.0
    }
  ],
  "estimated_duration": 60.0
}
```

### Detected Objects (Vision Messages)

**Topic Name**: `/detected_objects`
**Message Type**: `vision_msgs/Detection3DArray` (standard vision_msgs)
**Publisher**: Perception node (YOLO/DOPE/Isaac ROS)
**Subscriber**: Manipulation planner, LLM planner (for scene context)

**Message Structure**:
```
std_msgs/Header header
vision_msgs/Detection3D[] detections

# Each Detection3D contains:
# - BoundingBox3D: center, size (x, y, z)
# - ObjectHypothesisWithPose[]: class_id, score, pose
```

**Example**:
```yaml
detections:
  - bbox:
      center: {position: {x: 2.0, y: 1.0, z: 0.8}}
      size: {x: 0.1, y: 0.1, z: 0.15}  # Red box dimensions
    results:
      - hypothesis:
          class_id: "red_box"
          score: 0.92
        pose:
          pose: {position: {x: 2.0, y: 1.0, z: 0.8}, orientation: {x: 0, y: 0, z: 0, w: 1}}
```

### Task Status (Custom Message)

**Topic Name**: `/task_status`
**Message Type**: `task_msgs/TaskStatus` (custom)
**Publisher**: Orchestrator node
**Subscriber**: Feedback node, monitoring tools

**Message Definition** (`TaskStatus.msg`):
```
# TaskStatus.msg
std_msgs/Header header
string task_id
string current_state    # "IDLE", "LISTENING", "PLANNING", "NAVIGATING", "DETECTING", "GRASPING", "PLACING", "REPORTING", "ERROR"
string current_step_id  # Which step is currently executing
float32 progress        # 0.0 to 1.0 (overall task progress)
string status_message   # Human-readable status (e.g., "Navigating to table...")
bool success           # Overall task success/failure
string error_message    # If success=false, description of error
```

**Example**:
```yaml
task_id: "abc123"
current_state: "GRASPING"
current_step_id: "step3"
progress: 0.6
status_message: "Attempting to grasp red box"
success: false  # Not complete yet
error_message: ""
```

---

## ROS 2 Actions

### Navigate to Pose Action

**Action Name**: `/navigate_to_pose`
**Action Type**: `nav2_msgs/action/NavigateToPose` (standard Nav2)
**Server**: Nav2 controller_server
**Client**: Orchestrator node

**Goal**:
```
geometry_msgs/PoseStamped pose  # Target pose in map frame
string behavior_tree  # Optional: custom BT for this goal
```

**Feedback**:
```
geometry_msgs/PoseStamped current_pose  # Current robot pose
builtin_interfaces/Duration navigation_time  # Time elapsed
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries  # How many recovery behaviors triggered
float32 distance_remaining  # Meters to goal
```

**Result**:
```
std_msgs/Empty  # Empty result (success indicated by action status)
```

### Grasp Object Action

**Action Name**: `/grasp_object`
**Action Type**: `manipulation_msgs/action/GraspObject` (custom)
**Server**: Manipulation node
**Client**: Orchestrator node

**Goal**:
```
string object_id  # Object to grasp (from perception)
geometry_msgs/Pose object_pose  # 3D pose of object
string grasp_type  # "top_grasp", "side_grasp", "pinch_grasp"
float32 approach_distance  # Pre-grasp approach distance (meters)
```

**Feedback**:
```
string current_phase  # "APPROACHING", "GRASPING", "LIFTING", "RETRACTING"
float32 gripper_force  # Current gripper force (N)
float32 success_probability  # Estimated grasp success (0.0-1.0)
```

**Result**:
```
bool success  # Grasp succeeded
geometry_msgs/Pose final_object_pose  # Object pose after grasp
string failure_reason  # If success=false, why it failed
```

### Place Object Action

**Action Name**: `/place_object`
**Action Type**: `manipulation_msgs/action/PlaceObject` (custom)
**Server**: Manipulation node
**Client**: Orchestrator node

**Goal**:
```
string object_id  # Object currently grasped
geometry_msgs/Pose target_pose  # Where to place object
float32 placement_height  # Height above surface (meters)
```

**Feedback**:
```
string current_phase  # "MOVING_TO_TARGET", "LOWERING", "RELEASING", "RETRACTING"
float32 distance_to_target  # Meters
```

**Result**:
```
bool success
geometry_msgs/Pose final_object_pose
string failure_reason
```

---

## Service Contracts

### Get Scene Description Service

**Service Name**: `/get_scene_description`
**Service Type**: `perception_msgs/srv/GetSceneDescription` (custom)
**Server**: Perception node
**Client**: LLM Planner node

**Request**:
```
bool include_objects  # Include detected objects
bool include_map  # Include occupancy map summary
```

**Response**:
```
vision_msgs/Detection3DArray detected_objects
string map_summary  # Text description of environment (e.g., "3m x 4m room with table at (2, 1)")
geometry_msgs/Pose robot_pose  # Current robot position
```

### Validate Command Service

**Service Name**: `/validate_command`
**Service Type**: `safety_msgs/srv/ValidateCommand` (custom)
**Server**: Safety Validator node
**Client**: Action Grounding module

**Request**:
```
string command_type  # "navigate_to", "grasp_object", etc.
geometry_msgs/Pose target_pose
string[] parameters
```

**Response**:
```
bool is_valid
string validation_message  # If invalid, reason why
string[] warnings  # Non-fatal warnings (e.g., "Target pose near workspace limit")
```

---

## Transform (TF) Tree Structure

**Frame Hierarchy** (REP 105 compliant):

```
map
└── odom
    └── base_link (torso)
        ├── head_link
        │   ├── camera_link
        │   └── camera_depth_link
        ├── lidar_link
        ├── imu_link
        ├── left_upper_arm_link
        │   └── left_forearm_link
        │       └── left_hand_link
        ├── right_upper_arm_link
        │   └── right_forearm_link
        │       └── right_hand_link
        ├── left_thigh_link
        │   └── left_shin_link
        │       └── left_foot_link
        └── right_thigh_link
            └── right_shin_link
                └── right_foot_link
```

**Key Transforms**:
- `map` → `odom`: Published by AMCL or VSLAM (global localization)
- `odom` → `base_link`: Published by odometry source (wheel encoders, IMU integration, or ground truth in sim)
- `base_link` → `<all other links>`: Published by `robot_state_publisher` based on URDF and joint states

**Transform Publishers**:
- **robot_state_publisher**: Publishes all transforms from URDF based on `/joint_states`
- **AMCL/VSLAM**: Publishes `map` → `odom` transform
- **Gazebo/Isaac Sim**: Publishes `odom` → `base_link` (ground truth odometry in simulation)

---

## Parameter Contracts

### Nav2 Parameter Contract

**File**: `nav2_params_humanoid.yaml`
**Required Parameters**:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Hz
    min_vel_x: 0.0
    max_vel_x: 0.3  # m/s (slower for bipedal stability)
    max_vel_theta: 0.5  # rad/s (careful turning)
    acc_lim_x: 0.2  # m/s² (gradual acceleration)
    acc_lim_theta: 0.3  # rad/s²
    footprint: [[0.1, 0.15], [-0.1, 0.15], [-0.1, -0.15], [0.1, -0.15]]  # Humanoid foot area

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Hz
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.1  # m
      use_astar: true

costmap:
  global_costmap:
    global_frame: map
    robot_base_frame: base_link
    resolution: 0.05  # m (5cm cells)
    inflation_radius: 0.3  # m (larger for humanoid safety)
  local_costmap:
    global_frame: odom
    robot_base_frame: base_link
    resolution: 0.05
    width: 3.0  # m
    height: 3.0  # m
```

### Isaac ROS Parameter Contract

**File**: `isaac_ros_params.yaml`
**Required Parameters**:
```yaml
nvblox_node:
  ros__parameters:
    voxel_size: 0.05  # m (5cm voxels)
    max_integration_distance: 10.0  # m
    esdf_2d_min_height: 0.0  # m
    esdf_2d_max_height: 2.0  # m (humanoid height range)

visual_slam:
  ros__parameters:
    denoise_input_images: true
    rectified_images: true
    enable_slam_visualization: true
    enable_localization_n_mapping: true
    map_frame: map
    odom_frame: odom
    base_frame: base_link
```

### LLM Planner Parameter Contract

**File**: `llm_planner_params.yaml`
**Required Parameters**:
```yaml
llm_planner:
  ros__parameters:
    model: "gpt-4"  # or "qwen-2.5-7b"
    api_key: "${OPENAI_API_KEY}"  # Environment variable
    temperature: 0.2  # Low for deterministic planning
    max_tokens: 500
    timeout: 5.0  # seconds
    system_prompt_file: "config/robotics_system_prompt.txt"
    action_vocabulary: ["navigate_to", "detect_object", "grasp_object", "place_object", "report_status"]
    safety_validation_enabled: true
```

---

## Action Server Contracts

### Navigate to Pose Action Server

**Server Node**: `controller_server` (Nav2)
**Action Interface**: `nav2_msgs/action/NavigateToPose`

**Contract**:
- **Availability**: Must be available before task execution
- **Timeout**: 60 seconds per navigation goal (configurable)
- **Preemption**: Supports goal preemption (can cancel in-progress navigation)
- **Recovery**: Automatic recovery behaviors (rotate, back up) if stuck
- **Feedback Rate**: Minimum 1 Hz (position updates)

**Error Handling**:
- **NO_VALID_PATH**: Planner cannot find path → report to user
- **STUCK**: Robot unable to make progress → trigger recovery → if fails, report
- **COLLISION**: Unexpected collision → stop immediately → report

### Grasp Object Action Server

**Server Node**: `manipulation_server`
**Action Interface**: `manipulation_msgs/action/GraspObject` (custom)

**Contract**:
- **Availability**: Must be available after perception detects object
- **Timeout**: 30 seconds per grasp attempt
- **Preemption**: Supports cancellation (return to safe pose)
- **Feedback Rate**: Minimum 2 Hz (gripper force updates)
- **Prerequisites**: Object 3D pose must be known (from perception)

**Error Handling**:
- **OBJECT_NOT_FOUND**: Perception cannot locate object → request re-detection
- **IK_FAILED**: Inverse kinematics cannot find solution → report unreachable
- **GRASP_FAILED**: Gripper reports no contact → retry or report failure

### Place Object Action Server

**Server Node**: `manipulation_server`
**Action Interface**: `manipulation_msgs/action/PlaceObject` (custom)

**Contract**:
- **Availability**: Must be available after successful grasp
- **Timeout**: 30 seconds per placement
- **Preemption**: Supports cancellation (hold object, return to safe pose)
- **Feedback Rate**: Minimum 2 Hz
- **Prerequisites**: Object must be grasped

**Error Handling**:
- **TARGET_UNREACHABLE**: IK cannot reach target pose → report
- **UNSTABLE_PLACEMENT**: Surface unstable or sloped → warn and request confirmation
- **RELEASE_FAILED**: Gripper cannot release object → report

---

## Coordinate Frame Contracts

### Standard Frames (REP 105)

**World Frames**:
- **`map`**: Global fixed frame, origin at map origin, Z-up
- **`odom`**: Local fixed frame, origin where robot started or last reset
- **`base_link`**: Robot body frame, origin at center of torso, X-forward, Y-left, Z-up

**Sensor Frames**:
- **`camera_link`**: Camera optical frame, X-right, Y-down, Z-forward (camera convention)
- **`camera_depth_link`**: Depth camera optical frame
- **`lidar_link`**: LiDAR frame, X-forward, Y-left, Z-up
- **`imu_link`**: IMU frame, aligned with base_link

**Manipulation Frames**:
- **`left_hand_link`**: Left gripper frame, X-forward (grasp direction)
- **`right_hand_link`**: Right gripper frame

### Transform Timing Requirements

**Static Transforms** (robot_state_publisher):
- Published once at startup, then on robot model updates
- Latency: N/A (static)

**Dynamic Transforms** (odometry, SLAM):
- **`map` → `odom`**: Update rate ≥1 Hz (AMCL/VSLAM)
- **`odom` → `base_link`**: Update rate ≥50 Hz (odometry)

**Lookup Timeout**:
- All TF lookups should succeed within 100ms
- Use `tf2_ros::Buffer::lookupTransform()` with timeout parameter

---

## Validation Contracts

### Code Example Validation

**Criteria**:
1. **Syntax**: Python code passes `pylint` and `flake8` (PEP 8 compliant)
2. **Execution**: All examples run without errors on Ubuntu 22.04 + ROS 2 Humble
3. **Dependencies**: All dependencies listed in `requirements.txt` or `package.xml`
4. **Documentation**: Each example includes README with: purpose, usage, expected output

**Validation Script**:
```bash
#!/bin/bash
# validate_examples.sh
for example in module*/**/examples/*/; do
    cd "$example"
    # Check ROS 2 package
    colcon build
    colcon test
    # Check Python syntax
    pylint <package_name>/*.py
    cd -
done
```

### Chapter Validation

**Criteria**:
1. **Word Count**: Within target range ±10%
2. **References**: Minimum citations met per module
3. **Code Blocks**: All code blocks have language tags for syntax highlighting
4. **Links**: All internal links resolve correctly
5. **Diagrams**: All diagrams render in Docusaurus

**Validation Script**:
```bash
#!/bin/bash
# validate_chapters.sh
# Count words per module (excluding code blocks)
for module in docs/module*/; do
    word_count=$(grep -v '```' $module/*.md | wc -w)
    echo "$module: $word_count words"
done

# Check links
markdown-link-check docs/**/*.md

# Build Docusaurus
npm run build
```

---

## Conclusion

All data models, message contracts, action contracts, and validation criteria defined. Ready to proceed to quickstart.md generation.

**Phase 1 Status**:
- ✅ data-model.md complete
- ✅ contracts/module-interfaces.md complete
- ✅ contracts/ros2-message-contracts.md complete
- ⏳ quickstart.md (next)
