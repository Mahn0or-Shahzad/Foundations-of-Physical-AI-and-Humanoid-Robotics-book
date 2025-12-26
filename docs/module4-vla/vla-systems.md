---
id: vla-systems
title: Module 4 - Vision-Language-Action Robotics
sidebar_position: 7
description: Integrating large language models with robotic perception and control through Vision-Language-Action architectures for natural language humanoid control.
keywords: [VLA, vision-language-action, Whisper, GPT-4, Qwen, CLIP, LLaVA, embodied AI, language grounding]
---

# Module 4: Vision-Language-Action Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the Vision-Language-Action (VLA) paradigm and its role in embodied AI
2. Integrate Whisper automatic speech recognition (ASR) with ROS 2 for voice input
3. Use large language models (GPT-4 or Qwen) for robotic task planning
4. Design prompts that translate natural language commands into structured action sequences
5. Implement vision-language grounding using CLIP and LLaVA
6. Build action grounding modules that convert LLM plans into ROS 2 action goals
7. Create end-to-end voice-to-action pipelines for humanoid robot control
8. Understand multi-modal perception fusion (vision + language + proprioception)

---

## The Vision-Language-Action Paradigm

Traditional robotic systems require **explicit programming**: engineers hand-code behaviors for each task (pick up cup, navigate to table, open door). This approach:

- **Scales poorly**: Each new task requires new code
- **Lacks generalization**: "Pick up red cup" and "grasp blue mug" need separate implementations
- **Ignores common sense**: Robots cannot leverage knowledge that "cups are usually on tables" or "you should approach objects from above when grasping"

**Vision-Language-Action (VLA)** systems fundamentally change this by leveraging **foundation models** trained on massive internet-scale datasets (Driess et al., 2023; Brohan et al., 2023):

- **Vision Models**: Understand visual scenes (what objects exist, where they are)
- **Language Models**: Reason about tasks, decompose goals, apply common sense
- **Action Models**: Ground high-level plans into low-level motor commands

**Key Insight**: Large language models trained on billions of web pages contain implicit knowledge about the physical world ("objects have weight", "stairs require careful foot placement", "fragile items should be grasped gently"). VLA systems transfer this **web knowledge** to robotic control.

### The VLA Architecture

```
[Human Voice Command] → "Go to the table and bring me the red mug"
         ↓
[Speech Recognition] → Whisper ASR
         ↓
[Transcribed Text] → "go to the table and bring me the red mug"
         ↓
[Language Model] → GPT-4 / Qwen (task decomposition)
         ↓
[Structured Plan] → [
    {action: "navigate_to", target: "table", position: [2.0, 1.0, 0]},
    {action: "detect_object", object: "red mug"},
    {action: "grasp_object", object_id: "mug_001", grasp_type: "top_grasp"},
    {action: "navigate_to", target: "user", position: [0, 0, 0]},
    {action: "release_object"}
]
         ↓
[Vision-Language Grounding] → CLIP / LLaVA (find "red mug" in camera image)
         ↓
[3D Object Localization] → Depth camera + bounding box → (x: 2.1, y: 1.0, z: 0.8)
         ↓
[Action Grounding] → Convert plan into ROS 2 action goals
         ↓
[ROS 2 Actions] → /navigate_to_pose, /grasp_object (executed by humanoid)
         ↓
[Visual Feedback] → Camera confirms grasp success
         ↓
[Status Report] → "Task completed: I brought you the red mug"
```

This **closed-loop** system combines perception (seeing the mug), cognition (understanding the request), and action (physically retrieving the mug) in a unified architecture.

---

## Part 1: Speech Recognition with Whisper

**OpenAI Whisper** (Radford et al., 2022) is a state-of-the-art automatic speech recognition (ASR) system trained on 680,000 hours of multilingual audio. It achieves near-human accuracy and is open-source, making it ideal for robotics applications.

### Whisper Model Selection

| Model | Parameters | VRAM | Speed (CPU) | Accuracy | Use Case |
|-------|------------|------|-------------|----------|----------|
| **tiny** | 39M | ~1GB | ~5× real-time | 80% WER | Embedded (Jetson), low-latency |
| **base** | 74M | ~1GB | ~3× real-time | 85% WER | General robotics |
| **small** | 244M | ~2GB | ~1× real-time | 90% WER | Recommended balance |
| **medium** | 769M | ~5GB | ~0.5× real-time | 93% WER | High accuracy, GPU available |
| **large** | 1550M | ~10GB | ~0.2× real-time | 95% WER | Best accuracy, slow |

**Recommendation for Humanoids**: Use **small** model (90% accuracy, runs real-time on modern CPUs) or **medium** (93% accuracy, requires GPU but provides excellent quality).

### Integrating Whisper with ROS 2

**Install Whisper**:
```bash
pip install openai-whisper
# Or for faster inference:
pip install whisper-cpp-python  # C++ implementation, 4× faster
```

**Create Whisper ROS 2 Node** (`whisper_node.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import pyaudio

class WhisperNode(Node):
    """ROS 2 node for speech recognition using Whisper."""

    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model
        self.declare_parameter('model_size', 'small')
        model_size = self.get_parameter('model_size').value
        self.get_logger().info(f'Loading Whisper model: {model_size}...')
        self.model = whisper.load_model(model_size)

        # Publisher for transcribed text
        self.publisher_ = self.create_publisher(String, '/speech_command', 10)

        # Audio capture configuration
        self.sample_rate = 16000  # 16 kHz (Whisper requirement)
        self.chunk_duration = 5  # Record 5-second chunks

        self.get_logger().info('Whisper node ready. Listening...')

    def record_audio(self):
        """Capture audio from microphone."""
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=1024
        )

        self.get_logger().info('Recording...')
        frames = []
        for _ in range(0, int(self.sample_rate / 1024 * self.chunk_duration)):
            data = stream.read(1024)
            frames.append(np.frombuffer(data, dtype=np.int16))

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Convert to float32, normalize
        audio = np.concatenate(frames).astype(np.float32) / 32768.0
        return audio

    def transcribe_and_publish(self):
        """Record audio, transcribe with Whisper, publish to ROS 2."""
        audio = self.record_audio()

        # Transcribe
        result = self.model.transcribe(audio, language='en')
        text = result['text'].strip()

        if text:
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Transcribed: "{text}"')
        else:
            self.get_logger().warn('No speech detected')

def main():
    rclpy.init()
    node = WhisperNode()

    # Continuous listening loop
    while rclpy.ok():
        node.transcribe_and_publish()
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
```

**Usage**:
```bash
ros2 run humanoid_vla whisper_node --ros-args -p model_size:=small

# Speak into microphone: "go to the table and pick up the red box"
# Published to /speech_command: "Go to the table and pick up the red box"
```

**Latency**: ~1-3 seconds on modern CPU (acceptable for non-time-critical voice commands).

---

## Part 2: LLM-Based Task Planning

Once speech is transcribed, a **language model** decomposes the natural language command into a structured sequence of robotic actions.

### Prompt Engineering for Robotics

LLMs require **carefully designed prompts** that specify:
1. **Available actions**: What the robot can do
2. **Output format**: Structured JSON for easy parsing
3. **Safety constraints**: Physical limits, prohibited actions
4. **Scene context**: Current environment state, object locations

**System Prompt** (`config/robotics_system_prompt.txt`):

```text
You are a task planner for a humanoid robot. Given a natural language command and scene description, output a JSON plan with these actions ONLY:

Actions:
- navigate_to: Move to a location (x, y, theta in map frame)
- detect_object: Identify and localize an object by name/description
- grasp_object: Pick up a detected object (specify object_id and grasp_type)
- place_object: Put down grasped object at target location
- report_status: Communicate task completion or failure to user

Output Format (JSON):
{
  "task_description": "brief summary",
  "steps": [
    {"action": "navigate_to", "target": "table", "params": {"x": 2.0, "y": 1.0, "theta": 0.0}},
    {"action": "detect_object", "object_name": "red box", "params": {}},
    {"action": "grasp_object", "object_id": "auto", "params": {"grasp_type": "top_grasp"}},
    ...
  ],
  "estimated_duration": 60
}

Constraints:
- Ensure actions are physically feasible (e.g., cannot grasp before detecting object)
- Include navigation before manipulation (robot must be at object location)
- Limit plans to 5 steps maximum (complex tasks should be decomposed)
- If unclear or impossible, output {"error": "reason"}

Current Scene:
{scene_context}

User Command:
{user_command}
```

### LLM Integration: GPT-4 API

**Install OpenAI SDK**:
```bash
pip install openai
```

**LLM Planner Node** (`llm_planner_node.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LLMPlannerNode(Node):
    """ROS 2 node for LLM-based task planning."""

    def __init__(self):
        super().__init__('llm_planner')

        # Load API key from parameter
        self.declare_parameter('openai_api_key', '')
        api_key = self.get_parameter('openai_api_key').value
        openai.api_key = api_key

        # Load system prompt
        with open('config/robotics_system_prompt.txt', 'r') as f:
            self.system_prompt = f.read()

        # Subscribe to speech commands
        self.subscription = self.create_subscription(
            String, '/speech_command', self.command_callback, 10
        )

        # Publish task plans
        self.publisher_ = self.create_publisher(String, '/task_plan', 10)

        self.get_logger().info('LLM Planner ready (model: gpt-4)')

    def command_callback(self, msg):
        """Receive voice command, generate plan with LLM."""
        user_command = msg.data
        self.get_logger().info(f'Planning for command: "{user_command}"')

        # Get scene context (simplified - in real system, query perception node)
        scene_context = self.get_scene_context()

        # Construct prompt
        prompt = self.system_prompt.replace('{scene_context}', scene_context)
        prompt = prompt.replace('{user_command}', user_command)

        # Call GPT-4
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": f"Scene: {scene_context}\nCommand: {user_command}"}
                ],
                temperature=0.2,  # Low temperature for deterministic planning
                max_tokens=500
            )

            plan_json = response['choices'][0]['message']['content']

            # Validate JSON
            plan = json.loads(plan_json)

            # Publish plan
            msg_out = String()
            msg_out.data = plan_json
            self.publisher_.publish(msg_out)

            self.get_logger().info(f'Plan generated: {plan["task_description"]}')

        except Exception as e:
            self.get_logger().error(f'LLM planning failed: {e}')

    def get_scene_context(self):
        """Query perception system for current scene state."""
        # In real implementation, call service: /get_scene_description
        # Returns: list of detected objects, robot pose, obstacle locations
        return "Objects: table at (2.0, 1.0), red box on table, blue mug on shelf. Robot at origin."

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Usage**:
```bash
# Set API key
export OPENAI_API_KEY='your-key-here'

# Launch planner
ros2 run humanoid_vla llm_planner_node

# Publish voice command (simulated)
ros2 topic pub /speech_command std_msgs/String "data: 'bring me the red box'"

# Received on /task_plan:
# {
#   "task_description": "Retrieve red box and deliver to user",
#   "steps": [
#     {"action": "navigate_to", "target": "table", "params": {"x": 2.0, "y": 1.0, "theta": 0.0}},
#     {"action": "detect_object", "object_name": "red box", "params": {}},
#     {"action": "grasp_object", "object_id": "auto", "params": {"grasp_type": "top_grasp"}},
#     {"action": "navigate_to", "target": "user", "params": {"x": 0.0, "y": 0.0, "theta": 3.14}},
#     {"action": "release_object", "params": {}}
#   ],
#   "estimated_duration": 45
# }
```

**Cost**: GPT-4 API ~$0.03 per request (input) + $0.06 per response (~500 tokens) = **~$0.05 per command**. For 100 course exercises: ~$5. Affordable for educational use.

### Local LLM Alternative: Qwen

For offline operation or budget constraints, use **Qwen 2.5** (open-source LLM from Alibaba):

```python
from transformers import AutoModelForCausalLM, AutoTokenizer

# Load Qwen model (one-time download, ~15GB)
model = AutoModelForCausalLM.from_pretrained(
    "Qwen/Qwen2.5-7B-Instruct",
    device_map="auto",  # Automatically use GPU if available
    torch_dtype="auto"
)
tokenizer = AutoTokenizer.from_pretrained("Qwen/Qwen2.5-7B-Instruct")

# Generate plan
messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": f"Scene: {scene}\nCommand: {command}"}
]

inputs = tokenizer.apply_chat_template(messages, return_tensors="pt").to(model.device)
outputs = model.generate(inputs, max_new_tokens=512)
plan_json = tokenizer.decode(outputs[0], skip_special_tokens=True)
```

**Trade-offs**:
- **Pros**: Free, offline, no API limits, privacy (data stays local)
- **Cons**: Requires GPU (16GB VRAM for 7B model), slightly lower quality than GPT-4, slower inference (~2-5 seconds vs 1-2 seconds)

**Recommendation**: Use GPT-4 for development (faster iteration), switch to Qwen for deployment (no API costs, offline operation).

---

## Part 3: Vision-Language Grounding

LLMs can generate text like "detect the red box", but robots need **3D coordinates** (x, y, z) to navigate and grasp. **Vision-language models** bridge this gap.

### CLIP: Zero-Shot Object Recognition

**CLIP** (Contrastive Language-Image Pre-training) learns joint embeddings of images and text (Radford et al., 2021). Given an image and text query ("red box"), CLIP computes similarity scores without task-specific training.

**CLIP for Robotics**:

```python
import torch
import clip
from PIL import Image

# Load CLIP model
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Capture camera image from ROS 2 topic (simplified)
image = Image.open('/tmp/camera_image.jpg')  # In real code, subscribe to /camera/image_raw

# Text queries
text_queries = ["red box", "blue mug", "green bottle", "table", "chair"]
text = clip.tokenize(text_queries).to(device)

# Compute similarity
with torch.no_grad():
    image_features = model.encode_image(preprocess(image).unsqueeze(0).to(device))
    text_features = model.encode_text(text)

    # Cosine similarity
    similarity = (image_features @ text_features.T).softmax(dim=-1)

# Find best match
values, indices = similarity[0].topk(1)
detected_object = text_queries[indices[0].item()]
confidence = values[0].item()

print(f"Detected: {detected_object} (confidence: {confidence:.2f})")
# Output: "Detected: red box (confidence: 0.87)"
```

**Integration with Object Detection**:

1. **Run YOLO** to get bounding boxes of all objects in image
2. **For each bounding box**, crop image and run CLIP with text query ("red box")
3. **Best match** (highest CLIP similarity) is the target object
4. **Output**: Bounding box coordinates (pixel space) of "red box"

### LLaVA: Visual Question Answering

**LLaVA** (Large Language and Vision Assistant) extends LLMs with vision (Liu et al., 2023). Unlike CLIP (matches text to image), LLaVA **answers questions** about images.

**Example Queries**:
- "Where is the red box?" → "On the table, left side"
- "How many mugs are visible?" → "Two mugs: one blue, one white"
- "Is the table clear?" → "No, there are three objects on it"

**Using LLaVA** (via Hugging Face Transformers):

```python
from transformers import LlavaForConditionalGeneration, AutoProcessor
from PIL import Image

# Load LLaVA model
model = LlavaForConditionalGeneration.from_pretrained("llava-hf/llava-1.5-7b-hf")
processor = AutoProcessor.from_pretrained("llava-hf/llava-1.5-7b-hf")

# Load camera image
image = Image.open('/tmp/camera_image.jpg')

# Ask question
prompt = "USER: <image>\nWhere is the red box?\nASSISTANT:"
inputs = processor(text=prompt, images=image, return_tensors="pt")

# Generate answer
outputs = model.generate(**inputs, max_new_tokens=50)
answer = processor.decode(outputs[0], skip_special_tokens=True)

print(answer)
# Output: "The red box is on the table, approximately 30 cm from the left edge."
```

**Use Case**: When LLM plan says "detect red box" but **multiple boxes** exist, LLaVA can disambiguate: "Which box? There are two boxes." → "The one on the table" → LLaVA identifies correct object.

### 3D Object Localization

Vision-language models provide **image-space information** (bounding boxes, text descriptions). Robots need **3D world coordinates**.

**Pipeline**: Image → CLIP/YOLO → Bounding Box → Depth Map → 3D Point Cloud → Object Pose

**Code Example** (combining YOLO + Depth Camera):

```python
import cv2
import numpy as np
from ultralytics import YOLO

# Run YOLO detection
yolo_model = YOLO('yolov8n.pt')
results = yolo_model(image)  # Detects objects, returns bounding boxes

# Get bounding box for "red box" (assume YOLO class 0 = box)
for detection in results[0].boxes:
    if detection.cls == 0:  # Box class
        # Bounding box in pixels
        x1, y1, x2, y2 = detection.xyxy[0].cpu().numpy()
        center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)

        # Get depth at center
        depth_value = depth_image[center_y, center_x]  # Meters

        # Convert pixel to 3D using camera intrinsics
        fx, fy = 525.0, 525.0  # Focal lengths (from camera_info)
        cx, cy = 320.0, 240.0  # Principal point (image center)

        # Pinhole camera model
        x_3d = (center_x - cx) * depth_value / fx
        y_3d = (center_y - cy) * depth_value / fy
        z_3d = depth_value

        print(f"Red box at 3D position: ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f}) meters")
        # Output: "Red box at 3D position: (2.10, 1.05, 0.80) meters"
```

**Result**: 3D coordinates in camera frame. Use TF transforms to convert to `map` or `base_link` frame for navigation and grasping.

---

## Part 4: Action Grounding — From Plans to Robot Behavior

The LLM outputs JSON plans. **Action grounding** translates these into ROS 2 action goals that the robot executes.

### Action Grounding Node

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json

class ActionGroundingNode(Node):
    """Converts LLM plans into ROS 2 action executions."""

    def __init__(self):
        super().__init__('action_grounding')

        # Subscribe to task plans
        self.subscription = self.create_subscription(
            String, '/task_plan', self.plan_callback, 10
        )

        # Action clients for robot behaviors
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Wait for action servers to be available
        self.nav_client.wait_for_server()

        self.get_logger().info('Action Grounding ready')

    def plan_callback(self, msg):
        """Execute plan step-by-step."""
        plan = json.loads(msg.data)

        self.get_logger().info(f'Executing plan: {plan["task_description"]}')

        for step in plan['steps']:
            action_type = step['action']

            if action_type == 'navigate_to':
                self.execute_navigation(step)
            elif action_type == 'detect_object':
                self.execute_detection(step)
            elif action_type == 'grasp_object':
                self.execute_grasp(step)
            # ... handle other actions

    def execute_navigation(self, step):
        """Send navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()

        # Construct PoseStamped from step parameters
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = step['params']['x']
        goal_msg.pose.pose.position.y = step['params']['y']
        goal_msg.pose.pose.orientation.w = 1.0  # Simplified orientation

        # Send goal asynchronously
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        """Handle navigation result."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return

        self.get_logger().info('Navigation in progress...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Navigation completed."""
        result = future.result().result
        self.get_logger().info('Navigation complete!')
        # Continue to next step in plan...
```

**Workflow**: LLM plan → Action Grounding parses JSON → Sends ROS 2 action goals → Robot executes → Reports completion → Next step.

---

## Part 5: Multi-Modal Perception Fusion

VLA systems combine **vision** (cameras), **language** (LLM understanding), and **proprioception** (robot's own state):

### Multi-Modal Data Sources

1. **Vision**: RGB images (object appearance), depth maps (3D structure)
2. **Language**: User commands (intent), object descriptions (attributes)
3. **Proprioception**: Joint positions (robot pose), IMU (orientation), force sensors (contact)
4. **World Model**: Maps (environment structure), object database (known objects)

**Fusion Strategy**:

```python
class MultiModalFusion(Node):
    """Fuses vision, language, and proprioception for decision-making."""

    def __init__(self):
        super().__init__('multimodal_fusion')

        # Vision input
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Detection3DArray, '/detected_objects', self.detections_callback, 10)

        # Language input
        self.create_subscription(String, '/speech_command', self.command_callback, 10)

        # Proprioception input
        self.create_subscription(JointState, '/joint_states', self.joints_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Fused world state
        self.world_state = {
            'robot_pose': None,
            'detected_objects': [],
            'current_command': None,
            'robot_orientation': None
        }

    def image_callback(self, msg):
        """Update visual information."""
        # Run CLIP or YOLO on image, update detected objects
        pass

    def command_callback(self, msg):
        """Update current task from language."""
        self.world_state['current_command'] = msg.data
        # Trigger replanning if needed

    def imu_callback(self, msg):
        """Update robot orientation."""
        self.world_state['robot_orientation'] = msg.orientation
        # Check if robot is falling (adjust plan priority)

    def make_decision(self):
        """Combine all modalities to decide next action."""
        # Example: If robot is tilting (IMU) AND object detected (vision) AND user said "careful" (language)
        # → Slow down grasp approach velocity

        if self.world_state['robot_orientation'].y > 0.2:  # Tilting forward
            self.get_logger().warn('Robot unstable, pausing manipulation')
            # Cancel grasp action, stabilize first
```

**Replanning on Perception Failure**: If object detection fails (occlusion, lighting change), VLA system can:
1. **Retry**: Adjust camera angle, move closer, improve lighting
2. **Ask for help**: "I cannot find the red box. Can you move it to a clearer location?"
3. **Adapt plan**: Skip this object, try alternative approach

This **adaptive behavior** distinguishes VLA systems from brittle hand-coded robots.

---

## Safety Validation Layer

LLMs can generate **invalid or unsafe commands**. A safety validator filters these before execution.

### Safety Validator

```python
class SafetyValidator(Node):
    """Validates LLM-generated commands before execution."""

    def __init__(self):
        super().__init__('safety_validator')
        # Subscribe to raw plans, publish validated plans
        self.subscription = self.create_subscription(String, '/task_plan_raw', self.validate_plan, 10)
        self.publisher_ = self.create_publisher(String, '/task_plan_validated', 10)

        # Define workspace limits
        self.workspace_limits = {
            'x': (-3.0, 3.0),  # Robot can navigate within ±3m
            'y': (-3.0, 3.0),
            'z_grasp': (0.3, 1.5)  # Can only grasp objects 30cm-150cm high
        }

    def validate_plan(self, msg):
        """Check plan safety and feasibility."""
        plan = json.loads(msg.data)

        for step in plan['steps']:
            if step['action'] == 'navigate_to':
                x, y = step['params']['x'], step['params']['y']
                if not (self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]):
                    self.get_logger().error(f'Invalid navigation: x={x} outside workspace')
                    return  # Reject plan

            elif step['action'] == 'grasp_object':
                # Check if object position is reachable
                # (In real system, call IK solver service)
                pass

        # Plan is valid, publish
        self.publisher_.publish(msg)
        self.get_logger().info('Plan validated and approved')
```

**Safety Constraints**:
- **Workspace limits**: Reject goals outside robot's reach
- **Joint limits**: Ensure IK solutions don't violate joint bounds
- **Collision checking**: Verify planned paths are collision-free (use MoveIt or Nav2 costmaps)
- **Prohibited actions**: Blacklist dangerous commands (e.g., "jump off ledge")

---

## Complete VLA Pipeline: Voice-to-Action Example

### Scenario: "Go to the table and pick up the red box"

**Step-by-Step Execution**:

1. **Speech Input** (human speaks):
   - Audio captured via microphone
   - Whisper transcribes: "go to the table and pick up the red box"
   - Published to `/speech_command` (~2 seconds latency)

2. **LLM Planning** (GPT-4 or Qwen):
   - Receives transcribed text + scene context
   - Generates JSON plan:
     ```json
     {
       "steps": [
         {"action": "navigate_to", "target": "table", "params": {"x": 2.0, "y": 1.0}},
         {"action": "detect_object", "object_name": "red box"},
         {"action": "grasp_object", "params": {"grasp_type": "top_grasp"}},
         {"action": "report_status", "message": "I have the red box"}
       ]
     }
     ```
   - Published to `/task_plan` (~2 seconds latency)

3. **Safety Validation**:
   - Validator checks: table at (2.0, 1.0) is reachable ✅
   - No prohibited actions ✅
   - Approves plan (~0.1 seconds)

4. **Action Grounding - Navigate**:
   - Parses first step: `navigate_to table`
   - Sends Nav2 action goal: `/navigate_to_pose` with target (2.0, 1.0, 0.0)
   - Humanoid plans path (Nav2 global planner), executes (local planner)
   - **Duration**: 15-30 seconds (walking at 0.3 m/s)

5. **Action Grounding - Detect Object**:
   - Captures camera image
   - Runs YOLO + CLIP: detects all boxes, finds "red box" via text similarity
   - Gets bounding box: (x1=300, y1=200, x2=350, y2=280)
   - Retrieves depth at center: depth=0.8m
   - Computes 3D pose: (2.1, 1.0, 0.8) in `map` frame
   - **Duration**: 0.5-1 second

6. **Action Grounding - Grasp**:
   - Sends grasp action goal with object pose
   - Manipulation controller:
     - Computes IK for arm to reach (2.1, 1.0, 0.8)
     - Plans approach trajectory
     - Closes gripper when force sensors detect contact
   - **Duration**: 5-10 seconds

7. **Feedback**:
   - Publishes to `/task_status`: "Task completed successfully"
   - Optional: Text-to-speech (TTS) announces "I have the red box"

**Total Duration**: ~25-45 seconds from voice command to task completion.

**Success Rate**: With properly tuned systems:
- Speech recognition: over 90% (Whisper on clear audio)
- LLM planning: over 95% (well-designed prompts)
- Object detection: over 85% (CLIP + YOLO in good lighting)
- Navigation: over 90% (Nav2 on mapped environment)
- Grasping: over 75% (contact-rich, most challenging)
- **Overall**: ~50-70% end-to-end success (product of individual stages)

**Improvement Strategies**: Retry on failure, request user clarification, adaptive replanning.

---

## Practical Exercises: Building VLA Systems

### Exercise 1: Voice Command Recognition

**Objective**: Implement Whisper ASR publishing to ROS 2.

**Tasks**:
1. Install Whisper (`pip install openai-whisper`)
2. Create `whisper_node.py` (use code example above)
3. Record 5-second audio clips, transcribe, publish to `/speech_command`
4. Test with various commands: "go forward", "turn left", "stop"

**Success Criteria**:
- Transcription accuracy greater than 85% (test with 10 commands)
- Latency less than 3 seconds from speech end to publication
- Handles ambient noise reasonably (no transcription when silent)

### Exercise 2: LLM Task Planner

**Objective**: Use GPT-4 or Qwen to decompose commands into action sequences.

**Tasks**:
1. Set up OpenAI API key or download Qwen model
2. Create `llm_planner_node.py` with system prompt
3. Subscribe to `/speech_command`
4. Generate JSON plans for commands:
   - "Go to the kitchen" → [navigate_to kitchen]
   - "Bring me a cup" → [navigate_to kitchen, detect_object cup, grasp_object, navigate_to user, release_object]
5. Publish to `/task_plan`

**Success Criteria**:
- Valid JSON generated for 10 test commands
- Plans are logically ordered (navigate before grasp)
- Less than 5% invalid JSON (syntax errors)
- Plans match expected decomposition

### Exercise 3: CLIP Object Grounding

**Objective**: Detect objects using vision-language similarity.

**Tasks**:
1. Install CLIP (`pip install git+https://github.com/openai/CLIP.git`)
2. Capture image from `/camera/image_raw` topic
3. Run CLIP with queries: ["red box", "blue mug", "green bottle"]
4. Print similarity scores for each query
5. Identify object with highest score

**Success Criteria**:
- CLIP correctly identifies target object greater than 80% of time (test with 10 images)
- Similarity scores greater than 0.5 for correct object, less than 0.3 for incorrect
- Runs in less than 500ms on GPU (or less than 2 seconds on CPU)

### Exercise 4: End-to-End VLA Pipeline

**Objective**: Integrate all components: Whisper → LLM → Action Grounding → Nav2.

**Tasks**:
1. Launch all nodes: Whisper, LLM Planner, Action Grounding, Nav2
2. Launch Isaac Sim or Gazebo with humanoid
3. Speak command: "Go to the table"
4. Verify:
   - Whisper transcribes correctly
   - LLM generates navigation plan
   - Action Grounding sends Nav2 goal
   - Humanoid navigates to table
5. Test 5 different voice commands

**Success Criteria**:
- End-to-end pipeline executes without crashes
- At least 60% of commands result in correct robot behavior
- Latency: speech → robot starts moving in less than 10 seconds
- Failures gracefully reported ("I couldn't find the table")

---

## Summary and Next Steps

You now understand:

✅ **VLA paradigm**: Connecting language models with perception and action for embodied AI
✅ **Whisper ASR**: Speech recognition for voice-based robot control (~90% accuracy, 1-3s latency)
✅ **LLM planning**: GPT-4/Qwen decompose commands into structured action sequences
✅ **Prompt engineering**: Designing system prompts with action vocabularies and safety constraints
✅ **CLIP**: Zero-shot object recognition via vision-language similarity
✅ **LLaVA**: Visual question answering for scene understanding
✅ **3D localization**: Converting image detections to 3D world coordinates (YOLO + depth camera + camera intrinsics)
✅ **Action grounding**: Translating JSON plans to ROS 2 action goals (NavigateToPose, GraspObject)
✅ **Safety validation**: Filtering unsafe or infeasible commands
✅ **Multi-modal fusion**: Combining vision, language, and proprioception for robust decision-making

**In Module 5 (Capstone)**, you'll integrate **all four modules** into a complete autonomous humanoid system:
- ROS 2 (nervous system)
- Gazebo/Unity/Isaac (digital twins)
- Isaac ROS (perception)
- VLA (cognitive planning)

The capstone demonstrates a humanoid that **listens to voice commands, reasons about tasks, navigates environments, perceives objects, and executes manipulation**—all in a unified pipeline. This is the culmination of your Physical AI journey.

**Module 4 Validation Checkpoint**: You should be able to transcribe speech with Whisper, generate task plans with an LLM, ground objects using CLIP, and execute actions via ROS 2. If you can demonstrate a simple voice-controlled robot behavior (even just "go forward"), you're ready for the Capstone!

---

## References

Brohan, A., Brown, N., Carbajal, J., Chebotar, Y., Dabis, J., Finn, C., Gopalakrishnan, K., Hausman, K., Herzog, A., Hsu, J., Ibarz, J., Ichter, B., Irpan, A., Jackson, T., Jesmonth, S., Joshi, N. J., Julian, R., Kalashnikov, D., Kuang, Y., … Zitkovich, B. (2023). RT-2: Vision-language-action models transfer web knowledge to robotic control. *arXiv preprint arXiv:2307.15818*. https://arxiv.org/abs/2307.15818

Driess, D., Xia, F., Sajjadi, M. S., Lynch, C., Chowdhery, A., Ichter, B., Wahid, A., Tompson, J., Vuong, Q., Yu, T., Huang, W., Chebotar, Y., Sermanet, P., Duckworth, D., Levine, S., Vanhoucke, V., Hausman, K., Toussaint, M., Greff, K., … Florence, P. (2023). PaLM-E: An embodied multimodal language model. *Proceedings of the 40th International Conference on Machine Learning (ICML)*, 8469-8488.

Liu, H., Li, C., Wu, Q., & Lee, Y. J. (2023). Visual instruction tuning. *Advances in Neural Information Processing Systems (NeurIPS)*, 36.

Radford, A., Kim, J. W., Hallacy, C., Ramesh, A., Goh, G., Agarwal, S., Sastry, G., Askell, A., Mishkin, P., Clark, J., Krueger, G., & Sutskever, I. (2021). Learning transferable visual models from natural language supervision. *Proceedings of the 38th International Conference on Machine Learning (ICML)*, 8748-8763.

Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2022). Robust speech recognition via large-scale weak supervision. *arXiv preprint arXiv:2212.04356*. https://arxiv.org/abs/2212.04356

Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, 7(66), eabm6074.
