# Quick Start Guide: Embodied AI & Humanoid Robotics

**Project**: Embodied AI & Humanoid Robotics Book
**Branch**: `001-embodied-ai-book`
**Date**: 2025-12-19
**Purpose**: Reader environment setup and prerequisite validation

## Overview

This guide helps readers set up their development environment for the "Embodied AI & Humanoid Robotics" book. Follow these steps to ensure you can execute all code examples and complete the capstone project.

**Estimated Setup Time**: 2-4 hours (depending on internet speed and hardware)

---

## Prerequisites

### Required Background Knowledge

Before starting this book, you should have:

✅ **Programming**:
- Intermediate Python proficiency (functions, classes, modules, virtual environments)
- Familiarity with command-line interfaces (bash, terminal navigation)
- Basic understanding of package managers (pip, apt)

✅ **Mathematics**:
- Linear algebra fundamentals (vectors, matrices, dot product, cross product)
- Basic calculus (derivatives, gradients) - helpful but not essential
- Coordinate systems and transformations (2D/3D)

✅ **Operating Systems**:
- Comfortable with Linux (Ubuntu) command line
- Can install packages, edit configuration files, navigate filesystem

**Self-Assessment Quiz**:
If you answer "No" to more than 2 questions, consider prerequisite review (see Appendices):

1. Can you write a Python class with methods and attributes?
2. Do you understand what a matrix transformation does geometrically?
3. Can you install packages on Ubuntu using `apt` and `pip`?
4. Can you navigate directories and run scripts from the terminal?
5. Do you know what a coordinate frame or reference frame is?

**If you need review**, consult:
- Appendix A: Python Refresher
- Appendix B: Linear Algebra for Robotics
- Online: "Learn Python the Hard Way", "3Blue1Brown Linear Algebra" series

---

## Hardware Requirements

### Option 1: Local Workstation (Recommended for Full Experience)

**Minimum Configuration**:
- **CPU**: Intel i5-10400 (6 cores) or AMD Ryzen 5 5600X
- **GPU**: NVIDIA RTX 3060 (12GB VRAM) - **Required for Module 3 (Isaac Sim)**
- **RAM**: 16GB DDR4
- **Storage**: 100GB free space (SSD recommended)
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Recommended Configuration** (for better performance):
- **CPU**: Intel i7-12700 (12 cores) or AMD Ryzen 7 5800X
- **GPU**: NVIDIA RTX 4070 (16GB VRAM)
- **RAM**: 32GB DDR4
- **Storage**: 256GB SSD + 500GB HDD

**Estimated Cost**: $800-1500 (complete system) or $300-500 (GPU upgrade if you have existing PC)

### Option 2: Cloud GPU Instances (Budget-Friendly Alternative)

**AWS Option**:
- **Instance Type**: g4dn.xlarge
- **GPU**: NVIDIA Tesla T4 (16GB)
- **vCPUs**: 4
- **RAM**: 16GB
- **Storage**: 125GB SSD
- **Cost**: ~$0.526/hour (On-Demand) or ~$0.16/hour (Spot)
- **Estimated Course Cost**: ~$20-40 for 40-80 hours

**Azure Option**:
- **Instance Type**: NC6 (Standard_NC6)
- **GPU**: NVIDIA Tesla K80 (12GB)
- **vCPUs**: 6
- **RAM**: 56GB
- **Cost**: ~$0.90/hour
- **Estimated Course Cost**: ~$36-72 for 40-80 hours

**Google Cloud Option**:
- **Instance Type**: n1-standard-4 with T4 GPU
- **GPU**: NVIDIA Tesla T4 (16GB)
- **vCPUs**: 4
- **RAM**: 15GB
- **Cost**: ~$0.35/hour (preemptible) or ~$0.95/hour (on-demand)

**Cloud Setup Note**: All modules except Module 3 (Isaac Sim) can run on CPU-only cloud instances at much lower cost (~$0.05-0.10/hour).

### Hardware Decision Matrix

| Use Case | Recommended Hardware | Est. Cost |
|----------|---------------------|-----------|
| **Full course with Isaac Sim** | Local RTX 3060+ or cloud GPU | $300-500 (GPU) or $20-40 (cloud) |
| **Modules 1, 2, 4, 5 only (skip Isaac)** | CPU-only (any modern laptop) | $0 (existing hardware) or $5-10 (cloud) |
| **Budget-conscious with Isaac** | Cloud GPU instances (AWS Spot) | $10-20 (cloud spot pricing) |
| **Research/professional use** | Local RTX 4070+ workstation | $1000-1500 |

---

## Software Installation

### Step 1: Install Ubuntu 22.04 LTS

**If you already have Ubuntu 22.04**: Skip to Step 2.

**If you have Windows**: Install Ubuntu 22.04 via:
1. **Dual Boot** (recommended for performance): https://ubuntu.com/tutorials/install-ubuntu-desktop
2. **WSL 2** (Windows Subsystem for Linux): `wsl --install -d Ubuntu-22.04`
   - **Note**: WSL 2 supports GPU passthrough for NVIDIA GPUs (requires Windows 11 and NVIDIA drivers)
3. **Virtual Machine**: Use VirtualBox or VMware (not recommended for GPU-intensive tasks)

**If you have macOS**: Use cloud instances (AWS/Azure) or consider dual-boot with Ubuntu.

### Step 2: Install ROS 2 Humble

**Add ROS 2 Repository**:
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Install ROS 2 Humble Desktop**:
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

**Install Development Tools**:
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

**Source ROS 2 Setup** (add to `~/.bashrc`):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify Installation**:
```bash
ros2 --version
# Expected: ros2 cli version: 0.18.x

ros2 run demo_nodes_cpp talker
# Should see: [INFO] [talker]: Publishing: 'Hello World: 1'
```

### Step 3: Install Gazebo

**Gazebo Fortress** (recommended for ROS 2 Humble):
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

**Gazebo Classic 11** (alternative):
```bash
sudo apt install ros-humble-gazebo-ros
```

**Verify Installation**:
```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x or Gazebo Sim, version 6.x (Fortress)

# Test launch
gazebo
# Should open Gazebo GUI with empty world
```

### Step 4: Install Navigation Stack (Nav2)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*  # Useful examples
```

**Verify Installation**:
```bash
ros2 pkg list | grep nav2
# Expected: List of nav2 packages (nav2_bt_navigator, nav2_controller, etc.)
```

### Step 5: Install Python Dependencies

**Create Virtual Environment** (recommended):
```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
```

**Install Core Dependencies**:
```bash
pip install --upgrade pip
pip install opencv-python numpy pillow pyyaml
```

**Install AI/ML Libraries** (for Modules 3-4):
```bash
# Computer Vision
pip install ultralytics  # YOLOv8
pip install transformers torch  # Hugging Face models (Qwen, CLIP, LLaVA)

# Speech Recognition
pip install openai-whisper

# LLM Integration
pip install openai  # OpenAI API client
```

**Verify Installation**:
```bash
python3 -c "import cv2; import numpy; import whisper; print('All packages imported successfully')"
```

### Step 6: Install Visualization Tools

```bash
sudo apt install ros-humble-rviz2 ros-humble-rqt ros-humble-rqt-common-plugins
```

**Verify**:
```bash
rviz2
# Should open RViz 2 GUI
```

---

## Module-Specific Setup

### Module 2: Unity Setup (Optional but Recommended)

**Prerequisites**:
- Windows 10/11 or Ubuntu 22.04 (Unity has native Linux support)

**Install Unity Hub**:
```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage

# Make executable and run
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage
```

**Install Unity 2021.3 LTS**:
1. Open Unity Hub
2. Go to "Installs" → "Install Editor"
3. Select "2021.3 LTS (Long Term Support)"
4. Add modules: "Linux Build Support", "WebGL Build Support" (optional)

**Install Unity Robotics Hub**:
```bash
# Clone repository
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub

# Follow installation instructions in README
```

**ROS-TCP-Connector Setup**:
- Import package in Unity: Window → Package Manager → Add package from git URL
- URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

### Module 3: NVIDIA Isaac Sim Setup

**Prerequisites**:
- NVIDIA RTX GPU (RTX 3060 or better)
- NVIDIA drivers 525+ installed
- Ubuntu 22.04

**Install NVIDIA Drivers**:
```bash
# Check current driver
nvidia-smi

# If not installed or outdated:
sudo apt install nvidia-driver-535
sudo reboot
```

**Install Omniverse Launcher**:
```bash
# Download from NVIDIA
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Run installer
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**Install Isaac Sim 2023.1.0+**:
1. Open Omniverse Launcher
2. Go to "Exchange" → Search "Isaac Sim"
3. Click "Install" (requires NVIDIA account, free for individuals)
4. Wait for download (~50GB, may take 1-2 hours)

**Configure Isaac Sim for ROS 2**:
```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.0

# Install ROS 2 bridge dependencies
./python.sh -m pip install -r exts/omni.isaac.ros2_bridge/pip_requirements.txt

# Verify installation
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/ros2_talker.py
# Should see ROS 2 messages being published
```

**Isaac ROS Installation**:
```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-nvblox ros-humble-isaac-ros-visual-slam

# Or build from source (for latest features):
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
cd ~/isaac_ros_ws
colcon build
```

### Module 4: LLM API Setup

**Option A: OpenAI API** (recommended for simplicity):
```bash
# Sign up at https://platform.openai.com/
# Get API key from dashboard

# Set environment variable
echo "export OPENAI_API_KEY='your-api-key-here'" >> ~/.bashrc
source ~/.bashrc

# Test API
python3 -c "import openai; print(openai.Model.list())"
```

**Option B: Local Qwen Model** (for offline/budget-conscious users):
```bash
# Install Transformers
pip install transformers torch accelerate

# Download Qwen model (requires ~15GB disk space for 7B model)
python3 << EOF
from transformers import AutoModelForCausalLM, AutoTokenizer
model = AutoModelForCausalLM.from_pretrained("Qwen/Qwen2.5-7B-Instruct", device_map="auto")
tokenizer = AutoTokenizer.from_pretrained("Qwen/Qwen2.5-7B-Instruct")
print("Qwen model downloaded successfully")
EOF
```

---

## Environment Validation

### Validation Script

Create `validate_setup.sh`:
```bash
#!/bin/bash
# validate_setup.sh - Verify all tools installed correctly

echo "🔍 Validating setup..."

# Check OS
if [ "$(lsb_release -sc)" != "jammy" ]; then
    echo "❌ Ubuntu 22.04 (jammy) required, found: $(lsb_release -sc)"
    exit 1
else
    echo "✅ Ubuntu 22.04 detected"
fi

# Check ROS 2
if ros2 --version &> /dev/null; then
    echo "✅ ROS 2 installed: $(ros2 --version)"
else
    echo "❌ ROS 2 not found"
    exit 1
fi

# Check Gazebo
if gazebo --version &> /dev/null; then
    echo "✅ Gazebo installed: $(gazebo --version | head -n1)"
else
    echo "⚠️  Gazebo not found (optional for some modules)"
fi

# Check Python packages
python3 << EOF
try:
    import cv2
    import numpy
    import whisper
    print("✅ Python packages: opencv, numpy, whisper installed")
except ImportError as e:
    print(f"❌ Missing Python package: {e}")
    exit(1)
EOF

# Check NVIDIA GPU (for Isaac Sim)
if nvidia-smi &> /dev/null; then
    echo "✅ NVIDIA GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader)"
else
    echo "⚠️  NVIDIA GPU not detected (required for Module 3: Isaac Sim)"
fi

# Check Nav2
if ros2 pkg list | grep -q nav2; then
    echo "✅ Nav2 installed"
else
    echo "❌ Nav2 not found"
    exit 1
fi

echo ""
echo "🎉 Setup validation complete!"
echo "You are ready to start Module 1: ROS 2 Robotic Nervous System"
```

**Run Validation**:
```bash
chmod +x validate_setup.sh
./validate_setup.sh
```

---

## Workspace Setup

### Create ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/embodied_ai_ws/src
cd ~/embodied_ai_ws

# Clone companion code repository
cd src
git clone https://github.com/<your-org>/embodied-ai-book-code.git
cd ..

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source workspace (add to ~/.bashrc)
echo "source ~/embodied_ai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify Workspace**:
```bash
ros2 pkg list | grep embodied
# Expected: List of embodied-ai book packages
```

---

## Docker Setup (Alternative to Native Installation)

### Using Pre-Built Docker Images

**Pull ROS 2 Image**:
```bash
# Clone book code repository
git clone https://github.com/<your-org>/embodied-ai-book-code.git
cd embodied-ai-book-code/docker

# Build Docker image
docker build -f Dockerfile.ros2 -t embodied-ai:ros2 .

# Run container
docker run -it --rm \
    --network host \
    -v ~/embodied_ai_ws:/workspace \
    embodied-ai:ros2 \
    /bin/bash

# Inside container:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Isaac Sim Docker** (requires nvidia-docker):
```bash
# Install nvidia-docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update && sudo apt install -y nvidia-docker2
sudo systemctl restart docker

# Run Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.0
docker run --rm -it --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    nvcr.io/nvidia/isaac-sim:2023.1.0
```

---

## Common Setup Issues & Solutions

### Issue 1: ROS 2 commands not found

**Symptoms**: `ros2: command not found`

**Solution**:
```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue 2: Gazebo GPU errors

**Symptoms**: `GLX extension not found` or black screen in Gazebo

**Solution**:
```bash
# Install mesa drivers
sudo apt install libgl1-mesa-glx libgl1-mesa-dri

# For NVIDIA GPUs, ensure drivers installed
nvidia-smi  # Should show GPU info

# Try software rendering (slower)
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

### Issue 3: Isaac Sim won't launch

**Symptoms**: Isaac Sim crashes on startup or shows "No compatible GPU"

**Solution**:
```bash
# Check NVIDIA drivers
nvidia-smi
# Expected: Driver version 525+ and CUDA 11.8+

# Update drivers if needed
sudo apt install nvidia-driver-535
sudo reboot

# Check Vulkan support (required for Isaac Sim)
vulkaninfo | grep "Device Name"
# Should show your GPU
```

### Issue 4: Whisper installation fails

**Symptoms**: `pip install openai-whisper` errors

**Solution**:
```bash
# Install ffmpeg (required for Whisper)
sudo apt install ffmpeg

# Install Rust (required for some Whisper dependencies)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"

# Retry Whisper installation
pip install openai-whisper
```

### Issue 5: Out of disk space

**Symptoms**: Installation fails with "No space left on device"

**Solution**:
```bash
# Check disk usage
df -h

# Clean apt cache
sudo apt clean
sudo apt autoremove

# Remove old kernels
sudo apt autoremove --purge

# If using Docker, clean images/containers
docker system prune -a
```

---

## First Steps: Module 1 Quick Test

### Validate ROS 2 Installation

**Test 1: Create a Simple Publisher**:
```bash
# Create package
cd ~/embodied_ai_ws/src
ros2 pkg create --build-type ament_python hello_ros2 --dependencies rclpy std_msgs

# Build
cd ~/embodied_ai_ws
colcon build --packages-select hello_ros2

# Source
source install/setup.bash

# Run built-in demo
ros2 run demo_nodes_py talker
# Expected: "Publishing: 'Hello World: 1'" messages

# In another terminal:
ros2 run demo_nodes_py listener
# Expected: "I heard: [Hello World: 1]"
```

**Test 2: Visualize in RViz**:
```bash
# Launch RViz
rviz2

# You should see the RViz GUI
# Add displays: Grid, TF, RobotModel (later with URDF)
```

**Test 3: Launch Gazebo with Empty World**:
```bash
ros2 launch gazebo_ros gazebo.launch.py
# Should open Gazebo with empty world
```

**If all three tests pass**: ✅ You're ready to start Module 1!

---

## Optional Tools (Recommended)

### Visual Studio Code with ROS Extension

```bash
# Install VS Code
sudo snap install --classic code

# Install extensions (open VS Code):
# - "ROS" by Microsoft
# - "Python" by Microsoft
# - "Markdown All in One" by Yu Zhang
```

### ROS 2 Development Productivity Tools

```bash
# Install useful ROS 2 tools
sudo apt install ros-humble-rqt-tf-tree  # Visualize TF tree
sudo apt install ros-humble-rqt-graph    # Visualize node graph
sudo apt install ros-humble-plotjuggler-ros  # Plot ROS data
```

### Git Configuration

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

---

## Next Steps

After completing this setup:

1. ✅ **Start Module 1**: Open `docs/module1-ros2/index.md`
2. 📚 **Clone Code Repository**: `git clone https://github.com/<org>/embodied-ai-book-code.git`
3. 🔗 **Join Community**: Discord/Slack for questions and discussions (if available)
4. 📝 **Track Progress**: Use module validation checkpoints to verify learning

**Estimated Time to Complete All Modules**:
- Module 1: 6-10 hours
- Module 2: 8-12 hours
- Module 3: 10-15 hours (includes Isaac Sim setup)
- Module 4: 10-15 hours
- Capstone: 15-25 hours
- **Total**: 60-80 hours over 12-13 weeks (university semester pace)

**Support Resources**:
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Tutorials: https://gazebosim.org/docs
- Nav2 Tutorials: https://navigation.ros.org/tutorials/
- Isaac Sim Docs: https://docs.omniverse.nvidia.com/isaacsim/
- Book GitHub Issues: [link to repo issues page]

---

## Troubleshooting Contact

If you encounter issues not covered in this guide:

1. **Check Module-Specific Troubleshooting**: Each module has a troubleshooting section
2. **Search ROS Answers**: https://answers.ros.org/
3. **Check GitHub Issues**: [Book repository issues page]
4. **Community Forums**: Discord/Slack (if available)

**Ready to Begin?** Proceed to Module 1: ROS 2 as the Robotic Nervous System!
