---
id: course-structure
title: 13-Week Learning Path and Course Structure
sidebar_position: 2
description: Semester-long course timeline mapping modules to weekly topics, labs, and learning outcomes for Physical AI and humanoid robotics.
keywords: [course structure, weekly plan, semester timeline, learning path, capstone course]
---

# 13-Week Learning Path and Course Structure

## Course Overview

This book is designed as a **12-13 week university capstone course** in Physical AI and Humanoid Robotics. Each week combines conceptual learning (reading chapters), hands-on labs (executable code examples), and validation exercises. The course culminates in a portfolio-ready autonomous humanoid demonstration.

**Course Format**:
- **Lecture/Reading**: 2-3 hours per week (chapters + reference materials)
- **Lab Work**: 4-6 hours per week (coding, simulation, exercises)
- **Total Commitment**: 6-10 hours per week
- **Assessment**: Weekly validation checkpoints + final capstone demonstration

**Prerequisites**: Intermediate Python programming, basic linear algebra, Linux command-line familiarity. No prior robotics or ROS experience required.

---

## Phase 1: Foundations (Weeks 1-5)

### Week 1: Introduction and Environment Setup

**Topics**:
- What is Physical AI and embodied intelligence?
- Course structure and learning outcomes
- Hardware requirements and setup options

**Reading**:
- Introduction chapter
- Appendix: Hardware and Lab Infrastructure
- Quickstart Guide

**Lab**:
- Install Ubuntu 22.04 (native, WSL 2, or VM)
- Install ROS 2 Humble, Gazebo, development tools
- Run validation script (`validate_setup.sh`)
- Verify RViz and Gazebo launch successfully

**Deliverable**: Screenshot showing ROS 2 workspace with demo nodes running

**Learning Outcomes**:
- ✅ Understand Physical AI scope and applications
- ✅ Complete development environment setup
- ✅ Verify all prerequisites met

**Assessment**: Environment validation (pass/fail, ungraded)

---

### Week 2: ROS 2 Architecture Fundamentals

**Topics**:
- ROS 2 as robotic middleware
- Nodes, topics, services, actions
- Computational graphs and communication patterns
- DDS and Quality of Service (QoS)

**Reading**:
- Module 1A: ROS 2 Architecture (~1,450 words)

**Lab**:
- Create ROS 2 workspace
- Implement publisher/subscriber for sensor simulation
- Create launch file orchestrating multiple nodes
- Visualize computational graph with `rqt_graph`

**Deliverable**: ROS 2 package with IMU publisher (100 Hz) and subscriber, launched via launch file

**Learning Outcomes**:
- ✅ Explain nodes, topics, services, actions with examples
- ✅ Write ROS 2 publishers and subscribers in Python
- ✅ Design launch files for multi-node systems

**Assessment**: Code submission + quiz on communication patterns (10% of grade)

---

### Week 3: URDF Modeling and Humanoid Kinematics

**Topics**:
- URDF structure (links, joints, kinematic trees)
- Inertial properties and collision geometries
- Sensor attachment (camera, LiDAR, IMU)
- Gazebo plugin integration

**Reading**:
- Module 1B: Python & URDF (~1,480 words)

**Lab**:
- Create 5-link humanoid URDF (torso, head, 2 arms OR 2 legs)
- Add camera and IMU sensors with Gazebo plugins
- Validate with `check_urdf`
- Visualize in RViz with joint sliders
- Load in Gazebo and verify sensor topics publish

**Deliverable**: Humanoid URDF file + screenshot in RViz + sensor topic verification

**Learning Outcomes**:
- ✅ Model robot kinematics using URDF
- ✅ Define realistic inertial properties
- ✅ Integrate sensors for simulation

**Assessment**: URDF validation + sensor data verification (10% of grade)

---

### Week 4: Digital Twin Simulation in Gazebo

**Topics**:
- Digital twin paradigm and simulation benefits
- Gazebo architecture and physics engines (ODE, Bullet, DART)
- World files (SDF) and environment creation
- Realistic sensor simulation with noise models

**Reading**:
- Module 2A: Gazebo Simulation (~1,470 words)

**Lab**:
- Create Gazebo world with obstacles (tables, walls, objects)
- Convert URDF to SDF, spawn humanoid in custom world
- Configure realistic sensor noise (IMU ±0.01 rad/s, LiDAR ±2cm)
- Achieve stable standing simulation (no falling or vibration)

**Deliverable**: Gazebo world file + video showing humanoid standing stably with sensor data

**Learning Outcomes**:
- ✅ Create custom Gazebo environments
- ✅ Tune physics parameters for stable simulation
- ✅ Simulate sensors with realistic noise

**Assessment**: Gazebo world validation + stable simulation demonstration (10% of grade)

---

### Week 5: Unity Integration and Photorealistic Visualization

**Topics**:
- Unity for robotics visualization
- ROS-TCP-Connector architecture
- Hybrid simulation (Gazebo physics + Unity visuals)
- Human-robot interaction visualization

**Reading**:
- Module 2B: Unity Integration (~1,450 words)

**Lab**:
- Install Unity 2021.3 LTS and Unity Robotics Hub
- Import humanoid URDF into Unity
- Configure ROS-TCP-Connector, connect to ROS 2
- Synchronize Gazebo physics with Unity rendering

**Deliverable**: Video showing humanoid motion synchronized between Gazebo (physics) and Unity (visuals)

**Learning Outcomes**:
- ✅ Set up Unity for robotics development
- ✅ Import and configure URDF in Unity
- ✅ Bridge Unity and ROS 2 via TCP connector

**Assessment**: Unity-ROS 2 integration demonstration (10% of grade)

**Phase 1 Checkpoint**: Students have complete digital twin infrastructure (ROS 2 + Gazebo + Unity) and foundational humanoid model.

---

## Phase 2: AI Perception and Cognition (Weeks 6-9)

### Week 6: NVIDIA Isaac Sim and Synthetic Data

**Topics**:
- Isaac Sim overview and installation
- Photorealistic rendering with RTX
- Synthetic dataset generation
- Domain randomization strategies

**Reading**:
- Module 3: Isaac Platform, Part 1 (~700 words from full chapter)

**Lab**:
- Install Isaac Sim (or use cloud GPU instance)
- Import humanoid URDF into Isaac Sim
- Create simple scene with manipulable objects
- Generate 100 synthetic images with domain randomization (lighting, poses)

**Deliverable**: Synthetic dataset (100 images) with bounding box annotations

**Learning Outcomes**:
- ✅ Set up Isaac Sim environment
- ✅ Generate labeled synthetic data
- ✅ Apply domain randomization

**Assessment**: Synthetic dataset quality evaluation (10% of grade)

---

### Week 7: Isaac ROS and GPU-Accelerated Perception

**Topics**:
- Isaac ROS package ecosystem (nvblox, cuVSLAM, DOPE)
- GPU acceleration benefits (6× speedup)
- 3D reconstruction and visual SLAM

**Reading**:
- Module 3: Isaac Platform, Part 2 (~700 words)

**Lab**:
- Install Isaac ROS packages (nvblox, cuVSLAM)
- Launch perception pipeline in Isaac Sim
- Build 3D map with nvblox (visualize in RViz)
- Track robot pose with cuVSLAM

**Deliverable**: Video showing real-time 3D reconstruction (mesh + trajectory) as humanoid explores environment

**Learning Outcomes**:
- ✅ Configure Isaac ROS perception packages
- ✅ Integrate visual SLAM with navigation
- ✅ Validate GPU acceleration performance

**Assessment**: Perception pipeline demonstration (10% of grade)

---

### Week 8: Navigation with Nav2

**Topics**:
- Nav2 architecture (global/local planners, costmaps)
- Bipedal humanoid configuration
- Path planning and obstacle avoidance

**Reading**:
- Module 3: Isaac Platform, Part 3 (~350 words on Nav2)

**Lab**:
- Configure Nav2 for humanoid (footprint, velocity limits)
- Launch Nav2 with nvblox-generated costmap
- Send navigation goals via RViz (2D Nav Goal tool)
- Achieve autonomous navigation with over 80% success rate (5 goals)

**Deliverable**: Video showing autonomous navigation from start to goal, avoiding obstacles

**Learning Outcomes**:
- ✅ Configure Nav2 for bipedal constraints
- ✅ Integrate perception (nvblox) with navigation (Nav2)
- ✅ Debug navigation failures

**Assessment**: Navigation success rate + parameter tuning report (15% of grade)

---

### Week 9: Vision-Language-Action Foundations

**Topics**:
- VLA paradigm and embodied AI
- Whisper speech recognition
- LLM task planning (GPT-4, Qwen)
- Prompt engineering for robotics

**Reading**:
- Module 4: VLA Systems, Parts 1-2 (~900 words)

**Lab**:
- Install Whisper, transcribe 10 voice commands
- Set up OpenAI API or download Qwen model
- Create LLM planner node generating JSON plans
- Test with 10 different commands, validate JSON structure

**Deliverable**: LLM planner node generating valid plans for 10 test commands (over 90% valid JSON)

**Learning Outcomes**:
- ✅ Integrate Whisper ASR with ROS 2
- ✅ Design prompts for robotic task decomposition
- ✅ Parse LLM outputs into structured plans

**Assessment**: Voice-to-plan pipeline demonstration (10% of grade)

**Phase 2 Checkpoint**: Students have AI perception (Isaac ROS), navigation (Nav2), and cognitive planning (LLM) working independently.

---

## Phase 3: Integration and Capstone (Weeks 10-13)

### Week 10: Vision-Language Grounding and Action Execution

**Topics**:
- CLIP for object recognition
- LLaVA for visual question answering
- 3D object localization (depth + bounding boxes)
- Action grounding (JSON → ROS 2 actions)

**Reading**:
- Module 4: VLA Systems, Parts 3-4 (~880 words)

**Lab**:
- Install CLIP and LLaVA
- Implement object grounding (text query → 3D coordinates)
- Create action grounding node (parse plans, send ROS 2 action goals)
- Test end-to-end: Voice command → LLM plan → Object detection → Navigation goal

**Deliverable**: Video showing voice command resulting in autonomous navigation to detected object

**Learning Outcomes**:
- ✅ Ground language to visual objects using CLIP
- ✅ Localize objects in 3D world coordinates
- ✅ Execute ROS 2 actions from LLM plans

**Assessment**: Object grounding accuracy + action execution (10% of grade)

---

### Week 11: System Integration Architecture

**Topics**:
- Modular system design (7-node architecture)
- State machine orchestration (SMACH, BehaviorTree)
- Multi-modal sensor fusion
- Error handling and recovery

**Reading**:
- Capstone chapter (when available) or integration materials

**Lab**:
- Design system architecture (nodes, topics, actions)
- Implement orchestrator node with state machine
- Integrate modules: Perception + Navigation + VLA planning
- Test subsystem pairs (navigation + perception, perception + LLM)

**Deliverable**: System architecture diagram + integration test results

**Learning Outcomes**:
- ✅ Design modular robotic systems
- ✅ Implement state machines for coordination
- ✅ Debug integration issues across subsystems

**Assessment**: Architecture design + integration test report (15% of grade)

---

### Week 12: Capstone Development (Part 1)

**Topics**:
- Complete pipeline integration
- End-to-end testing
- Performance optimization
- Debugging strategies

**Lab**:
- Integrate all 7 nodes: Voice, Planner, Orchestrator, Perception, Navigation, Manipulation, Feedback
- Create master launch file
- Test complete voice-to-action pipeline: "Go to table and pick up red box"
- Measure performance: latencies, success rates, failure modes

**Deliverable**: Functioning end-to-end system (over 50% success rate on 5 test commands)

**Learning Outcomes**:
- ✅ Execute complete voice-to-action pipeline
- ✅ Measure and optimize system performance
- ✅ Handle and debug failures

**Assessment**: System functionality demonstration (progress checkpoint, ungraded)

---

### Week 13: Capstone Demonstration and Portfolio

**Topics**:
- Demo preparation and recording
- Performance evaluation and reporting
- System documentation
- Extensions and future work

**Lab**:
- Record 3-5 minute demo video showing multiple voice commands
- Complete performance evaluation (success rates, latencies)
- Write capstone report (architecture, results, challenges, improvements)
- Prepare presentation (10-minute talk)

**Deliverable**:
1. **Demo video** (3-5 minutes)
2. **Capstone report** (5-10 pages: architecture, results, evaluation)
3. **Presentation slides** (10 minutes)
4. **Code repository** (documented, reproducible)

**Learning Outcomes**:
- ✅ Demonstrate portfolio-quality autonomous humanoid system
- ✅ Articulate architectural decisions and trade-offs
- ✅ Present technical work to peers and stakeholders

**Assessment**: Final capstone demonstration and report (20% of grade)

**Phase 3 Checkpoint**: Students deliver complete autonomous humanoid system demonstrating: speech recognition → task planning → navigation → perception → manipulation → status reporting.

---

## Assessment Breakdown

### Weekly Assessments (60% total)

| Week | Assessment | Weight | Type |
|------|------------|--------|------|
| Week 1 | Environment setup | 0% | Pass/fail (ungraded) |
| Week 2 | ROS 2 publisher/subscriber | 10% | Code + quiz |
| Week 3 | URDF humanoid model | 10% | Validation + visualization |
| Week 4 | Gazebo simulation | 10% | World creation + stability |
| Week 5 | Unity integration | 10% | Synchronization demo |
| Week 6 | Synthetic dataset | 10% | Dataset quality |
| Week 7 | Isaac ROS perception | 10% | Real-time SLAM demo |
| Week 8 | Nav2 navigation | 15% | Success rate + tuning report |
| Week 9 | LLM task planner | 10% | Plan generation accuracy |
| Week 10 | VLA pipeline | 10% | Voice-to-action demo |
| Week 11 | System integration | 15% | Architecture + tests |

### Final Capstone (40% total)

| Component | Weight | Criteria |
|-----------|--------|----------|
| **System Functionality** | 15% | End-to-end success rate (≥60% target) |
| **Code Quality** | 10% | Modular, documented, reproducible |
| **Demo Video** | 5% | Clear demonstration of 3+ voice commands |
| **Technical Report** | 10% | Architecture, evaluation, challenges, extensions |

**Total**: 100%

**Grading Philosophy**: Progressive assessment ensures students master each module before advancing. Capstone weight (40%) reflects integration complexity and portfolio value.

---

## Module-to-Week Alignment

### Module 1: ROS 2 Robotic Nervous System (Weeks 2-3)

**Week 2**: ROS 2 architecture, computational graphs, communication patterns
**Week 3**: Python development (rclpy), URDF modeling, sensor simulation

**Validation**: Functioning ROS 2 workspace with humanoid URDF and sensor publishers

**Time Allocation**: 12-20 hours total (2 weeks × 6-10 hours)

### Module 2: Digital Twin Simulation (Weeks 4-5)

**Week 4**: Gazebo physics simulation, world creation, sensor fidelity
**Week 5**: Unity visualization, ROS-TCP-Connector, hybrid simulation

**Validation**: Humanoid simulated in both Gazebo and Unity with synchronized motion

**Time Allocation**: 12-20 hours total

### Module 3: NVIDIA Isaac AI Platform (Weeks 6-8)

**Week 6**: Isaac Sim setup, photorealistic rendering, synthetic data generation
**Week 7**: Isaac ROS perception (nvblox, cuVSLAM), GPU acceleration
**Week 8**: Nav2 configuration, bipedal path planning, autonomous navigation

**Validation**: Autonomous navigation using Isaac ROS + Nav2 with over 80% success

**Time Allocation**: 18-30 hours total (more complex, includes Isaac Sim learning curve)

### Module 4: Vision-Language-Action (Weeks 9-10)

**Week 9**: Whisper ASR, LLM planning (GPT-4/Qwen), prompt engineering
**Week 10**: CLIP/LLaVA grounding, action execution, complete VLA pipeline

**Validation**: Voice command triggers autonomous navigation or simple manipulation

**Time Allocation**: 12-20 hours total

### Module 5: Capstone Integration (Weeks 11-13)

**Week 11**: System architecture design, multi-module integration, state machine
**Week 12**: End-to-end testing, performance optimization, debugging
**Week 13**: Demo recording, documentation, presentation

**Validation**: Portfolio-quality demonstration with ≥60% end-to-end success rate

**Time Allocation**: 18-30 hours (most intensive phase)

---

## Self-Paced Learning Path

For independent learners (non-university setting):

### Fast Track (8 weeks, 15-20 hours/week)

**Weeks 1-2**: Module 1 (ROS 2 + URDF)
**Weeks 3-4**: Module 2 (Gazebo + Unity)
**Weeks 5-6**: Module 3 (Isaac Sim + Isaac ROS + Nav2)
**Week 7**: Module 4 (VLA systems)
**Week 8**: Capstone integration + demo

**Total**: ~120-160 hours intensive

### Standard Pace (13 weeks, 6-10 hours/week)

Follow university course structure above.

**Total**: ~80-130 hours

### Relaxed Pace (20 weeks, 4-6 hours/week)

**Weeks 1-4**: Module 1 (extra time for ROS 2 mastery)
**Weeks 5-8**: Module 2 (practice simulation extensively)
**Weeks 9-13**: Module 3 (Isaac ecosystem deep dive)
**Weeks 14-16**: Module 4 (VLA systems)
**Weeks 17-20**: Capstone (thorough integration)

**Total**: ~80-120 hours spread over 5 months

---

## Learning Outcome Progression

### Beginner (Weeks 1-5): Foundation Competency

**After Phase 1**, you can:
- Create ROS 2 workspaces with multiple communicating nodes
- Model robots using URDF with realistic physics properties
- Simulate robots in Gazebo and Unity
- Verify sensor data flows through ROS 2 topics
- Debug common ROS 2 and simulation issues

**Portfolio Value**: ⭐⭐ (demonstrates ROS 2 basics, foundational for robotics internships)

### Intermediate (Weeks 6-10): AI Integration

**After Phase 2**, you can:
- Generate synthetic training datasets with Isaac Sim
- Deploy GPU-accelerated perception pipelines (Isaac ROS)
- Configure autonomous navigation for humanoid robots
- Integrate LLMs for high-level task planning
- Ground natural language to visual objects and 3D coordinates

**Portfolio Value**: ⭐⭐⭐⭐ (demonstrates cutting-edge AI+robotics skills, competitive for robotics positions)

### Advanced (Weeks 11-13): System Integration

**After Phase 3**, you can:
- Design and implement multi-node robotic architectures
- Integrate perception, planning, and control into unified systems
- Debug complex integration issues across heterogeneous components
- Demonstrate autonomous behaviors via natural language commands
- Evaluate system performance and propose improvements

**Portfolio Value**: ⭐⭐⭐⭐⭐ (portfolio-ready capstone, suitable for graduate admissions, industry senior roles)

---

## Optional Extensions (Post-Course)

For students completing early or seeking additional challenges:

### Extension 1: Reinforcement Learning for Locomotion

**Time**: +2-3 weeks
**Topics**: Isaac Gym, PPO training, reward shaping, Sim2Real transfer
**Outcome**: Trained bipedal walking policy

### Extension 2: Dexterous Manipulation

**Time**: +2-4 weeks
**Topics**: Multi-finger hands, grasp planning, contact-rich manipulation
**Outcome**: Complex grasping (opening bottles, using tools)

### Extension 3: Multi-Robot Coordination

**Time**: +2-3 weeks
**Topics**: Namespaces, distributed planning, fleet management
**Outcome**: 2-3 humanoids collaborating on task

### Extension 4: Physical Hardware Deployment

**Time**: +4-8 weeks (hardware-dependent)
**Topics**: Jetson deployment, Sim2Real tuning, safety systems
**Outcome**: Algorithm validated on physical humanoid

**Recommendation**: Complete core capstone first, then pursue one extension as independent study or thesis project.

---

## Instructor Notes

### For University Adoption

**Prerequisites to Enforce**:
- Python proficiency test (functions, classes, file I/O)
- Linux familiarity quiz (bash commands, file navigation)
- Linear algebra check (vector/matrix operations)

**Hardware Lab Requirement**:
- 5-10 workstations with RTX 3060+ GPUs
- Institutional cloud credits (AWS Educate: $100-200 per student)
- **Alternative**: Students use personal hardware or cloud (cost: $40-90)

**Teaching Assistants**:
- 1 TA per 10-15 students recommended
- TA duties: Office hours (debugging ROS 2, Isaac Sim), grading code submissions

**Collaboration Policy**:
- Weeks 1-10: Individual work (build personal competency)
- Weeks 11-13: Optional pairs (capstone can be collaborative, simulates team development)

### Grading Rubrics

**Code Assignments** (Weeks 2-10):
- **Functionality** (60%): Code executes, meets requirements
- **Code Quality** (20%): PEP 8 compliance, comments, modularity
- **Documentation** (20%): README with usage instructions, clear explanations

**Capstone** (Weeks 11-13):
- **System Functionality** (40%): End-to-end success rate, robustness
- **Code Quality** (20%): Modular, well-documented, reproducible
- **Technical Report** (25%): Architecture clarity, evaluation thoroughness, insights
- **Presentation** (15%): Communication clarity, demo effectiveness

---

## Summary

This 13-week course structure provides:

✅ **Progressive learning**: Foundations (Weeks 1-5) → AI Integration (Weeks 6-10) → Capstone (Weeks 11-13)
✅ **Hands-on validation**: Weekly labs with deliverables ensure practical skill development
✅ **Flexible pacing**: Self-paced options (8 weeks fast, 20 weeks relaxed)
✅ **Clear assessment**: Grading criteria for each weekly assignment and final capstone (100% total)
✅ **Portfolio outcome**: Capstone demonstration suitable for graduate admissions or industry interviews
✅ **Accessibility**: Cloud GPU option ($40-90) or local workstation ($1,400), no mandatory physical robot

**Success Metric**: Students completing all 13 weeks will have mastered Physical AI fundamentals, built an autonomous humanoid system in simulation, and developed portfolio-ready demonstrations showcasing:
- ROS 2 system design
- Digital twin simulation (Gazebo, Unity, Isaac Sim)
- AI perception (Isaac ROS, VSLAM, object detection)
- Cognitive planning (VLA with LLMs)
- End-to-end system integration

**Next Steps**: Begin Week 1 by completing environment setup, then proceed sequentially through modules. Remember: **80% of learning happens in the lab**—prioritize hands-on exercises over passive reading.

---

## References

Macenski, S., Martín, F., White, R., & Ginés Clavero, J. (2020). The Marathon 2: A navigation system. *Proceedings of the 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2718-2725.

NVIDIA (2023). *Isaac Sim Documentation*. NVIDIA Developer. https://docs.omniverse.nvidia.com/isaacsim/latest/

Rudin, N., Hoeller, D., Reist, P., & Hutter, M. (2022). Learning to walk in minutes using massively parallel deep reinforcement learning. *Proceedings of the 6th Conference on Robot Learning (CoRL)*, 91-100.

Unity Technologies (2023). *Unity Robotics Hub*. GitHub Repository. https://github.com/Unity-Technologies/Unity-Robotics-Hub
