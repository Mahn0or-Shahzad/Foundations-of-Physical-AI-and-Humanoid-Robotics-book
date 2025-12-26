# Implementation Plan: Embodied AI & Humanoid Robotics Book

**Branch**: `001-embodied-ai-book` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-embodied-ai-book/spec.md`

## Summary

This plan outlines the development of a research-grade technical book titled "Embodied AI & Humanoid Robotics" structured as a 12-13 week capstone course. The book integrates ROS 2 fundamentals, digital twin simulation (Gazebo/Unity), NVIDIA Isaac AI platform, and Vision-Language-Action systems into a comprehensive educational resource for intermediate-to-advanced students and professionals transitioning to Physical AI and humanoid robotics.

**Primary Requirement**: Create Docusaurus-formatted Markdown chapters (10,000-15,000 words total) covering 5 progressive modules with executable code examples, mathematical formulations, and a capstone integration project.

**Technical Approach**: Specification-first authoring using Spec-Kit Plus workflow, with each module treated as independently testable content unit. Content will be validated against constitution requirements (scientific rigor, reproducibility, citation standards) before publication.

## Technical Context

**Language/Version**:
- Python 3.10+ (for ROS 2 Humble/Iron compatibility)
- Markdown (CommonMark/GFM for Docusaurus)
- ROS 2 Humble LTS (primary) or Iron (secondary)

**Primary Dependencies**:
- **Content Authoring**: Docusaurus 3.x, MDX, Mermaid (diagrams)
- **Code Examples**: ROS 2 Humble/Iron, rclpy, Gazebo Fortress/Classic, Unity 2021.3 LTS+, NVIDIA Isaac Sim 2023.1.0+
- **AI/ML**: OpenAI Whisper, GPT-4 API or Qwen (local), Isaac ROS packages, CLIP, LLaVA
- **Navigation**: Nav2 (ROS 2), AMCL, DWB, Costmap2D
- **Perception**: nvblox, cuVSLAM, YOLO, DOPE, depth processing libraries
- **Reference Management**: Zotero or Mendeley (APA 7th edition)
- **Plagiarism Detection**: Turnitin or iThenticate (pre-publication)

**Storage**:
- Git repository with semantic versioning
- Companion code repository (GitHub) with per-module examples
- Docusaurus static site deployment (GitHub Pages)
- Optional: PDF/EPUB export artifacts

**Testing**:
- **Code Validation**: pytest for Python utilities, ROS 2 launch tests for node integration
- **Content Validation**: Markdown linters, Docusaurus build validation, link checkers
- **Citation Validation**: Reference format checkers, DOI/arXiv link verification
- **Plagiarism Detection**: Pre-publication scan with zero-tolerance threshold

**Target Platform**:
- **Development**: Ubuntu 22.04 LTS (for ROS 2 Humble)
- **Simulation**: Local RTX workstations (RTX 3060+) or cloud GPU instances (AWS g4dn/g5, Azure NC-series)
- **Deployment**: Docusaurus static site (web), PDF export (academic distribution), optional EPUB

**Project Type**: Technical book / Documentation site (Docusaurus-based)

**Performance Goals**:
- Docusaurus build time: <2 minutes for full site
- Code example execution: 95% success rate on clean Ubuntu 22.04 + ROS 2 Humble
- Reader completion rate: 70%+ for capstone project (based on module design)
- Citation verification: 100% traceable and properly formatted

**Constraints**:
- Total word count: 10,000-15,000 words (excluding code blocks and appendices)
- Module word counts: M1 (2k-3k), M2 (2k-3k), M3 (2.5k-3.5k), M4 (3k-4k), Capstone (3.5k-5k)
- Minimum 25 academically validated sources, ≥50% peer-reviewed
- Readability: Flesch-Kincaid Grade Level 10-12
- All code must run on Ubuntu 22.04 + ROS 2 Humble without modification
- Hardware accessibility: Solutions viable within university lab budgets (<$200 cloud compute for full course)

**Scale/Scope**:
- 5 main modules + Introduction + Appendices = ~8-10 Docusaurus pages
- 45 acceptance scenarios across all modules (testable learning outcomes)
- ~50-75 code examples (ROS 2 nodes, URDF files, launch files, Python scripts, config files)
- ~20-30 diagrams (system architecture, data flow, state machines, perception pipelines)
- 25+ academic references (IEEE, ACM, Springer, Nature Robotics, IJRR, ICRA, IROS, RSS)
- Companion GitHub repository with ~10-15 ROS 2 packages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Scientific Correctness & Rigor ✅
- **Status**: PASS
- **Validation**: All mathematical formulations (FK/IK, dynamics, ZMP, control laws) will be validated against authoritative robotics textbooks and peer-reviewed papers. ROS 2 conventions follow REPs 103, 105.
- **Action**: Research phase will identify authoritative sources for each technical claim. Plan includes numerical validation checkpoints.

### Principle II: Reproducibility & Verifiability ✅
- **Status**: PASS
- **Validation**: All code examples will be tested on Ubuntu 22.04 + ROS 2 Humble. Simulation environments will specify exact versions and configurations.
- **Action**: Phase 1 will define test harness for code validation. Companion repository will include CI/CD pipeline for continuous validation.

### Principle III: Academic Citation Standards ✅
- **Status**: PASS
- **Validation**: Minimum 25 sources with ≥50% peer-reviewed. APA 7th edition enforced via reference management tools.
- **Action**: Research phase will curate citation library. Each module requires minimum 5 sources (35 total > 25 minimum).

### Principle IV: Conceptual Clarity & Accessibility ✅
- **Status**: PASS
- **Validation**: Target FK Grade 10-12. Progressive complexity from ROS 2 basics → VLA integration. Visual aids for complex concepts.
- **Action**: Plan includes diagram requirements for each module. Readability assessment in review phase.

### Principle V: Specification-First Authoring ✅
- **Status**: PASS - Following SDD workflow
- **Validation**: This plan document fulfills planning requirement. Tasks.md will follow. PHRs will be created for significant sessions.
- **Action**: Proceed with Phase 0-1 as specified.

### Principle VI: Code Quality & Executability ✅
- **Status**: PASS
- **Validation**: PEP 8 for Python, ROS 2 conventions, executable without modification, security-conscious.
- **Action**: Code examples will be validated via pytest and ROS 2 launch tests. Plan includes code review checkpoints.

### Principle VII: Plagiarism Zero-Tolerance ✅
- **Status**: PASS
- **Validation**: Pre-publication Turnitin/iThenticate scan required. Zero unattributed overlap.
- **Action**: Plagiarism scan is final gate before v1.0 release.

### Principle VIII: Version Control & Traceability ✅
- **Status**: PASS
- **Validation**: Git with semantic versioning. Commit messages reference specifications.
- **Action**: All content development will be version-controlled with clear commit messages.

**Constitution Check Result**: ✅ **PASS** - All 8 principles satisfied. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/001-embodied-ai-book/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Technology research and best practices
├── data-model.md        # Phase 1: Book structure, URDF models, data schemas
├── quickstart.md        # Phase 1: Reader setup guide
├── contracts/           # Phase 1: Module interfaces and dependencies
│   ├── module-interfaces.md
│   └── ros2-message-contracts.md
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── tasks.md             # Phase 2: Actionable authoring tasks (/sp.tasks output)
```

### Source Code (Docusaurus book + companion repository)

**Docusaurus Book Structure** (this repository):
```text
docs/                          # Docusaurus content root
├── intro.md                   # Introduction to Physical AI and book structure
├── module1-ros2/              # Module 1: ROS 2 Robotic Nervous System
│   ├── index.md               # Module overview and learning objectives
│   ├── ros2-architecture.md   # Nodes, topics, services, actions
│   ├── rclpy-development.md   # Python development with rclpy
│   ├── urdf-modeling.md       # URDF for humanoid robots
│   └── exercises.md           # Hands-on exercises and validation
├── module2-digital-twin/      # Module 2: Digital Twin Simulation
│   ├── index.md
│   ├── gazebo-simulation.md   # Gazebo fundamentals
│   ├── unity-integration.md   # Unity visualization
│   ├── sensor-simulation.md   # LiDAR, cameras, IMU
│   └── exercises.md
├── module3-isaac/             # Module 3: NVIDIA Isaac AI Platform
│   ├── index.md
│   ├── isaac-sim.md           # Photorealistic simulation
│   ├── isaac-ros.md           # Hardware-accelerated perception
│   ├── nav2-integration.md    # Humanoid navigation
│   ├── rl-locomotion.md       # Reinforcement learning
│   └── exercises.md
├── module4-vla/               # Module 4: Vision-Language-Action
│   ├── index.md
│   ├── whisper-integration.md # Speech recognition
│   ├── llm-planning.md        # Cognitive planning
│   ├── vision-language.md     # VLM integration
│   ├── action-grounding.md    # ROS 2 action execution
│   └── exercises.md
├── capstone/                  # Module 5: End-to-End Capstone
│   ├── index.md
│   ├── architecture.md        # System design
│   ├── integration.md         # Module integration
│   ├── state-machine.md       # Orchestration
│   ├── debugging.md           # Troubleshooting guide
│   └── demo-guide.md          # Demo preparation
├── appendices/
│   ├── python-refresher.md    # Python basics
│   ├── linear-algebra.md      # Math fundamentals
│   ├── hardware-guide.md      # Setup recommendations
│   └── glossary.md            # Terminology
└── references.md              # Complete bibliography (APA 7th)

docusaurus.config.js           # Docusaurus configuration
sidebars.js                    # Sidebar navigation structure
static/                        # Images, diagrams, videos
├── diagrams/
│   ├── ros2-computational-graph.svg
│   ├── digital-twin-pipeline.svg
│   ├── isaac-perception-pipeline.svg
│   ├── vla-architecture.svg
│   └── capstone-system-diagram.svg
└── videos/                    # Demo videos (embedded or linked)
```

**Companion Code Repository** (separate GitHub repo):
```text
embodied-ai-book-code/
├── module1-ros2/
│   ├── examples/
│   │   ├── hello_ros2/        # Basic publisher/subscriber
│   │   ├── humanoid_urdf/     # Sample URDF models
│   │   └── sensor_publisher/  # Sensor simulation
│   ├── exercises/
│   └── README.md
├── module2-digital-twins/
│   ├── gazebo_worlds/         # SDF world files
│   ├── unity_scenes/          # Unity project files
│   └── README.md
├── module3-isaac-sim/
│   ├── isaac_sim_scripts/     # Python scripts for Isaac Sim
│   ├── isaac_ros_launch/      # Launch files for Isaac ROS
│   ├── nav2_configs/          # Nav2 parameter files
│   └── README.md
├── module4-vla/
│   ├── whisper_ros2/          # Whisper integration
│   ├── llm_planner/           # LLM-ROS 2 bridge
│   ├── vision_language/       # CLIP, LLaVA examples
│   └── README.md
├── capstone/
│   ├── complete_system/       # Full integration workspace
│   ├── launch/                # Launch files
│   ├── configs/               # All configuration files
│   └── README.md
├── docker/
│   ├── Dockerfile.ros2        # ROS 2 Humble environment
│   ├── Dockerfile.isaac       # Isaac Sim environment
│   └── docker-compose.yml
└── README.md                  # Repository overview
```

**Structure Decision**:
This book uses a **Documentation site + Companion code repository** structure. The main repository contains Docusaurus Markdown content organized by module, while a separate GitHub repository hosts all executable code examples, URDF models, configuration files, and Docker environments. This separation ensures:
1. Clean content vs. code organization
2. Independent versioning of documentation and code
3. CI/CD validation of code examples without cluttering docs repo
4. Easy forking of code repo by readers

## Complexity Tracking

> **No violations identified.** Constitution check passed for all principles. No complexity justifications required.

---

## Phase 0: Research & Technology Validation

**Objective**: Resolve all technical unknowns, validate tooling choices, and establish authoritative sources for each module.

### Research Tasks

1. **ROS 2 Best Practices Research**
   - Topic: Current ROS 2 Humble best practices for humanoid robot control
   - Deliverable: Document recommended patterns for nodes, topics, services, actions
   - Sources: ROS 2 documentation, REPs, robotics textbooks

2. **URDF/SDF Modeling Standards**
   - Topic: Humanoid robot description standards (DH parameters, coordinate frames, inertial properties)
   - Deliverable: Reference URDF template with proper conventions
   - Sources: ROS/Gazebo documentation, robotics kinematics textbooks

3. **Gazebo vs Unity Comparison**
   - Topic: Physics engine accuracy, sensor simulation fidelity, performance benchmarks
   - Deliverable: Comparison matrix for simulation platform selection
   - Sources: Gazebo/Unity documentation, simulation research papers

4. **NVIDIA Isaac Ecosystem**
   - Topic: Isaac Sim capabilities, Isaac ROS package ecosystem, hardware requirements
   - Deliverable: Isaac platform capabilities matrix and setup guide
   - Sources: NVIDIA official documentation, Isaac Sim tutorials

5. **Nav2 for Bipedal Humanoids**
   - Topic: Nav2 configuration for bipedal locomotion (vs wheeled robots)
   - Deliverable: Nav2 parameter tuning guidelines for humanoid constraints
   - Sources: Nav2 documentation, bipedal navigation papers

6. **Vision-Language-Action Architectures**
   - Topic: Current VLA research, LLM-robotics integration patterns, safety considerations
   - Deliverable: VLA architecture patterns and prompt engineering best practices
   - Sources: Recent VLA papers (2023-2024), LLM robotics papers, embodied AI literature

7. **Reinforcement Learning for Locomotion**
   - Topic: PPO/SAC for bipedal walking, reward shaping, sim-to-real transfer
   - Deliverable: RL training pipeline recommendations
   - Sources: RL locomotion papers, Isaac Gym documentation

8. **Academic Source Curation**
   - Topic: Identify 25+ authoritative sources (≥50% peer-reviewed)
   - Deliverable: Annotated bibliography with source categorization
   - Sources: IEEE Xplore, ACM Digital Library, arXiv, Springer Robotics

### Research Output

**Deliverable**: `research.md` containing:
- Technology decisions with rationale
- Authoritative source citations for each module
- Best practice recommendations
- Known limitations and workarounds
- Hardware requirement validation

---

## Phase 1: Data Models & Contracts

**Objective**: Define book structure, URDF models, ROS 2 message contracts, and module interfaces.

### Data Model: Book Structure

**Entity**: Book Chapter
- **Attributes**: title, module_number, word_count_target, learning_objectives[], acceptance_scenarios[], code_examples[], diagrams[], references[]
- **Relationships**: Chapters belong to Modules; Modules have prerequisites
- **Validation**: Word count within target ±10%, all learning objectives testable, all references APA-compliant

**Entity**: Code Example
- **Attributes**: example_id, module, language, dependencies[], file_path, validation_status
- **Relationships**: Code examples referenced by Chapters
- **Validation**: Executable on Ubuntu 22.04 + ROS 2 Humble, passes pytest/launch tests

**Entity**: URDF Humanoid Model
- **Attributes**: model_name, joints[], links[], sensors[], actuators[], inertial_properties
- **Relationships**: Used across multiple modules (M1, M2, M3, Capstone)
- **Validation**: Valid URDF XML, loads in Gazebo/Isaac Sim without errors

### Contracts: Module Interfaces

**Module 1 → Module 2 Interface**:
- **Output**: Humanoid URDF model with joints, links, sensors defined
- **Input**: None (foundational module)
- **Contract**: URDF model must be importable into Gazebo and Unity

**Module 2 → Module 3 Interface**:
- **Output**: Digital twin environment (Gazebo world or Unity scene) with humanoid
- **Input**: URDF model from Module 1
- **Contract**: Simulation environment compatible with Isaac Sim import or Nav2 integration

**Module 3 → Module 4 Interface**:
- **Output**: Perception pipeline (Isaac ROS or equivalent) providing object detections, VSLAM map
- **Input**: Sensor data from Module 2 simulations
- **Contract**: ROS 2 topics publishing standard message types (sensor_msgs, geometry_msgs, nav_msgs)

**Module 4 → Capstone Interface**:
- **Output**: VLA system components (Whisper node, LLM planner, action grounding)
- **Input**: Perception data from Module 3, ROS 2 action servers
- **Contract**: Voice command → structured action plan → ROS 2 action execution

### ROS 2 Message Contracts

**Speech Command Topic**: `/speech_command` (std_msgs/String)
**Task Plan Topic**: `/task_plan` (custom JSON structure as String or custom msg)
**Detected Objects Topic**: `/detected_objects` (vision_msgs/Detection3DArray)
**Navigation Goal Action**: `navigate_to_pose` (nav2_msgs/NavigateToPose)
**Grasp Action**: `grasp_object` (control_msgs/GripperCommand or custom action)
**Task Status Topic**: `/task_status` (std_msgs/String or custom status msg)

### Deliverables

1. **`data-model.md`**: Complete book structure, URDF model specifications, ROS 2 message schemas
2. **`contracts/module-interfaces.md`**: Module dependency graph, interface definitions
3. **`contracts/ros2-message-contracts.md`**: ROS 2 topic/service/action specifications
4. **`quickstart.md`**: Reader environment setup guide (Ubuntu 22.04, ROS 2 Humble, Gazebo, Isaac Sim, dependencies)

---

## Next Steps (Post-Planning)

### Phase 2: Task Generation (`/sp.tasks`)

After this planning phase, run `/sp.tasks` to generate actionable authoring tasks organized by module:

- **Setup Tasks**: Docusaurus initialization, repository structure, companion code repo
- **Module 1 Tasks**: Write ROS 2 chapters, create code examples, develop URDF models
- **Module 2 Tasks**: Write simulation chapters, create Gazebo worlds, Unity integration
- **Module 3 Tasks**: Write Isaac chapters, develop Isaac Sim scripts, Nav2 configs
- **Module 4 Tasks**: Write VLA chapters, implement Whisper-LLM pipeline, vision-language examples
- **Capstone Tasks**: Write integration chapters, develop complete system, create demo
- **Review Tasks**: Citation verification, plagiarism scan, code validation, readability assessment

### Phase 3: Implementation (`/sp.implement`)

Execute tasks in dependency order:
1. Setup → Module 1 → Module 2 → Module 3 → Module 4 → Capstone
2. Parallel work possible within modules (code examples, diagrams, prose can be developed concurrently)
3. Validation checkpoints after each module

### Phase 4: Publication

- Docusaurus build and GitHub Pages deployment
- PDF export generation
- Final plagiarism scan and citation audit
- Expert review coordination

---

**Planning Complete**: This plan is ready for task generation. All technical context defined, constitution gates passed, research tasks identified, data models specified, and module interfaces contracted.
