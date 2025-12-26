# Tasks: Embodied AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-embodied-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by user story (module) to enable independent implementation and testing of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/module this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `docs/` (Markdown chapters)
- **Code examples**: Separate companion repository `embodied-ai-book-code/`
- **Diagrams**: `static/diagrams/` (SVG, PNG)
- **Specifications**: `specs/001-embodied-ai-book/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, Docusaurus setup, companion repository structure

- [ ] T001 Initialize Docusaurus project with npm (docusaurus.config.js, package.json)
- [ ] T002 [P] Configure sidebar navigation in sidebars.js for all 5 modules
- [ ] T003 [P] Create directory structure for all modules in docs/ (module1-ros2, module2-digital-twin, module3-isaac, module4-vla, capstone, appendices)
- [ ] T004 [P] Initialize companion code repository with module directories (module1-ros2/, module2-digital-twins/, module3-isaac-sim/, module4-vla/, capstone/)
- [ ] T005 [P] Create Docker environment files (Dockerfile.ros2, Dockerfile.isaac, docker-compose.yml)
- [ ] T006 Set up Git repository structure with .gitignore for Docusaurus and node_modules
- [ ] T007 [P] Create placeholder README.md files for each module in companion repo
- [ ] T008 [P] Set up CI/CD pipeline (GitHub Actions) for Docusaurus build validation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY module content can be authored

**⚠️ CRITICAL**: No module writing can begin until this phase is complete

- [ ] T009 Create Zotero reference library with 35 curated sources from research.md
- [ ] T010 [P] Set up reference management workflow (Zotero → BibTeX → APA 7th formatting)
- [ ] T011 [P] Create citation validation script to check APA formatting and DOI/arXiv links
- [ ] T012 [P] Create word count tracking script for monitoring module lengths
- [ ] T013 [P] Set up plagiarism detection workflow (Turnitin or iThenticate access)
- [ ] T014 [P] Create Markdown linter configuration for consistent formatting
- [ ] T015 Create humanoid URDF template (simple_humanoid.urdf) with 15 DOF structure per data-model.md
- [ ] T016 [P] Create validation script (validate_urdf.sh) to check URDF correctness
- [ ] T017 [P] Design core diagrams template (SVG) for system architecture diagrams

**Checkpoint**: Foundation ready - module content authoring can now begin in parallel

---

## Phase 3: User Story 1 - Module 1: ROS 2 Robotic Nervous System (Priority: P1) 🎯 MVP

**Goal**: Complete foundational ROS 2 module enabling readers to create workspaces, understand communication patterns, and model humanoid robots in URDF.

**Independent Test**: Reader can install ROS 2 Humble, create workspace with communicating nodes, define humanoid URDF, launch in Gazebo, and control joints via topics/services.

### Implementation for Module 1

- [ ] T018 [P] [US1] Write docs/module1-ros2/index.md (module overview, learning objectives, prerequisites) ~400 words
- [ ] T019 [P] [US1] Write docs/module1-ros2/ros2-architecture.md (nodes, topics, services, actions, computational graph) ~600 words
- [ ] T020 [P] [US1] Write docs/module1-ros2/rclpy-development.md (Python ROS 2 development, workspace setup, package creation) ~700 words
- [ ] T021 [P] [US1] Write docs/module1-ros2/urdf-modeling.md (URDF for humanoids, joints, links, sensors) ~800 words
- [ ] T022 [P] [US1] Write docs/module1-ros2/exercises.md (hands-on exercises, validation checkpoints) ~500 words
- [ ] T023 [P] [US1] Create ROS 2 computational graph diagram in static/diagrams/ros2-computational-graph.svg
- [ ] T024 [P] [US1] Create humanoid kinematic tree diagram in static/diagrams/humanoid-kinematics.svg
- [ ] T025 [P] [US1] Create code example: hello_ros2 publisher/subscriber in companion repo module1-ros2/examples/hello_ros2/
- [ ] T026 [P] [US1] Create code example: joint_state_publisher in companion repo module1-ros2/examples/joint_state_publisher/
- [ ] T027 [P] [US1] Create code example: sensor_simulator (IMU, camera) in companion repo module1-ros2/examples/sensor_simulator/
- [ ] T028 [P] [US1] Create code example: simple_controller (joint position control) in companion repo module1-ros2/examples/simple_controller/
- [ ] T029 [P] [US1] Complete simple_humanoid.urdf with sensors (camera, LiDAR, IMU) in companion repo module1-ros2/examples/humanoid_urdf/
- [ ] T030 [P] [US1] Create launch file examples (gazebo_launch.py, rviz_launch.py) in companion repo module1-ros2/examples/launch_files/
- [ ] T031 [P] [US1] Write pytest tests for ROS 2 nodes in companion repo module1-ros2/tests/
- [ ] T032 [US1] Add 5 ROS 2 authoritative references to docs/module1-ros2/ chapters (inline citations APA 7th)
- [ ] T033 [US1] Validate Module 1 word count (target 2000-3000 words)
- [ ] T034 [US1] Run Docusaurus build test for Module 1 pages
- [ ] T035 [US1] Create Module 1 README.md in companion repo with setup instructions

**Checkpoint**: Module 1 complete and independently testable

---

## Phase 4: User Story 2 - Module 2: Digital Twin Simulation (Priority: P2)

**Goal**: Complete digital twin module enabling readers to simulate humanoids in Gazebo and Unity with realistic physics and sensors.

**Independent Test**: Reader can create humanoid in both Gazebo and Unity, configure sensors (LiDAR, depth camera, IMU), verify sensor data on ROS 2 topics in both platforms.

### Implementation for Module 2

- [ ] T036 [P] [US2] Write docs/module2-digital-twin/index.md (module overview, digital twin concepts) ~400 words
- [ ] T037 [P] [US2] Write docs/module2-digital-twin/gazebo-simulation.md (Gazebo fundamentals, SDF, world files, plugins) ~700 words
- [ ] T038 [P] [US2] Write docs/module2-digital-twin/unity-integration.md (Unity setup, URDF import, ROS-TCP-Connector) ~600 words
- [ ] T039 [P] [US2] Write docs/module2-digital-twin/sensor-simulation.md (LiDAR, cameras, IMU, force-torque) ~800 words
- [ ] T040 [P] [US2] Write docs/module2-digital-twin/exercises.md (Gazebo vs Unity comparison exercises) ~500 words
- [ ] T041 [P] [US2] Create digital twin pipeline diagram in static/diagrams/digital-twin-pipeline.svg
- [ ] T042 [P] [US2] Create Gazebo vs Unity comparison table diagram in static/diagrams/gazebo-unity-comparison.svg
- [ ] T043 [P] [US2] Create code example: Convert URDF to SDF (conversion script) in companion repo module2-digital-twins/gazebo_worlds/
- [ ] T044 [P] [US2] Create Gazebo world file (warehouse_world.sdf) with obstacles and objects in companion repo module2-digital-twins/gazebo_worlds/
- [ ] T045 [P] [US2] Create Gazebo launch file (warehouse.launch.py) in companion repo module2-digital-twins/gazebo_worlds/
- [ ] T046 [P] [US2] Create Unity scene project files (WarehouseScene.unity) in companion repo module2-digital-twins/unity_scenes/
- [ ] T047 [P] [US2] Create ROS-TCP-Connector setup script in companion repo module2-digital-twins/unity_scenes/
- [ ] T048 [P] [US2] Create sensor visualization examples (RViz configs, PlotJuggler) in companion repo module2-digital-twins/examples/
- [ ] T049 [P] [US2] Create physics comparison test script (measure collision accuracy, friction) in companion repo module2-digital-twins/tests/
- [ ] T050 [US2] Add 5 simulation authoritative references to docs/module2-digital-twin/ chapters
- [ ] T051 [US2] Validate Module 2 word count (target 2000-3000 words)
- [ ] T052 [US2] Test Gazebo and Unity examples for reproducibility
- [ ] T053 [US2] Create Module 2 README.md in companion repo

**Checkpoint**: Module 2 complete and independently testable

---

## Phase 5: User Story 3 - Module 3: NVIDIA Isaac AI Platform (Priority: P3)

**Goal**: Complete Isaac module enabling readers to use photorealistic simulation, GPU-accelerated perception, Nav2 navigation, and RL for locomotion.

**Independent Test**: Reader can create Isaac Sim scene with humanoid, configure sensors for synthetic data, integrate Isaac ROS perception, enable Nav2 navigation with VSLAM, and train basic RL locomotion policy.

### Implementation for Module 3

- [ ] T054 [P] [US3] Write docs/module3-isaac/index.md (module overview, Isaac ecosystem intro) ~400 words
- [ ] T055 [P] [US3] Write docs/module3-isaac/isaac-sim.md (Isaac Sim setup, USD scenes, synthetic data generation) ~800 words
- [ ] T056 [P] [US3] Write docs/module3-isaac/isaac-ros.md (Isaac ROS packages: nvblox, cuVSLAM, DOPE, depth processing) ~700 words
- [ ] T057 [P] [US3] Write docs/module3-isaac/nav2-integration.md (Nav2 for bipedal humanoids, path planning, costmaps) ~700 words
- [ ] T058 [P] [US3] Write docs/module3-isaac/rl-locomotion.md (Isaac Gym, PPO training, reward shaping, sim-to-real) ~600 words
- [ ] T059 [P] [US3] Write docs/module3-isaac/exercises.md (perception validation, navigation tests, RL training exercises) ~500 words
- [ ] T060 [P] [US3] Create Isaac perception pipeline diagram in static/diagrams/isaac-perception-pipeline.svg
- [ ] T061 [P] [US3] Create Nav2 architecture diagram in static/diagrams/nav2-architecture.svg
- [ ] T062 [P] [US3] Create code example: Isaac Sim basic scene setup (Python script) in companion repo module3-isaac-sim/isaac_sim_scripts/
- [ ] T063 [P] [US3] Create code example: Import humanoid URDF into Isaac Sim in companion repo module3-isaac-sim/isaac_sim_scripts/
- [ ] T064 [P] [US3] Create code example: Synthetic data generation with domain randomization in companion repo module3-isaac-sim/isaac_sim_scripts/
- [ ] T065 [P] [US3] Create Isaac ROS launch file (nvblox + cuVSLAM) in companion repo module3-isaac-sim/isaac_ros_launch/
- [ ] T066 [P] [US3] Create Isaac ROS object detection launch (DOPE) in companion repo module3-isaac-sim/isaac_ros_launch/
- [ ] T067 [P] [US3] Create Nav2 parameter files for humanoid in companion repo module3-isaac-sim/nav2_configs/
- [ ] T068 [P] [US3] Create Nav2 launch file (humanoid_navigation.launch.py) in companion repo module3-isaac-sim/nav2_configs/
- [ ] T069 [P] [US3] Create RL environment script (Isaac Gym) in companion repo module3-isaac-sim/rl_training/
- [ ] T070 [P] [US3] Create PPO training script for bipedal locomotion in companion repo module3-isaac-sim/rl_training/
- [ ] T071 [P] [US3] Create policy evaluation script in companion repo module3-isaac-sim/rl_training/
- [ ] T072 [US3] Add 5 Isaac/perception/RL authoritative references to docs/module3-isaac/ chapters
- [ ] T073 [US3] Validate Module 3 word count (target 2500-3500 words)
- [ ] T074 [US3] Test Isaac Sim examples for reproducibility (requires RTX GPU or cloud instance)
- [ ] T075 [US3] Create Module 3 README.md with hardware requirements and Isaac Sim setup

**Checkpoint**: Module 3 complete and independently testable

---

## Phase 6: User Story 4 - Module 4: Vision-Language-Action Systems (Priority: P4)

**Goal**: Complete VLA module enabling readers to implement voice-to-action pipelines with LLMs, vision-language grounding, and ROS 2 integration.

**Independent Test**: Reader can implement Whisper ASR → LLM planner → ROS 2 action execution pipeline with vision-language object grounding.

### Implementation for Module 4

- [ ] T076 [P] [US4] Write docs/module4-vla/index.md (module overview, VLA architecture introduction) ~500 words
- [ ] T077 [P] [US4] Write docs/module4-vla/whisper-integration.md (Whisper ASR, ROS 2 integration, audio capture) ~700 words
- [ ] T078 [P] [US4] Write docs/module4-vla/llm-planning.md (GPT-4/Qwen integration, prompt engineering, structured outputs) ~900 words
- [ ] T079 [P] [US4] Write docs/module4-vla/vision-language.md (CLIP, LLaVA, object grounding, scene understanding) ~700 words
- [ ] T080 [P] [US4] Write docs/module4-vla/action-grounding.md (Plan parsing, ROS 2 action clients, safety validation) ~700 words
- [ ] T081 [P] [US4] Write docs/module4-vla/exercises.md (Voice command tests, LLM planning validation, end-to-end pipeline) ~500 words
- [ ] T082 [P] [US4] Create VLA architecture diagram in static/diagrams/vla-architecture.svg
- [ ] T083 [P] [US4] Create vision-language grounding diagram in static/diagrams/vision-language-grounding.svg
- [ ] T084 [P] [US4] Create code example: Whisper ROS 2 node (speech_recognizer.py) in companion repo module4-vla/whisper_ros2/
- [ ] T085 [P] [US4] Create code example: LLM planner node (llm_planner.py) with GPT-4 integration in companion repo module4-vla/llm_planner/
- [ ] T086 [P] [US4] Create code example: Local Qwen integration (qwen_planner.py) in companion repo module4-vla/llm_planner/
- [ ] T087 [P] [US4] Create prompt template files (robotics_system_prompt.txt, few_shot_examples.txt) in companion repo module4-vla/llm_planner/prompts/
- [ ] T088 [P] [US4] Create code example: Action grounding module (action_grounding.py) in companion repo module4-vla/action_grounding/
- [ ] T089 [P] [US4] Create code example: Safety validator (command_validator.py) in companion repo module4-vla/action_grounding/
- [ ] T090 [P] [US4] Create code example: CLIP object detection integration (clip_detector.py) in companion repo module4-vla/vision_language/
- [ ] T091 [P] [US4] Create code example: LLaVA scene understanding (llava_scene.py) in companion repo module4-vla/vision_language/
- [ ] T092 [P] [US4] Create code example: Object grounding 3D (language_to_3d.py) in companion repo module4-vla/vision_language/
- [ ] T093 [P] [US4] Create ROS 2 action server examples (navigate_action.py, grasp_action.py) in companion repo module4-vla/action_servers/
- [ ] T094 [P] [US4] Create task status publisher node (status_reporter.py) in companion repo module4-vla/feedback/
- [ ] T095 [P] [US4] Create conversational manager for multi-turn interactions in companion repo module4-vla/llm_planner/
- [ ] T096 [US4] Add 7 VLA/LLM authoritative references to docs/module4-vla/ chapters
- [ ] T097 [US4] Validate Module 4 word count (target 3000-4000 words)
- [ ] T098 [US4] Test Whisper and LLM examples (requires OpenAI API key or local Qwen)
- [ ] T099 [US4] Create Module 4 README.md with API setup instructions and cost estimates

**Checkpoint**: Module 4 complete and independently testable

---

## Phase 7: User Story 5 - Capstone: End-to-End Autonomous Humanoid (Priority: P5)

**Goal**: Complete capstone module integrating all 4 modules into unified autonomous system demonstrating voice → planning → navigation → perception → manipulation.

**Independent Test**: Reader can demonstrate humanoid that listens to voice commands, reasons with LLM, navigates with Nav2, perceives objects, manipulates objects while maintaining balance, and reports status.

### Implementation for Capstone

- [ ] T100 [P] [US5] Write docs/capstone/index.md (capstone overview, system scenario, integration objectives) ~600 words
- [ ] T101 [P] [US5] Write docs/capstone/architecture.md (system design, 7-node architecture, data flow, control flow) ~900 words
- [ ] T102 [P] [US5] Write docs/capstone/integration.md (module integration strategies, interface implementations) ~800 words
- [ ] T103 [P] [US5] Write docs/capstone/state-machine.md (orchestrator design, state transitions, error handling) ~700 words
- [ ] T104 [P] [US5] Write docs/capstone/debugging.md (troubleshooting guide, ROS 2 diagnostic tools, common issues) ~800 words
- [ ] T105 [P] [US5] Write docs/capstone/demo-guide.md (demo preparation, video recording, presentation guidelines) ~600 words
- [ ] T106 [P] [US5] Write docs/capstone/exercises.md (final validation, assessment rubric, portfolio preparation) ~500 words
- [ ] T107 [P] [US5] Create capstone system diagram in static/diagrams/capstone-system-diagram.svg
- [ ] T108 [P] [US5] Create capstone data flow diagram in static/diagrams/capstone-data-flow.svg
- [ ] T109 [P] [US5] Create capstone state machine diagram in static/diagrams/capstone-state-machine.svg
- [ ] T110 [P] [US5] Create complete ROS 2 workspace structure in companion repo capstone/complete_system/
- [ ] T111 [P] [US5] Create voice interface node (voice_interface.py) in companion repo capstone/complete_system/src/voice_interface/
- [ ] T112 [P] [US5] Create cognitive planner node (cognitive_planner.py) in companion repo capstone/complete_system/src/cognitive_planner/
- [ ] T113 [P] [US5] Create orchestrator node with state machine (orchestrator.py, SMACH or BehaviorTree) in companion repo capstone/complete_system/src/orchestrator/
- [ ] T114 [P] [US5] Create perception pipeline node (perception_pipeline.py) in companion repo capstone/complete_system/src/perception/
- [ ] T115 [P] [US5] Create navigation client node (navigation_client.py) in companion repo capstone/complete_system/src/navigation/
- [ ] T116 [P] [US5] Create manipulation server node (manipulation_server.py) in companion repo capstone/complete_system/src/manipulation/
- [ ] T117 [P] [US5] Create feedback publisher node (feedback_publisher.py) in companion repo capstone/complete_system/src/feedback/
- [ ] T118 [P] [US5] Create custom ROS 2 messages (TaskPlan.msg, TaskStep.msg, TaskStatus.msg) in companion repo capstone/complete_system/msg/
- [ ] T119 [P] [US5] Create master launch file (capstone_system.launch.py) orchestrating all nodes in companion repo capstone/launch/
- [ ] T120 [P] [US5] Create all configuration files (nav2_params.yaml, llm_config.yaml, perception_config.yaml) in companion repo capstone/configs/
- [ ] T121 [P] [US5] Create unit tests for each node in companion repo capstone/tests/unit/
- [ ] T122 [P] [US5] Create integration tests for subsystem pairs in companion repo capstone/tests/integration/
- [ ] T123 [US5] Create end-to-end system test (full voice-to-action pipeline) in companion repo capstone/tests/system/
- [ ] T124 [US5] Create demo scenario scripts (scenario1_fetch.py, scenario2_bring.py, scenario3_rearrange.py) in companion repo capstone/demos/
- [ ] T125 [US5] Record demo video (3-5 minutes showing complete pipeline) and add to static/videos/
- [ ] T126 [US5] Create performance evaluation script (measure latencies, success rates) in companion repo capstone/evaluation/
- [ ] T127 [US5] Add 5 system integration references to docs/capstone/ chapters
- [ ] T128 [US5] Validate Capstone word count (target 3500-5000 words)
- [ ] T129 [US5] Test complete capstone system end-to-end (>70% success rate target)
- [ ] T130 [US5] Create Capstone README.md with complete setup, launch, and demo instructions

**Checkpoint**: Capstone complete, all modules integrated and independently functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Finalization tasks affecting multiple modules

- [ ] T131 [P] Write docs/appendices/python-refresher.md (Python basics for readers needing review) ~500 words
- [ ] T132 [P] Write docs/appendices/linear-algebra.md (Vectors, matrices, transformations for robotics) ~600 words
- [ ] T133 [P] Write docs/appendices/hardware-guide.md (Hardware recommendations, budget options, cloud setup) ~500 words
- [ ] T134 [P] Write docs/appendices/glossary.md (Technical terms, acronyms, definitions) ~400 words
- [ ] T135 Write docs/references.md (Complete bibliography with all 35 sources, APA 7th edition)
- [ ] T136 [P] Create book structure overview diagram in static/diagrams/book-structure.svg
- [ ] T137 Run complete word count validation across all modules (verify 10,000-15,000 total)
- [ ] T138 Run citation audit (verify all 35 sources properly cited, >50% peer-reviewed)
- [ ] T139 Run plagiarism detection scan on all content (Turnitin or iThenticate)
- [ ] T140 Run Docusaurus build validation (npm run build, verify no errors)
- [ ] T141 [P] Test all code examples for execution (Ubuntu 22.04 + ROS 2 Humble clean install)
- [ ] T142 [P] Run Markdown linter on all docs/ files
- [ ] T143 [P] Validate all internal links (markdown-link-check)
- [ ] T144 Create GitHub Pages deployment workflow (GitHub Actions)
- [ ] T145 [P] Generate PDF export using Docusaurus PDF plugin
- [ ] T146 Conduct readability assessment (Flesch-Kincaid Grade Level validation, target 10-12)
- [ ] T147 Coordinate expert review (submit to domain experts in robotics, AI, control theory)
- [ ] T148 [P] Create errata page for known issues and updates
- [ ] T149 Final constitution compliance check (verify all 8 principles satisfied)
- [ ] T150 Prepare v1.0 release (tag, changelog, deployment)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately ✅
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **Module 1 (Phase 3)**: Depends on Foundational phase completion
- **Module 2 (Phase 4)**: Depends on Foundational + Module 1 (needs URDF model)
- **Module 3 (Phase 5)**: Depends on Foundational + Module 2 (needs digital twin environment)
- **Module 4 (Phase 6)**: Depends on Foundational + Module 2 (needs sensor data)
- **Capstone (Phase 7)**: Depends on ALL modules (1, 2, 3, 4) - full integration
- **Polish (Phase 8)**: Depends on all desired modules being complete

### User Story Dependencies

- **Module 1 (P1)**: No dependencies on other modules (foundational)
- **Module 2 (P2)**: Depends on Module 1 (needs URDF model)
- **Module 3 (P3)**: Depends on Module 2 (needs digital twin environment) - Can start after M2
- **Module 4 (P4)**: Depends on Module 2 (needs sensor simulation) - Can run parallel with M3
- **Capstone (P5)**: Depends on M1, M2, M3, M4 (integrates all modules)

### Within Each Module

- Content writing tasks (markdown files) can run in parallel [P]
- Code examples can be developed in parallel [P]
- Diagrams can be created in parallel [P]
- References must be added after content written
- Word count validation after all content written
- Testing after all code examples created

### Parallel Opportunities

**Setup Phase**: All tasks T001-T008 can run in parallel

**Foundational Phase**: Tasks T009-T017 can run in parallel (different artifacts)

**Module 1**: Tasks T018-T031 (content + code + diagrams) can run in parallel

**Module 2**: Tasks T036-T049 can run in parallel (depends on Module 1 URDF)

**Module 3 & 4**: Can run in parallel after Module 2 completes:
- Module 3 tasks T054-T071 (perception/navigation track)
- Module 4 tasks T076-T095 (VLA track)

**Capstone**: Most tasks T100-T120 can run in parallel (different nodes)

**Polish**: Tasks T131-T143 can run in parallel (different validation types)

---

## Parallel Example: Module 1 Content Creation

```bash
# Launch all Module 1 content writing tasks together:
Task T018: Write index.md
Task T019: Write ros2-architecture.md
Task T020: Write rclpy-development.md
Task T021: Write urdf-modeling.md
Task T022: Write exercises.md

# Launch all Module 1 code examples together:
Task T025: hello_ros2 example
Task T026: joint_state_publisher example
Task T027: sensor_simulator example
Task T028: simple_controller example

# Launch all Module 1 diagrams together:
Task T023: ROS 2 computational graph diagram
Task T024: Humanoid kinematic tree diagram
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup ✅
2. Complete Phase 2: Foundational (CRITICAL - blocks all modules)
3. Complete Phase 3: Module 1 (ROS 2 foundation)
4. **STOP and VALIDATE**: Test Module 1 independently
5. Review and publish Module 1 as early release

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add Module 1 → Test independently → Publish (MVP!)
3. Add Module 2 → Test independently → Publish
4. Add Modules 3 & 4 in parallel → Test independently → Publish
5. Add Capstone (requires all 4 modules) → Test → Final publication
6. Polish phase → v1.0 release

### Parallel Team Strategy

With multiple authors:

1. Team completes Setup + Foundational together (Tasks T001-T017)
2. Once Foundational is done:
   - Author A: Module 1 (T018-T035)
   - Author B: Module 2 (T036-T053) - starts after Module 1 URDF ready
   - Author C: Module 3 (T054-T075) - starts after Module 2 ready
   - Author D: Module 4 (T076-T099) - starts after Module 2 ready
3. All authors: Capstone (T100-T130) - after modules 1-4 complete
4. Team: Polish & review (T131-T150)

---

## Task Summary

**Total Tasks**: 150 tasks

**Breakdown by Phase**:
- Setup (Phase 1): 8 tasks
- Foundational (Phase 2): 9 tasks
- Module 1 (Phase 3): 18 tasks (content: 5, diagrams: 2, code: 11)
- Module 2 (Phase 4): 18 tasks (content: 5, diagrams: 2, code: 11)
- Module 3 (Phase 5): 22 tasks (content: 6, diagrams: 2, code: 14)
- Module 4 (Phase 6): 24 tasks (content: 6, diagrams: 2, code: 16)
- Capstone (Phase 7): 31 tasks (content: 7, diagrams: 3, code: 21)
- Polish (Phase 8): 20 tasks (appendices: 4, validation: 16)

**Parallel Opportunities**: ~80% of tasks marked [P] can run in parallel within phases

**Independent Testing**: Each module phase includes validation checkpoint

**MVP Scope**: Module 1 only (18 tasks after Setup + Foundational)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific module for traceability
- Each module should be independently completable and testable
- Commit after each module completion or logical group
- Stop at any checkpoint to validate module independently
- Introduction chapter (docs/intro.md) already completed (outside task list)
- Avoid: vague tasks, same file conflicts, cross-module dependencies that break independence
