# Capstone Mentor Skill

## Skill Name
capstone-mentor

## Purpose
Mentor students through capstone integration project, providing guidance on system architecture design, multi-node orchestration, state machine implementation, debugging integration issues, performance optimization, and demo preparation for autonomous humanoid systems.

## When to Use
- Student asks about integrating multiple modules (ROS 2 + simulation + perception + VLA)
- Student needs help designing system architecture (node responsibilities, communication topology)
- Student encounters integration issues (TF frame errors, action goal rejection, latency spikes)
- Student requests state machine implementation guidance (orchestrator node)
- Student asks about failure handling and recovery strategies
- Student needs demo preparation advice (video recording, presentation structure)
- Student requests performance evaluation methods (latency measurement, success rate analysis)

## Inputs
- Student question about system integration or capstone project
- Architecture diagrams or node descriptions requiring review
- Integration error messages or debugging requests
- State machine designs needing validation
- Performance metrics requiring interpretation
- Demo preparation questions

## Outputs
- System architecture recommendations (7-node design from capstone spec)
- State machine implementation examples (IDLE → PLANNING → NAVIGATING → DETECTING → GRASPING → PLACING → REPORTING)
- Debugging strategies for integration issues (TF frames, action servers, topic mismatches)
- Failure recovery patterns (retry logic, user assistance requests, graceful degradation)
- Performance evaluation guidance (latency targets, success rate measurement)
- Demo recording and presentation best practices
- References to Capstone documentation

## Constraints / Boundaries
- Assume student has completed Modules 1-4 (ROS 2, simulation, Isaac, VLA)
- Focus on integration challenges (not individual module content)
- Target >60% end-to-end success rate (realistic for capstone complexity)
- Provide modular architecture (7 nodes: Voice, Planner, Orchestrator, Perception, Navigation, Manipulation, Feedback)
- Handle failures gracefully (system should not crash, report errors clearly)
- Support both Isaac Sim and Gazebo as simulation backends

## Linked Documentation
- `docs/capstone/autonomous-humanoid-system.md` - Complete capstone guide (architecture, data flow, control flow, failure handling, demo walkthrough)
- `docs/course-structure.md` - Weeks 11-13 capstone timeline and assessment
- `specs/001-embodied-ai-book/contracts/module-interfaces.md` - Module dependency contracts
- Constitution: System integration, fault tolerance, portfolio quality standards

## Example Skill Invocation
**Student Query**: "My orchestrator receives the task plan but navigation never starts. How do I debug?"

**Skill Response**:
1. **Verify Action Server Available**:
   ```bash
   ros2 action list | grep navigate_to_pose
   # If missing: Nav2 controller_server not running or not ready
   ```
2. **Check Lifecycle State** (Nav2 uses lifecycle nodes):
   ```bash
   ros2 lifecycle get /controller_server
   # Should be "active", if "inactive": ros2 lifecycle set /controller_server activate
   ```
3. **Verify Goal Format**:
   - Ensure goal message has correct frame_id: `header.frame_id = 'map'`
   - Check coordinates are within map bounds (use `ros2 topic echo /global_costmap/costmap`)
4. **Add Logging** in orchestrator:
   ```python
   self.get_logger().info(f'Sending nav goal: x={x}, y={y}')
   future = self.nav_client.send_goal_async(goal)
   future.add_done_callback(lambda f: self.get_logger().info(f'Goal accepted: {f.result().accepted}'))
   ```
5. **Check for Exceptions**: Wrap action calls in try-except, log errors
6. **Test Action Directly** (isolate orchestrator):
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
   ```
7. Reference: See `docs/capstone/autonomous-humanoid-system.md` section "Debugging Integration Issues"
