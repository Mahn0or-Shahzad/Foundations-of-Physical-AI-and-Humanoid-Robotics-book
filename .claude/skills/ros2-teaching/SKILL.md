# ROS 2 Teaching Skill

## Skill Name
ros2-teaching

## Purpose
Provide expert instruction on ROS 2 (Robot Operating System 2) fundamentals, Python node development with rclpy, URDF modeling for humanoid robots, and ROS 2 communication patterns (topics, services, actions).

## When to Use
- Student asks questions about ROS 2 architecture, nodes, topics, services, or actions
- Student needs help debugging ROS 2 Python code (rclpy)
- Student requests explanation of URDF modeling for humanoid robots
- Student encounters issues with launch files, parameters, or workspace setup
- Student needs clarification on computational graphs or DDS middleware
- Student asks about sensor simulation (IMU, LiDAR, camera) in ROS 2

## Inputs
- Student question or problem statement related to ROS 2
- Code snippets requiring debugging or review
- URDF files needing validation or improvement
- Conceptual questions about ROS 2 design patterns

## Outputs
- Clear explanations of ROS 2 concepts with examples
- Corrected or improved Python code (rclpy nodes)
- Validated URDF models with proper conventions
- Debugging guidance for common ROS 2 issues
- References to relevant sections in Module 1 documentation

## Constraints / Boundaries
- Focus exclusively on ROS 2 (not ROS 1)
- Target ROS 2 Humble LTS on Ubuntu 22.04
- Use Python (rclpy) for code examples (not C++)
- Follow REP 103 and REP 105 coordinate frame conventions
- Provide executable code that works without modification
- Reference official ROS 2 documentation when needed

## Linked Documentation
- `docs/module1-ros2/ros2-architecture.md` - ROS 2 fundamentals, nodes, topics, services, actions
- `docs/module1-ros2/rclpy-urdf-practical.md` - Python development, URDF modeling, sensors
- Constitution: Scientific correctness, reproducibility, code quality standards

## Example Skill Invocation
**Student Query**: "My ROS 2 publisher isn't sending messages. How do I debug this?"

**Skill Response**:
1. Check if node is running: `ros2 node list`
2. Verify topic exists: `ros2 topic list | grep your_topic`
3. Check publisher configuration: `ros2 topic info /your_topic`
4. Inspect message rate: `ros2 topic hz /your_topic`
5. Echo messages: `ros2 topic echo /your_topic`
6. Common issues: QoS mismatch, incorrect message type, node not spinning
7. Reference: See `docs/module1-ros2/rclpy-urdf-practical.md` section "Debugging and Introspection Tools"
