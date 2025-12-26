# Digital Twin Visualization Skill

## Skill Name
digital-twin-visualization

## Purpose
Provide expert instruction on creating digital twin simulations for humanoid robots using Gazebo (physics-accurate simulation), Unity (photorealistic visualization), and hybrid Gazebo+Unity workflows for combined physics and visual fidelity.

## When to Use
- Student asks about Gazebo simulation setup, world files, or SDF format
- Student needs help with Unity URDF import and ROS-TCP-Connector configuration
- Student encounters physics simulation issues (robot falling, instability, vibrations)
- Student requests guidance on sensor simulation (LiDAR, RGB-D cameras, IMU) in Gazebo
- Student asks about hybrid simulation (Gazebo physics + Unity visuals)
- Student needs help debugging collision detection or contact parameters

## Inputs
- Student question about simulation platforms or configuration
- URDF/SDF files requiring physics tuning
- Gazebo world files needing environment design
- Unity scenes requiring ROS 2 integration
- Physics simulation problems (instability, unrealistic behavior)

## Outputs
- Gazebo world file examples with realistic physics
- Unity scene setup instructions with ROS-TCP-Connector
- Physics parameter tuning guidance (timestep, contact properties, friction)
- Sensor plugin configurations (camera, LiDAR, IMU) with noise models
- Debugging solutions for common simulation issues
- References to Module 2 documentation

## Constraints / Boundaries
- Focus on Gazebo Fortress/Classic (compatible with ROS 2 Humble)
- Use Unity 2021.3 LTS (long-term support version)
- Provide SDF 1.6+ format examples
- Follow physics engine best practices (ODE, Bullet, PhysX)
- Ensure sensor configurations publish standard ROS 2 message types
- Maintain simulation real-time factor ≥0.8 (performance considerations)

## Linked Documentation
- `docs/module2-digital-twin/gazebo-simulation.md` - Gazebo setup, physics engines, sensor simulation, world creation
- `docs/module2-digital-twin/unity-integration.md` - Unity setup, URDF import, ROS-TCP-Connector, hybrid simulation
- Constitution: Reproducibility, executable code, simulation fidelity standards

## Example Skill Invocation
**Student Query**: "My humanoid robot falls through the ground plane in Gazebo. What's wrong?"

**Skill Response**:
1. Verify ground plane has collision geometry: `<collision>` with `<plane>` geometry
2. Check robot spawn height: Ensure `<pose>` Z-coordinate ≥1.0m (above ground)
3. Validate URDF inertial properties: Non-zero mass, correct inertia tensors
4. Increase physics solver iterations: `<iterations>50</iterations>` (or higher)
5. Check timestep: `<max_step_size>0.001</max_step_size>` (1ms recommended)
6. Verify contact parameters: `<soft_cfm>0.01</soft_cfm>`, `<soft_erp>0.2</soft_erp>`
7. Reference: See `docs/module2-digital-twin/gazebo-simulation.md` section "Common Simulation Issues and Solutions"
