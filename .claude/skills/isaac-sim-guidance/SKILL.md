# Isaac Sim Guidance Skill

## Skill Name
isaac-sim-guidance

## Purpose
Provide expert guidance on NVIDIA Isaac Sim setup, photorealistic simulation, synthetic data generation, Isaac ROS GPU-accelerated perception packages (nvblox, cuVSLAM, DOPE), Nav2 integration for humanoid navigation, and Jetson deployment.

## When to Use
- Student asks about Isaac Sim installation or system requirements
- Student needs help with synthetic dataset generation or domain randomization
- Student requests guidance on Isaac ROS packages (nvblox, cuVSLAM, DOPE)
- Student encounters performance issues or GPU errors in Isaac Sim
- Student asks about Nav2 configuration for bipedal humanoid navigation
- Student needs help with Sim2Real transfer concepts or Jetson deployment

## Inputs
- Student question about Isaac Sim, Isaac ROS, or Nav2
- Isaac Sim scene files or Python scripts requiring debugging
- Nav2 parameter files needing humanoid-specific tuning
- Performance issues (low FPS, GPU memory errors)
- Jetson deployment questions (cross-compilation, ARM64)

## Outputs
- Isaac Sim installation instructions (Omniverse Launcher, system requirements)
- Synthetic data generation scripts using omni.replicator
- Isaac ROS launch file examples (nvblox, cuVSLAM configuration)
- Nav2 parameter files tuned for bipedal constraints (footprint, velocity limits)
- Performance optimization guidance (voxel size, resolution, GPU utilization)
- Jetson deployment workflow (cross-compilation, ARM64 builds)
- References to Module 3 documentation

## Constraints / Boundaries
- Require NVIDIA RTX GPU (RTX 3060+ minimum, cannot run on AMD/Intel GPUs)
- Target Isaac Sim 2023.1.0+ (latest stable version)
- Focus on ROS 2 Humble compatibility (Isaac ROS humble-devel branch)
- Provide cloud GPU alternatives (AWS g4dn, Azure NC-series) for students without RTX
- Nav2 tuning specific to bipedal humanoids (not wheeled robots)
- Acknowledge Sim2Real reality gap (10-30% performance degradation typical)

## Linked Documentation
- `docs/module3-isaac/isaac-sim.md` - Complete Isaac ecosystem guide (Isaac Sim, Isaac ROS, Nav2, RL concepts, Jetson deployment)
- `specs/001-embodied-ai-book/research.md` - Isaac platform research, performance benchmarks
- Constitution: GPU acceleration, reproducibility, hardware requirements

## Example Skill Invocation
**Student Query**: "Isaac ROS nvblox is running slow (5 FPS). How do I improve performance?"

**Skill Response**:
1. Check GPU utilization: `nvidia-smi` (should be >80% during mapping)
2. Reduce voxel size: `voxel_size: 0.1` → `0.05` (coarser map, faster)
3. Limit integration distance: `max_integration_distance: 5.0` (only map nearby)
4. Lower mesh update rate: `mesh_update_rate: 5.0` → `2.0` (publish less frequently)
5. Reduce ESDF slice height: `esdf_2d_max_height: 2.0` (humanoid height, not full 3D)
6. Check camera resolution: 640×480 faster than 1920×1080
7. Verify RTX GPU model: RTX 3060 achieves ~20 FPS, RTX 4070 achieves ~30 FPS
8. Reference: See `docs/module3-isaac/isaac-sim.md` section "nvblox: 3D Reconstruction" parameters
