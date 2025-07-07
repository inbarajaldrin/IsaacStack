<!-- # UR5e Robot Control in Isaac Sim -->

## **Inverse Kinematics with Articulation (set_joint_positions)**

```python
import omni.usd
import numpy as np
import os
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats

# CHANGE THESE VALUES AND RERUN
target_pos = [0.1, 0.1, 0.9]      # X, Y, Z position
target_rot_deg = [0, 0, 0]       # Roll, Pitch, Yaw in degrees
target_rot = [x * 3.14159/180 for x in target_rot_deg]  # convert to radians

# Check if robot already exists
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")

if not prim.IsValid():
    # First time - load everything
    from omni.isaac.core.utils.physics import simulate_async
    world = World()
    world.scene.add_default_ground_plane()
    
    asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    
    # Enable physics scene first
    from omni.physx import get_physx_interface
    get_physx_interface().start_simulation()
    
    world.reset()
    print("Robot loaded for first time")

# Load IK solver
home_dir = os.getenv("HOME")
pkg_dir = os.path.join(home_dir, ".local", "share", "ov", "pkg")
isaac_sim_versions = [d for d in os.listdir(pkg_dir) if d.startswith("isaac-sim-")]
isaac_sim_version = isaac_sim_versions[0]
base_path = os.path.join(pkg_dir, isaac_sim_version, "exts", "omni.isaac.motion_generation")
robot_description_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "rmpflow", "ur5e_robot_description.yaml")
urdf_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "ur5e.urdf")

ik_solver = LulaKinematicsSolver(robot_description_path=robot_description_path, urdf_path=urdf_path)

# Solve IK
articulation = Articulation("/World/UR5e")
articulation.initialize()
articulation_ik_solver = ArticulationKinematicsSolver(articulation, ik_solver, "tool0")

target_position = np.array(target_pos, dtype=np.float32)
target_orientation = np.array(euler_angles_to_quats(target_rot), dtype=np.float32)

action, success = articulation_ik_solver.compute_inverse_kinematics(target_position, target_orientation)

if success:
    articulation.set_joint_positions(
        action.joint_positions,
        joint_indices=action.joint_indices
    )
    print(f"IK Success! Target: {target_pos}, {target_rot}")
    print(f"Joint angles: {action.joint_positions}")
else:
    print(f"IK Failed for target: {target_pos}, {target_rot}")
```

## **Inverse Kinematics with Articulation (apply_action)**

```python
import omni.usd
import numpy as np
import os
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats

# CHANGE THESE VALUES AND RERUN
target_pos = [0.5, 0.5, 0.5]      # X, Y, Z position
target_rot_deg = [0, 0, 0]       # Roll, Pitch, Yaw in degrees
target_rot = [x * 3.14159/180 for x in target_rot_deg]  # convert to radians

# Check if robot already exists
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")

if not prim.IsValid():
    # First time - load everything
    from omni.isaac.core.utils.physics import simulate_async
    world = World()
    world.scene.add_default_ground_plane()

    asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")

    # Enable physics scene first
    from omni.physx import get_physx_interface
    get_physx_interface().start_simulation()

    world.reset()
    print("Robot loaded for first time")

# Load IK solver
home_dir = os.getenv("HOME")
pkg_dir = os.path.join(home_dir, ".local", "share", "ov", "pkg")
isaac_sim_versions = [d for d in os.listdir(pkg_dir) if d.startswith("isaac-sim-")]
isaac_sim_version = isaac_sim_versions[0]
base_path = os.path.join(pkg_dir, isaac_sim_version, "exts", "omni.isaac.motion_generation")
robot_description_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "rmpflow", "ur5e_robot_description.yaml")
urdf_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "ur5e.urdf")

ik_solver = LulaKinematicsSolver(robot_description_path=robot_description_path, urdf_path=urdf_path)

# Solve IK
articulation = Articulation("/World/UR5e")
articulation.initialize()
articulation_ik_solver = ArticulationKinematicsSolver(articulation, ik_solver, "tool0")

target_position = np.array(target_pos, dtype=np.float32)
target_orientation = np.array(euler_angles_to_quats(target_rot), dtype=np.float32)

action, success = articulation_ik_solver.compute_inverse_kinematics(target_position, target_orientation)

if success:
    articulation.apply_action(action)
    print(f"IK Success! Target: {target_pos}, {target_rot}")
    print(f"Joint angles: {action.joint_positions}")
else:
    print(f"IK Failed for target: {target_pos}, {target_rot}")
```

## **Inverse Kinematics with ArticulationView (apply_action)**

```python
import omni.usd
import numpy as np
import os
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView, Articulation
from omni.isaac.core.utils.types import ArticulationActions
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats

# CHANGE THESE VALUES AND RERUN
target_pos = [0.3, 0.3, 0.8]      # X, Y, Z position
target_rot_deg = [0, 0, 0]       # Roll, Pitch, Yaw in degrees
target_rot = [x * 3.14159/180 for x in target_rot_deg]  # convert to radians

# Check if robot already exists
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")

if not prim.IsValid():
    # First time - load everything
    from omni.isaac.core.utils.physics import simulate_async
    world = World()
    world.scene.add_default_ground_plane()
    
    asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    
    # Enable physics scene first
    from omni.physx import get_physx_interface
    get_physx_interface().start_simulation()
    
    world.reset()
    print("Robot loaded for first time")

# Load IK solver
home_dir = os.getenv("HOME")
pkg_dir = os.path.join(home_dir, ".local", "share", "ov", "pkg")
isaac_sim_versions = [d for d in os.listdir(pkg_dir) if d.startswith("isaac-sim-")]
isaac_sim_version = isaac_sim_versions[0]
base_path = os.path.join(pkg_dir, isaac_sim_version, "exts", "omni.isaac.motion_generation")
robot_description_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "rmpflow", "ur5e_robot_description.yaml")
urdf_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "ur5e.urdf")

ik_solver = LulaKinematicsSolver(robot_description_path=robot_description_path, urdf_path=urdf_path)

# Create ArticulationView
ur5e_view = World.instance().scene.get_object("ur5e_view")
if ur5e_view is None:
    ur5e_view = ArticulationView(prim_paths_expr="/World/UR5e", name="ur5e_view")
    World.instance().scene.add(ur5e_view)
    ur5e_view.initialize()

# Create separate Articulation for IK solving (ArticulationView can't be used directly with IK solver)
articulation = Articulation("/World/UR5e")
articulation.initialize()
articulation_ik_solver = ArticulationKinematicsSolver(articulation, ik_solver, "tool0")

# Solve IK
target_position = np.array(target_pos, dtype=np.float32)
target_orientation = np.array(euler_angles_to_quats(target_rot), dtype=np.float32)

action, success = articulation_ik_solver.compute_inverse_kinematics(target_position, target_orientation)

if success:
    # Apply using ArticulationView
    ur5e_view.apply_action(action)
    print(f"IK Success! Target: {target_pos}, {target_rot}")
    print(f"Joint angles: {action.joint_positions}")
else:
    print(f"IK Failed for target: {target_pos}, {target_rot}")
```
