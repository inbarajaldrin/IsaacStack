articulation.apply_action(action)
articulation.set_joint_positions(
    action.joint_positions,
    joint_indices=action.joint_indices
)

## **Import UR5e and perform forward kinematics (Articulation set joints)** 

```python
import omni.usd
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation

# CHANGE THESE VALUES AND RERUN
joint_angles_deg = [30, -30, 45, -60, 30, 0]  # degrees
joint_angles = [x * 3.14159/180 for x in joint_angles_deg]  # convert to radians
#joint_angles = [0.5, -0.5, 0.8, -1.0, 0.5, 0.0]  #for radians

# Check if robot already exists
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")

if not prim.IsValid():
    # First time - load everything
    world = World()
    world.scene.add_default_ground_plane()
    
    asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    world.reset()
    print("Robot loaded for first time")

# Apply joint positions
articulation = Articulation("/World/UR5e")
articulation.initialize()
articulation.set_joint_positions(joint_angles)

print(f"Applied joint angles: {joint_angles}")
position, orientation = articulation.get_world_pose()
print(f"End-effector position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
```

## **Import UR5e and perform forward kinematics** (Articulation apply action)
```python
import numpy as np
import omni.usd
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path

# Set joint angles in degrees (CHANGE HERE)
joint_angles_deg = [0, -45, 0, 0, 0, 0]  # degrees
joint_angles = [np.deg2rad(x) for x in joint_angles_deg]  # convert to radians

# Load USD stage
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")

if not prim.IsValid():
    # First-time setup
    world = World()
    world.scene.add_default_ground_plane()

    # Load UR5e asset
    try:
        asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    except:
        fallback_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        add_reference_to_stage(usd_path=fallback_path, prim_path="/World/UR5e")

    world.reset()
    print("UR5e robot loaded.")

# Control using Articulation only
articulation = Articulation("/World/UR5e")
articulation.initialize()
action = ArticulationAction(joint_positions=np.array(joint_angles, dtype=np.float32))
articulation.apply_action(action)

# Report results
print(f"Applied joint angles (degrees): {joint_angles_deg}")
print(f"Applied joint angles (radians): {joint_angles}")

# Get EE pose
position, orientation = articulation.get_world_pose()
print(f"End-effector position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
```
## **Import UR5e and perform forward kinematics (ArticulationView apply action)** 
```python
import numpy as np
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationActions
from omni.isaac.nucleus import get_assets_root_path
import omni.usd

# Change these joint angles (degrees) and run the script again
joint_angles_deg = [0, 0, 0, 0, 0, 0]

# Setup (only runs once)
stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath("/World/UR5e").IsValid():
    world = World()
    world.scene.add_default_ground_plane()
    asset_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    world.reset()

# Move robot
ur5e_view = World.instance().scene.get_object("ur5e_view")
if ur5e_view is None:
    ur5e_view = ArticulationView(prim_paths_expr="/World/UR5e", name="ur5e_view")
    World.instance().scene.add(ur5e_view)
    ur5e_view.initialize()

# Apply joint positions using ArticulationActions
joint_positions_array = np.array(np.radians(joint_angles_deg), dtype=np.float32)
action = ArticulationActions(joint_positions=joint_positions_array)
ur5e_view.apply_action(action)
print(f"Moved to: {joint_angles_deg}")
```