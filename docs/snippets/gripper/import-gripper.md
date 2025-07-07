## **Import Robotiq 2f-85 Gripper**
Tutorial on how to rig Robotiq2f-85 gripper ../tutorials/post5.md

[Download Robotiq 2F-85](../../../assets/assets/Robotiq_2F_85.usd)

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

rg2_usd_path = "omniverse://localhost/Library/Robotiq_2F_85.usd"
add_reference_to_stage(rg2_usd_path, "/Robotiq_2F_85_Gripper")
print("RG2 Gripper imported at /Robotiq_2F_85_Gripper")
```

## **Import Robotiq 2f-85 Gripper from root layer in a usd file**
```python
# Import and Control Robotiq Gripper
import omni.usd
from pxr import UsdPhysics, Sdf

stage = omni.usd.get_context().get_stage()

# Import gripper (only if not already imported)
sublayer_path = "omniverse://localhost/Library/Robotiq_2F_85.usd"
root_layer = stage.GetRootLayer()

if sublayer_path not in root_layer.subLayerPaths:
    root_layer.subLayerPaths.append(sublayer_path)
    print("Robotiq gripper imported")
else:
    print("Robotiq gripper already imported")
```

## **Import Robotiq 2f-85 and Open Close gripper**
```python
# Import and Control Robotiq Gripper
import omni.usd
from pxr import UsdPhysics, Sdf

stage = omni.usd.get_context().get_stage()

def control_robotiq_gripper(action="open"):
    """Control Robotiq gripper: 'open' or 'close'"""
    knuckle_joint_path = "/Robotiq_2F_85_Gripper/robotiq_arg2f_base_link/right_outer_knuckle_joint"
    finger_joint_path = "/Robotiq_2F_85_Gripper/robotiq_arg2f_base_link/finger_joint"
    
    knuckle_joint = stage.GetPrimAtPath(knuckle_joint_path)
    finger_joint = stage.GetPrimAtPath(finger_joint_path)
    
    if not knuckle_joint or not finger_joint:
        print("Gripper joints not found.")
        return
    
    velocity = -50.0 if action == "open" else 50.0
    
    for joint in [knuckle_joint, finger_joint]:
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTargetVelocityAttr(velocity)
    
    print(f"Robotiq gripper {action}ed with velocity {velocity}")

# Examples:
control_robotiq_gripper('close')  # Close gripper
# control_robotiq_gripper("open")   # Open gripper
```

## **Import RG2 Gripper**
Tutorial on how to rig RG2 gripper ../tutorials/post3.md

[Download RG2 Gripper](../../../assets/assets/RG2.usd)

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

rg2_usd_path = "omniverse://localhost/Library/RG2.usd"
add_reference_to_stage(rg2_usd_path, "/RG2_Gripper")
print("RG2 Gripper imported at /RG2_Gripper")
```

## **Import RG2 and Open Close gripper**
```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.world import World
from omni.isaac.core.utils.types import ArticulationActions
import numpy as np
import omni.usd

# Check if gripper already exists in scene
stage = omni.usd.get_context().get_stage()
rg2_exists = stage.GetPrimAtPath("/RG2_Gripper").IsValid()

if not rg2_exists:
    # Import gripper only if it doesn't exist
    rg2_usd_path = "omniverse://localhost/Library/RG2.usd"
    add_reference_to_stage(rg2_usd_path, "/RG2_Gripper")
    print("RG2 Gripper imported")
else:
    print("RG2 Gripper already exists in scene")

# Setup gripper control (finds existing gripper)
gripper_view = None
try:
    gripper_view = World.instance().scene.get_object("RG2_Gripper_View")
    if gripper_view is not None:
        print("Using existing gripper view")
    else:
        raise Exception("Gripper view not found")
except:
    gripper_view = ArticulationView(prim_paths_expr="/RG2_Gripper", name="RG2_Gripper_View")
    World.instance().scene.add(gripper_view)
    print("Created new gripper view")

# Initialize gripper view
if gripper_view is not None:
    gripper_view.initialize()
    print("Gripper view initialized")
else:
    print("Error: Failed to create gripper view")
    exit()

def control_gripper(width_mm):
    """Control gripper: 0mm=closed, 110mm=open"""
    joint_angle = -np.pi/4 + (width_mm/110.0) * (np.pi/6 + np.pi/4)
    target_positions = np.array([joint_angle, joint_angle])
    action = ArticulationActions(joint_positions=target_positions, joint_indices=np.array([0, 1]))
    gripper_view.apply_action(action)
    print(f"Gripper set to {width_mm}mm")

# Examples (uncomment to use):
control_gripper(0)    # Close gripper
# control_gripper(110)  # Open gripper
# control_gripper(55)   # Half open
```