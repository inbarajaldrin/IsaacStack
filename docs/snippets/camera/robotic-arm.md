
## **Attach to UR5e robotic arm**
Import your UR5e before hand

```python
import omni.usd
from pxr import Sdf, Gf

stage = omni.usd.get_context().get_stage()

# Get gripper and camera prims
ur5e_gripper_path = "/World/UR5e/Gripper"
camera_path = "/World/rsd455"

ur5e_prim = stage.GetPrimAtPath(ur5e_gripper_path)
camera_prim = stage.GetPrimAtPath(camera_path)

# Copy transforms from UR5e gripper to camera (like RG2)
translate_attr = ur5e_prim.GetAttribute("xformOp:translate")
orient_attr = ur5e_prim.GetAttribute("xformOp:orient")

if translate_attr.IsValid() and orient_attr.IsValid():
    camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(translate_attr.Get())
    camera_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(orient_attr.Get())

# Set specific camera values with proper xform order
camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3, custom=True).Set(Gf.Vec3d(0.82946, 0.33031, 0.17495))
camera_prim.CreateAttribute("xformOp:rotateXYZ", Sdf.ValueTypeNames.Float3, custom=True).Set(Gf.Vec3f(0.0, 0.0, 90.0))

# Set the xformOpOrder to ensure rotateXYZ is applied
camera_prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(["xformOp:translate", "xformOp:rotateXYZ"])

# Create FixedJoint inside /World/rsd455/RSD455
joint_prim = stage.DefinePrim("/World/rsd455/RSD455/FixedJoint", "PhysicsFixedJoint")
joint_prim.CreateRelationship("physics:body0").SetTargets([Sdf.Path("/World/UR5e/wrist_3_link/flange")])
joint_prim.CreateRelationship("physics:body1").SetTargets([Sdf.Path("/World/rsd455/RSD455")])
joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)

print("✓ Camera attached with proper xformOpOrder")
```
