##**Connect Robotiq 2F-85 to UR5e**
Import UR5e version from IsaacSim Assets 4.2 version as sublayer(insert link to usd) omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
```python
# Attach Robotiq Gripper to UR5e
import omni.usd
from pxr import Sdf, Gf

stage = omni.usd.get_context().get_stage()

tool0_prim_path = "/ur5e/tool0"
gripper_prim_path = "/Robotiq_2F_85"
flange_tool_path = "/ur5e/flange/flange_tool0"

tool0_prim = stage.GetPrimAtPath(tool0_prim_path)
gripper_prim = stage.GetPrimAtPath(gripper_prim_path)
flange_tool_prim = stage.GetPrimAtPath(flange_tool_path)

if not tool0_prim or not gripper_prim:
    print("Could not find tool0 or Robotiq gripper")
else:
    # Attach transform attributes
    translate_attr = tool0_prim.GetAttribute("xformOp:translate")
    orient_attr = tool0_prim.GetAttribute("xformOp:orient")
    
    if translate_attr.IsValid() and orient_attr.IsValid():
        gripper_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(translate_attr.Get())
        gripper_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(orient_attr.Get())
        print("Gripper attached to tool0 transform.")
    else:
        print("Could not get valid attributes for transform.")
    
    # Set gripper orientation: x=-90°, y=90°, z=0°
    rotate_attr = gripper_prim.CreateAttribute("xformOp:rotateXYZ", Sdf.ValueTypeNames.Float3, custom=True)
    rotate_attr.Set(Gf.Vec3f(-90.0, 90.0, 0.0))  # XYZ rotation in degrees
    print("Set gripper orientation: x=-90°, y=90°, z=0°")
    
    # Set physics:body1 relationship
    if flange_tool_prim:
        body1_rel = flange_tool_prim.GetRelationship("physics:body1")
        if body1_rel:
            body1_rel.SetTargets([Sdf.Path("/Robotiq_2F_85/robotiq_arg2f_base_link")])
            print("Body1 target set to /Robotiq_2F_85/robotiq_arg2f_base_link.")
        else:
            print("Failed to find or set the body1 relationship.")
    else:
        print("Flange tool prim not found.")

print("Robotiq gripper attachment complete")
```
##**Connect RG2 to UR5e**
Import UR5e version from IsaacSim Assets 4.5 version (insert link to usd)
omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
```python
import omni.usd
from pxr import Usd, Sdf, UsdGeom, Gf
import math

stage = omni.usd.get_context().get_stage()
ur5e_gripper_path = "/World/UR5e/Gripper"
rg2_path = "/RG2_Gripper"
joint_path = "/World/UR5e/joints/robot_gripper_joint"
rg2_base_link = "/RG2_Gripper/onrobot_rg2_base_link"

ur5e_prim = stage.GetPrimAtPath(ur5e_gripper_path)
rg2_prim = stage.GetPrimAtPath(rg2_path)
joint_prim = stage.GetPrimAtPath(joint_path)

if not ur5e_prim or not rg2_prim:
    print("Error: UR5e or RG2 gripper prim not found.")
else:
    # Copy transforms from UR5e gripper to RG2
    translate_attr = ur5e_prim.GetAttribute("xformOp:translate")
    orient_attr = ur5e_prim.GetAttribute("xformOp:orient")

    if translate_attr.IsValid() and orient_attr.IsValid():
        rg2_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(translate_attr.Get())
        rg2_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(orient_attr.Get())

    # --------- 1. Set orientation for RG2 Gripper ---------
    print("Setting RG2 gripper orientation...")
    if rg2_prim.IsValid():
        quat_attr = rg2_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd, custom=True)
        quat_attr.Set(Gf.Quatd(0.70711, Gf.Vec3d(-0.70711, 0.0, 0.0)))
        print(" Set xformOp:orient for RG2 gripper.")
    else:
        print(f" Gripper not found at {rg2_path}")

    # Create or update the physics joint
    if not joint_prim:
        joint_prim = stage.DefinePrim(joint_path, "PhysicsFixedJoint")

    joint_prim.CreateRelationship("physics:body1").SetTargets([Sdf.Path(rg2_base_link)])
    joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    joint_prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)

    # --------- 2. Set localRot0 and localRot1 for joint ---------
    print("Setting joint rotation parameters...")
    if joint_prim.IsValid():
        def euler_to_quatf(x_deg, y_deg, z_deg):
            """Convert Euler angles (XYZ order, degrees) to Gf.Quatf"""
            rx = Gf.Quatf(math.cos(math.radians(x_deg) / 2), Gf.Vec3f(1, 0, 0) * math.sin(math.radians(x_deg) / 2))
            ry = Gf.Quatf(math.cos(math.radians(y_deg) / 2), Gf.Vec3f(0, 1, 0) * math.sin(math.radians(y_deg) / 2))
            rz = Gf.Quatf(math.cos(math.radians(z_deg) / 2), Gf.Vec3f(0, 0, 1) * math.sin(math.radians(z_deg) / 2))
            return rx * ry * rz  # Apply in XYZ order

        # Set the rotation quaternions for proper joint alignment
        quat0 = euler_to_quatf(-90, 0, -90)
        quat1 = euler_to_quatf(-180, 90, 0)
        
        joint_prim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat0)
        joint_prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat1)
        print(" Set physics:localRot0 and localRot1 for robot_gripper_joint.")
    else:
        print(f" Joint not found at {joint_path}")

    print("RG2 successfully attached to UR5e with proper orientation and joint configuration.")
```