## **Import a USD prim into the stage**

```python
import os
from omni.isaac.core.utils.stage import add_reference_to_stage

# Import a USD file as a prim
usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"  # Change this to your USD path
filename = os.path.splitext(os.path.basename(usd_path))[0]
prim_path = f"/World/{filename}" # Auto-name the prim using filename
# prim_path = "/World/ur5e"  # Change this to your desired prim path

add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
print(f"Prim imported at {prim_path}")
```

## **Import a USD prim at x,y,z,r,p,y, quaternions**

```python
from pxr import Usd, UsdGeom, Gf
import numpy as np
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
import omni.usd

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/ur5e")
if not prim.IsValid():
    raise RuntimeError("Prim at /World/ur5e not found.")

xform = UsdGeom.Xform(prim)
xform.ClearXformOpOrder()

# Translation
position = Gf.Vec3d(1.0, 0.0, 0.0)
xform.AddTranslateOp().Set(position)

# RPY in degrees → radians → quaternion
rpy_deg = np.array([0.0, 0.0, 0.0])
rpy_rad = np.deg2rad(rpy_deg)
quat_xyzw = euler_angles_to_quats(rpy_rad)
quat = Gf.Quatd(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])

# Add orient op with matching precision
orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
orient_op.Set(quat)

print("Applied translation and rotation with double-precision quaternion.")
```
