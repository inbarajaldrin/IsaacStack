```python
import omni.kit.commands
from pxr import Gf

# Generic variables - change these values
prim_path = "/World/your_object_name"  # Full path to your object
position = [0.0, 0.0, 0.0]            # [x, y, z] in meters
rotation = [0.0, 0.0, 0.0]            # [x, y, z] in degrees  
scale = [1.0, 1.0, 1.0]               # [x, y, z] scale factors

# Set translate
omni.kit.commands.execute('ChangeProperty',
                         prop_path=f"{prim_path}.xformOp:translate",
                         value=Gf.Vec3d(position[0], position[1], position[2]),
                         prev=None)

# Set rotation (Euler angles in degrees)
omni.kit.commands.execute('ChangeProperty',
                         prop_path=f"{prim_path}.xformOp:rotateXYZ",
                         value=Gf.Vec3d(rotation[0], rotation[1], rotation[2]),
                         prev=None)

# Set scale
omni.kit.commands.execute('ChangeProperty',
                         prop_path=f"{prim_path}.xformOp:scale",
                         value=Gf.Vec3d(scale[0], scale[1], scale[2]),
                         prev=None)

print(f"Transform set for: {prim_path}")
print(f"Position: {position}")
print(f"Rotation: {rotation}")
print(f"Scale: {scale}")
```