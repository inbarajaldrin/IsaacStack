## Create a Cube in the Stage
```python
from pxr import Usd, UsdGeom, Gf
import omni.usd

# Get the stage
stage = omni.usd.get_context().get_stage()

# Define prim path for the cube
cube_path = "/World/TestCube"

# Create the cube if it doesn't exist
if not stage.GetPrimAtPath(cube_path).IsValid():
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(0.2)  # Set cube size (optional)
    print(f"Created cube at {cube_path}")
else:
    print(f"Cube already exists at {cube_path}")
```