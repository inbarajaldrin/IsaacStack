## **Create Camera top down**
```python
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.sensor import Camera
import numpy as np

def create_topdown_camera(x, y, z, name="topdown_camera"):
    camera = Camera(
        prim_path=f"/World/{name}",
        position=np.array([x, y, z]),
        frequency=30,
        resolution=(640, 480),
        orientation=rot_utils.euler_angles_to_quats(
            np.array([0, 90, 0]), degrees=True
        ),
    )
    camera.initialize()
    camera.add_motion_vectors_to_frame()
    print(f"Top-down camera created at ({x}, {y}, {z})")
    return camera

# Use it:
camera = create_topdown_camera(0, 0, 5)  # Above origin at height 5
```

## **Create Camera Isometric view**
```python
from omni.isaac.sensor import Camera
import omni.usd
from pxr import Gf
import numpy as np

# Create basic camera first
camera = Camera(
    prim_path="/World/isometric_camera",
    position=np.array([0, 0, 0]),  # We'll set position via USD
    frequency=30,
    resolution=(640, 480),
)
camera.initialize()

# Set exact transform values via USD attributes
stage = omni.usd.get_context().get_stage()
camera_prim = stage.GetPrimAtPath("/World/isometric_camera")

# Set exact position
camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(2.3, 3.2, 1.3))

# Set exact quaternion (w, x, y, z order for USD)
camera_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(0.25624, 0.19434, 0.57219, 0.75443))

```

