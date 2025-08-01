
## **Attach to UR5e robotic arm**
Import your UR5e before hand

```python
import omni.kit.commands
from pxr import Gf

# Move the prim
omni.kit.commands.execute('MovePrim',
                         path_from="/World/rsd455",
                         path_to="/World/UR5e/wrist_3_link/rsd455")

# Set transform properties
omni.kit.commands.execute('ChangeProperty',
                         prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:translate",
                         value=Gf.Vec3d(-0.012, -0.06, -0.01),
                         prev=None)

omni.kit.commands.execute('ChangeProperty',
                         prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:rotateXYZ",
                         value=Gf.Vec3d(0, 270, 90),
                         prev=None)
```
