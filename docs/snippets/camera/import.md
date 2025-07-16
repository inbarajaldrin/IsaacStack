## **Import Intelrealses D455**
```python
import os
from omni.isaac.core.utils.stage import add_reference_to_stage

# Import a USD file as a prim
usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Sensors/Intel/RealSense/rsd455.usd"  # Change this to your USD path
filename = os.path.splitext(os.path.basename(usd_path))[0]
prim_path = f"/World/{filename}" # Auto-name the prim using filename

add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
print(f"Prim imported at {prim_path}")
```

## **Calibrate Camera**