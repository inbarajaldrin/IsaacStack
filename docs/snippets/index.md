# **Isaac Sim Snippets**

A collection of minimal, ready-to-use code blocks for common Isaac Sim tasks.

---

## **Initialize simulation and add a ground plane**

```python
import asyncio
from omni.isaac.core.world import World

async def load_scene():
    world = World()
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()
    print("Scene loaded successfully.")

# Run the function
asyncio.ensure_future(load_scene())
```

---

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
