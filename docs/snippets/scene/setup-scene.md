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
