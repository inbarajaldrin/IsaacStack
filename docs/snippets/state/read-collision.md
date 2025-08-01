
## **Setup Two cubes with collision**

```python
import omni.kit.commands
from omni.physx.scripts.physicsUtils import set_or_add_translate_op
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Usd, Gf
import omni.physxdemos as demo

def add_minimal_collision(stage, prim_path):
    """Add only collision - no rigid body physics"""
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        # Add collision to mesh children - this might be all we need
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh) or desc.IsA(UsdGeom.Gprim):
                UsdPhysics.CollisionAPI.Apply(desc).CreateCollisionEnabledAttr(True)
                PhysxSchema.PhysxCollisionAPI.Apply(desc).CreateRestOffsetAttr(0.0)
                UsdPhysics.MeshCollisionAPI.Apply(desc).CreateApproximationAttr().Set("convexDecomposition")

def spawn_two_cubes_minimal(stage):
    # Setup physics scene (needed for query interface)
    defaultPrimPath, scene = demo.setup_physics_scene(None, stage)
    
    # Create first cube
    result, cube1_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Cube", select_new_prim=False)
    final_cube1_path = defaultPrimPath + "/cube1"
    omni.kit.commands.execute("MovePrim", path_from=cube1_path, path_to=final_cube1_path)
    
    cube1_mesh = UsdGeom.Mesh.Get(stage, final_cube1_path)
    set_or_add_translate_op(cube1_mesh, translate=Gf.Vec3f(-50.0, 0.0, 0.0))
    
    # Add only collision (no rigid body)
    add_minimal_collision(stage, final_cube1_path)
    
    # Create second cube
    result, cube2_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Cube", select_new_prim=False)
    final_cube2_path = defaultPrimPath + "/cube2"
    omni.kit.commands.execute("MovePrim", path_from=cube2_path, path_to=final_cube2_path)
    
    cube2_mesh = UsdGeom.Mesh.Get(stage, final_cube2_path)
    set_or_add_translate_op(cube2_mesh, translate=Gf.Vec3f(50.0, 0.0, 0.0))
    
    # Add only collision (no rigid body)
    add_minimal_collision(stage, final_cube2_path)
    
    print(f"Spawned cubes with minimal collision at: {final_cube1_path} and {final_cube2_path}")
    return final_cube1_path, final_cube2_path

# Usage
# stage = omni.usd.get_context().get_stage()
# cube1_path, cube2_path = spawn_two_cubes_minimal(stage)
```

## **Check Collision**

```python
from omni.physx import get_physx_scene_query_interface
from pxr import Sdf, PhysicsSchemaTools

def check_collision(prim1_path, prim2_path):
    """
    Check if two prims are colliding
    
    Args:
        prim1_path (str): Full path to first prim (e.g., "/World/cube1")
        prim2_path (str): Full path to second prim (e.g., "/World/cube2")
    
    Returns:
        bool: True if collision detected, False otherwise
    """
    collision_detected = False
    target_prim_path = prim2_path
    
    def collision_callback(hit):
        nonlocal collision_detected, target_prim_path
        if hit.rigid_body == target_prim_path:
            collision_detected = True
            print(f"COLLISION DETECTED between {prim1_path} and {prim2_path}")
        return True
    
    # Convert paths for PhysX query
    path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(prim1_path))
    
    # Perform overlap query
    get_physx_scene_query_interface().overlap_mesh(
        path_tuple[0], 
        path_tuple[1], 
        collision_callback, 
        False
    )
    
    return collision_detected

# Usage example:
# collision_found = check_collision("/World/cube1", "/World/cube2")
# if collision_found:
#     print("Cubes are colliding!")
# else:
#     print("No collision detected.")
```