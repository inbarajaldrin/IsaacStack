## **Apply Rigid body preset**
```python
from pxr import UsdPhysics, Usd, UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()
prim_path = "/World/TestCube"
prim = stage.GetPrimAtPath(prim_path)

if prim.IsValid() and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    prim.GetAttribute("physics:rigidBodyEnabled").Set(True)
    print(f"Applied RigidBodyAPI to: {prim_path}")
else:
    print(f"Prim not found or already has RigidBodyAPI: {prim_path}")
```
     **Applies `UsdPhysics.RigidBodyAPI` to a prim, enabling dynamic simulation and gravity response within the physics scene.**
## **Apply Colliders preset**
```python
from pxr import UsdPhysics, UsdGeom, Usd
import omni.usd

stage = omni.usd.get_context().get_stage()
prim_path = "/World/TestCube"
prim = stage.GetPrimAtPath(prim_path)

if prim.IsValid():
    for desc in Usd.PrimRange(prim):
        if desc.IsA(UsdGeom.Mesh) or desc.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(desc).CreateCollisionEnabledAttr(True)
            print(f"Collider enabled on: {desc.GetPath()}")
else:
    print(f"Prim not found at: {prim_path}")
```
        **Applies `UsdPhysics.CollisionAPI` to enable collision detection on mesh or geometric prims for physical interaction.**

## **Apply Colliders preset with ConvexDecomposition**
```python
from pxr import UsdPhysics, UsdGeom, PhysxSchema, Usd
import omni.usd

stage = omni.usd.get_context().get_stage()
prim_path = "/World/TestCube"
root_prim = stage.GetPrimAtPath(prim_path)

if not root_prim.IsValid():
    print(f"Prim not found at {prim_path}")
else:
    for desc in Usd.PrimRange(root_prim):
        if desc.IsA(UsdGeom.Mesh) or desc.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(desc).CreateCollisionEnabledAttr(True)
            PhysxSchema.PhysxCollisionAPI.Apply(desc).CreateRestOffsetAttr(0.0)
            UsdPhysics.MeshCollisionAPI.Apply(desc).CreateApproximationAttr().Set("convexDecomposition")
            print(f"Collider set on: {desc.GetPath()}")
```
        **Enables collision using PhysX convex decomposition for accurate shape approximation on meshes.**


<div style="display: flex; gap: 1.5rem; flex-wrap: wrap; margin-top: 1rem;">

  <div style="flex: 1; min-width: 300px;">
    <p><strong>Without Convex Decomposition</strong></p>
    <video width="100%" controls>
      <source src="../assets/convexDecomposition1.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>

  <div style="flex: 1; min-width: 300px;">
    <p><strong>With Convex Decomposition</strong></p>
    <video width="100%" controls>
      <source src="../assets/convexDecomposition2.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>

</div>
