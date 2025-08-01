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

## **Set Restitution to deal with bounciness of object**

Youtube Tutorial:  [Omniverse Physics Extension - Kit104 - Part 9: Materials - Friction Restitution and Defaults ](https://youtu.be/tHOM-OCnBLE?si=Xxj7BmRoyHycmATc)

```python
from pxr import UsdPhysics, PhysxSchema, UsdShade
import omni.usd

stage = omni.usd.get_context().get_stage()

# === Create Physics Material (Low Bounce for Jenga) ===
mat_path = "/World/JengaPhysMat"
mat_prim = UsdShade.Material.Define(stage, mat_path).GetPrim()

# Apply physics material APIs
UsdPhysics.MaterialAPI.Apply(mat_prim)
PhysxSchema.PhysxMaterialAPI.Apply(mat_prim)

# Set material properties for realistic wood behavior
UsdPhysics.MaterialAPI(mat_prim).CreateRestitutionAttr().Set(0.1)      # Very low bounce
UsdPhysics.MaterialAPI(mat_prim).CreateStaticFrictionAttr().Set(0.7)   # Good grip when stationary
UsdPhysics.MaterialAPI(mat_prim).CreateDynamicFrictionAttr().Set(0.6)  # Sliding friction

# PhysX-specific settings
PhysxSchema.PhysxMaterialAPI(mat_prim).CreateRestitutionCombineModeAttr().Set(PhysxSchema.Tokens.min)  # Use minimum restitution
PhysxSchema.PhysxMaterialAPI(mat_prim).CreateFrictionCombineModeAttr().Set(PhysxSchema.Tokens.average)

# === Apply Material to Jenga Block ===
cube_path = "/World/JengaBlock2"
cube_prim = stage.GetPrimAtPath(cube_path)

if cube_prim:
    UsdShade.MaterialBindingAPI.Apply(cube_prim).Bind(UsdShade.Material(mat_prim))
    print(f"Low-bounce physics material applied to {cube_path}")
else:
    print(f"Prim not found at {cube_path}")
```
        **Enabling restitution enables accurate pick and place sequence.**

<div style="display: flex; gap: 1.5rem; flex-wrap: wrap; margin-top: 1rem;">

  <div style="flex: 1; min-width: 300px;">
    <p><strong>With no Restitution set</strong></p>
    <video width="100%" controls>
      <source src="../assets/without_restitution.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>

  <div style="flex: 1; min-width: 300px;">
    <p><strong>With Restitution=0.7</strong></p>
    <video width="100%" controls>
      <source src="../assets/with_restitution.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>

</div>