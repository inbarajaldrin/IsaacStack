## **Remove Prim**
```python
import omni.usd

stage = omni.usd.get_context().get_stage()
prim_path = "/World/ur5e"  # Change this to the prim you want to delete
prim = stage.GetPrimAtPath(prim_path)

stage.RemovePrim(prim_path)
```