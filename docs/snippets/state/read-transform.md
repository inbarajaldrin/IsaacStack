
## Translation
```python
prim_path = "/World/your_object_name"

import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(prim_path)

translate_attr = prim.GetAttribute("xformOp:translate")
position = [0.0, 0.0, 0.0]
if translate_attr.IsValid():
    translate_value = translate_attr.Get()
    if translate_value:
        position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]

print(f"Translate: {position}")
```
## Orientation
```python
prim_path = "/World/your_object_name"

import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(prim_path)

orient_attr = prim.GetAttribute("xformOp:orient")
quaternion = [1.0, 0.0, 0.0, 0.0]
if orient_attr.IsValid():
    orient_value = orient_attr.Get()
    if orient_value:
        quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                     float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]

print(f"Orient [w,x,y,z]: {quaternion}")
```
## Scale
```python 
prim_path = "/World/your_object_name"

import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(prim_path)

scale_attr = prim.GetAttribute("xformOp:scale")
scale = [1.0, 1.0, 1.0]
if scale_attr.IsValid():
    scale_value = scale_attr.Get()
    if scale_value:
        scale = [float(scale_value[0]), float(scale_value[1]), float(scale_value[2])]

print(f"Scale: {scale}")
```