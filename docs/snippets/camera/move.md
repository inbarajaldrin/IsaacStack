
## **Move camera**
```python
import omni.usd
from pxr import Gf

def move_camera(camera_prim_path, x, y, z):
    """
    Move camera to new position using xformOp:translate
    
    Args:
        camera_prim_path: USD path to camera (e.g. "/World/topdown_camera")
        x, y, z: New position coordinates
    """
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    
    if camera_prim:
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))
        print(f"Camera moved to ({x}, {y}, {z})")
    else:
        print(f"Camera not found at {camera_prim_path}")

def rotate_camera(camera_prim_path, w, x, y, z):
    """
    Rotate camera using xformOp:orient quaternion
    
    Args:
        camera_prim_path: USD path to camera
        w, x, y, z: Quaternion components (w is real part)
    """
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    
    if camera_prim:
        camera_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(w, x, y, z))
        print(f"✓ Camera orientation set to ({w}, {x}, {y}, {z})")
    else:
        print(f"Camera not found at {camera_prim_path}")

def rotate_camera_degrees(camera_prim_path, roll, pitch, yaw):
    """
    Rotate camera using degrees (roll, pitch, yaw)
    
    Args:
        camera_prim_path: USD path to camera
        roll, pitch, yaw: Rotation in degrees
    """
    import math
    
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    # Convert Euler angles to quaternion
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    
    if camera_prim:
        camera_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(w, x, y, z))
        print(f"Camera rotated: roll={roll}°, pitch={pitch}°, yaw={yaw}°")
    else:
        print(f"Camera not found at {camera_prim_path}")

def set_focal_length(camera_prim_path, focal_length):
    """
    Set camera focal length
    
    Args:
        camera_prim_path: USD path to camera
        focal_length: Focal length in mm (float)
    """
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    
    if camera_prim:
        camera_prim.GetAttribute("focalLength").Set(float(focal_length))
        print(f"Focal length set to {focal_length}mm")
    else:
        print(f"Camera not found at {camera_prim_path}")

if __name__ == "__main__":
    camera_path = "/World/topdown_camera"  # Change this to your camera path
    
    # Move camera
    move_camera(camera_path, 2, 3, 5)
    
    # Set focal length to 35mm
    set_focal_length(camera_path, 35.0)
    
    # Top-down orientation using degrees (default camera orientation)
    rotate_camera_degrees(camera_path, 0, 0, 0)  # 0,0,0 = looking down
```

## **Point towards an object with a set focal distance**

