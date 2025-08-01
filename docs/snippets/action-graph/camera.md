## **Code to set action graph to publish a camera topic**
```python
import omni.graph.core as og

# =============== CONFIGURATION ===============
CAMERA_PRIM = "/World/UR5e/Gripper/rsd455_camera"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
ROS2_TOPIC = "isometric_camera_rgb"
# =============================================

def create_camera_ros2_actiongraph():
    """Create ActionGraph for camera ROS2 publishing"""

    graph_path = "/World/ActionGraph_01"

    print(f"Creating ActionGraph: {graph_path}")
    print(f"Camera: {CAMERA_PRIM}")
    print(f"Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
    print(f"ROS2 Topic: {ROS2_TOPIC}")

    # Create ActionGraph
    try:
        og.Controller.create_graph(graph_path)
        print(f"Created ActionGraph at {graph_path}")
    except Exception:
        print(f"ActionGraph already exists at {graph_path}")

    # Create nodes
    nodes = [
        ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
        ("isaac_run_one_simulation_frame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
        ("isaac_create_render_product", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
        ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
        ("ros2_camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
    ]

    print("\nCreating nodes...")
    for node_name, node_type in nodes:
        try:
            node_path = f"{graph_path}/{node_name}"
            og.Controller.create_node(node_path, node_type)
            print(f"Created {node_name}")
        except Exception as e:
            print(f"Node {node_name} already exists")

    # Set node attributes
    print("\nConfiguring nodes...")

    # Configure render product
    try:
        og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:cameraPrim").set([CAMERA_PRIM])
        og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:width").set(IMAGE_WIDTH)
        og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:height").set(IMAGE_HEIGHT)
        og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:enabled").set(True)
        print(f"Configured render product: {CAMERA_PRIM} @ {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
    except Exception as e:
        print(f"Error configuring render product: {e}")

    # Configure ROS2 camera helper
    try:
        og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:topicName").set(ROS2_TOPIC)
        og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:frameId").set("camera_link")
        og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:type").set("rgb")
        og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:enabled").set(True)
        og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:queueSize").set(10)
        print(f"Configured ROS2 helper: topic={ROS2_TOPIC}")
    except Exception as e:
        print(f"Error configuring ROS2 helper: {e}")

    # Create connections
    print("\nConnecting nodes...")
    connections = [
        ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
        ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
        ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
        ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
        ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
    ]

    for source, target in connections:
        try:
            og.Controller.connect(f"{graph_path}/{source}", f"{graph_path}/{target}")
            print(f"Connected {source.split('.')[0]} -> {target.split('.')[0]}")
        except Exception as e:
            print(f"Failed to connect {source} -> {target}: {e}")

    print("\nActionGraph created successfully!")

if __name__ == "__main__":
    print("Creating Camera ROS2 ActionGraph...")
    print("=" * 50)
    create_camera_ros2_actiongraph()

```