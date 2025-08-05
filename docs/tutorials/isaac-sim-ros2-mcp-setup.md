# Isaac Sim + ROS 2 MCP Integration

---

## Isaac Sim MCP Extension

[Youtube video- IsaacSim MCP integration on Linux](https://youtu.be/8LsHCGvJaek?si=ge73Z-tm0e_R7Tos)

### Repo:  
[https://github.com/inbarajaldrin/isaac-sim-mcp](https://github.com/inbarajaldrin/isaac-sim-mcp)

This repo includes:

- MCP setup instructions
- Claude Desktop integration
- Example agent workflows

If you're using **Claude Desktop**, follow the full setup instructions in the repo and video.

If your using API calls

```bash
python3 isaac_mcp/main.py
```
---
## ROS 2 MCP Server

### Repo:

[ros-mcp-server](https://github.com/inbarajaldrin/ros-mcp-server)


This repo includes:

- MCP setup instructions
- rosbridge_suite installation instructions

Then launch the WebSocket server:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## URSim / Real Robot Configuration

> In both URSim and real robot, **enable the URCap** to allow external agent control.

### UR Primitives
[https://github.com/karishmapatnaik/ur_asu](https://github.com/karishmapatnaik/ur_asu)

update the location of the file [https://github.com/karishmapatnaik/ur_asu/blob/main/ur_asu/custom_libraries/ik_solver.py](https://github.com/karishmapatnaik/ur_asu/blob/main/ur_asu/custom_libraries/ik_solver.py) in your directory in this file [https://github.com/inbarajaldrin/isaac-sim-mcp/blob/main/isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py](https://github.com/inbarajaldrin/isaac-sim-mcp/blob/main/isaac.sim.mcp_extension/isaac_sim_mcp_extension/extension.py) and this file [https://github.com/inbarajaldrin/ros-mcp-server/blob/main/server.py](https://github.com/inbarajaldrin/ros-mcp-server/blob/main/server.py)