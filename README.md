# PX4_TELEOP

A Ros2 teleoperation package for PX4-based UAVs using MAVROS. This is the **multiagent branch**, which supports coordinated control of a swarm of agents.

## Overview
'px4_teleop' provides a joystick-driven teleoperation interface for PX4 flight controllers. It handles arming, takeoff, landing, flight mode switching, and velocity commands through a mapped Xbox controller. In the multiagent branch, the operator can switch between connected agents at runtime, while the package tracks neighbor states (pose, velocity, flight mode) across the swarm.

A 'JoyHandler' interface is provided for interpretting joystick commands.

The 'ExperimentRunner' interface provides a structure for autonomous experiments (e.g. leader-follower), triggered via ROS2 service calls defined in the 'swarm_interfaces' library.

---

## Dependencies

| Package | Notes |
|---|---|
| `rclcpp` | ROS 2 core |
| `mavros` / `mavros_msgs` | PX4 bridge |
| `sensor_msgs`, `geometry_msgs`, `geographic_msgs` | Standard message types |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs` | Coordinate transforms |
| `visualization_msgs` | RViz markers |
| `px4_safety_lib` | Safety envelope enforcement |
| `swarm_interfaces` | Custom swarm coordination messages |
| `fleet_manager` | Agent discovery and connection tracking |

---

## Package Structure

```
px4_teleop/
├── include/
│   ├── PX4Teleop.hpp        # Main teleoperation node
│   ├── JoyHandler.hpp       # Xbox controller input parser
│   └── ExperimentRunner.hpp # Abstract base for experiment plugins
├── src/
│   ├── PX4Teleop.cpp
│   ├── JoyHandler.cpp
│   ├── px4_teleop_node.cpp  # Node entry point
│   └── Experiment/          # Experiment implementations
├── launch/
│   ├── joy.launch.py                  # Joystick driver only
│   ├── n1.launch.py                   # Single agent (namespace: n1)
│   ├── n2.launch.py                   # Single agent (namespace: n2)
│   ├── n3.launch.py                   # Single agent (namespace: n3)
│   ├── n4.launch.py                   # Single agent (namespace: n4)
│   ├── sim_teleop_viz.launch.py       # Simulation + RViz
│   └── astro3_teleop_viz.launch.py    # Hardware (Astro3) + RViz
└── param/
    ├── button_config.yaml     # Xbox button index mapping
    ├── joy_config.yaml        # Joystick driver settings
    ├── teleop_config.yaml     # Velocity limits
    ├── experiment_test.yaml   # Example experiment parameters
    ├── sim_obstacles.yaml     # Simulation obstacle definitions
    └── astro3_obstacles.yaml  # Hardware obstacle definitions
```

---
