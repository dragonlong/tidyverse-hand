# 🦾 Aero Hand Open Description — ROS 2 Package

The **Aero Hand Open Description** package provides the 3D model, URDF, and visualization configuration for **TetherIA’s Aero Hand Open** — a 7-DoF tendon-driven robotic hand designed for research and dexterous manipulation.  

---

## 🧩 Overview

This package contains:
- ✅ **URDF and meshes** for the right and left Aero Hand Open models.  
- ⚙️ **Launch files** for quick visualization in RViz.  


## ⚙️ Launch File: `display.launch.py`

This launch file loads the URDF model, publishes its joint states, and visualizes it in RViz.  

### Example Usage

```bash
ros2 launch aero_hand_open_description display.launch.py
```

## 🧪 Visualization Workflow

1. Launch RViz using this package:
   ```bash
   ros2 launch aero_hand_open_description display.launch.py
   ```
2. Move sliders in the **joint_state_publisher_gui** window to manipulate finger joints.
3. Observe the live visualization in RViz.

You can also remap the topic `/joint_states` or connect it to a live controller node (e.g., from `aero_hand_open` or `aero_hand_open_teleop`).

---

## ⚖️ License

This project is licensed under the **Apache License 2.0**.

---

<div align="center">
If you find this project useful, please give it a star! ⭐  

Built with ❤️ by <a href="https://tetheria.ai">TetherIA.ai</a>
</div>
