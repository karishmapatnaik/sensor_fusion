# 🧭 Sensor Fusion Tutorial

This repository contains sample codes for learning and experimenting with **sensor fusion** and **control** concepts using both **Python (ROS 2)** and **MATLAB**.

- 🐍 **Python files** can be run directly on your system or in **Google Colab**.  
- 💻 **MATLAB files** require version **R2023a or later**.
- 💻 **ROS2 files** require version **ROS2 Humble**.  

---

## 📂 ROS 2 Nodes

This tutorial demonstrates a simple closed-loop control setup using a **Kalman filter**, **PID controller**, and **double-integrator simulation**.

| File | Description |
|------|--------------|
| `state_estimate_ros2.py` | Implements a Kalman Filter for state estimation (requires `filterpy`). |
| `double_integrator_sim.py` | Simulates a double-integrator system and publishes noisy measurements. |
| `pid_controller.py` | Runs a PID control law to regulate position based on estimated states. |

---

## ⚙️ Prerequisites

Ensure that you have:
- **ROS 2** installed (tested with Humble +)
- The following Python packages:
  ```bash
  pip install filterpy numpy matplotlib

## ▶️ How to Run

Open three terminals, and in each one, **source** your ROS 2 workspace and navigate to ```<path to python files in the ROS2 python package>```
Run the nodes in this order:

🖥️ Terminal 1 — Simulation
 ```bash
  python3 double_integrator_sim.py
```

🧮 Terminal 2 — State Estimation (Kalman Filter)
  ```bash
  python3 state_estimate_ros2.py
```
⚙️ Terminal 3 — PID Control
  ```bash
python3 pid_controller.py
```
## 📊 Results
After the simulation ends, two figures are automatically saved in the run directory:
- `position_response.png` – Position vs. Time  
- `control_effort.png` – Control Input vs. Time

These plots visualize the closed-loop response of the double-integrator system under Kalman-based state estimation and PID control.
