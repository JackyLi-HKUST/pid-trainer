# PID Trainer
**By Jacky Li from HKUST**

An interactive simulator for learning **PID control** by tuning a differential-drive “robot” to track a moving centerline.  
Includes realistic imperfections (sensor noise, actuator lag, steering bias), real-time plots, and an automatic tuning method: **Shadow Simulation Search**.

![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Pygame](https://img.shields.io/badge/pygame-2.5%2B-orange)
![License](https://img.shields.io/badge/license-MIT-green)

> **Demo** (replace with your own file)
>
> ![Demo](assets/demo.gif)

---

## Why this project exists (educational goal)
This project is designed like a mini **university control lab**:
- Discrete-time **PID** (sampling effects, integral windup protection, derivative noise sensitivity)
- **Lookahead tracking** geometry (error defined ahead of the robot for smoother path following)
- **Differential-drive kinematics**: turning wheel PWM into forward speed and yaw rate
- “Reality” in simulation: **sensor noise**, **actuator lag** (first-order response), **steering bias**
- Auto-tuning idea: evaluate candidate PID gains by running **shadow rollouts** in simulation

---

## Features
- Real-time **Error chart** (body error + measured lookahead error)
- **Improvement chart** per cycle (average cycle error + best-so-far)
- Manual tuning mode: Base PWM / Kp / Ki / Kd / Trim
- AUTO tuning: **Shadow Simulation Search** (champion vs challenger with validation)
- Auto HOLD logic when error remains below a target for a period

---

## Install & Run

### 1) Create environment (recommended)
```bash
python -m venv .venv
source .venv/bin/activate   # macOS/Linux
# .venv\Scripts\activate    # Windows
