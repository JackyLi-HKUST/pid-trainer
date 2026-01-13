# PID Trainer
**By Jacky Li from HKUST**

An interactive sandbox for learning **PID control** by tuning a differential-drive “robot” to track a moving centerline.  
**Lightweight and simple**, but still includes realistic imperfections (sensor noise, actuator lag, steering bias), real-time plots, and automatic tuning methods/algorithms (more coming soon).

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![Pygame](https://img.shields.io/badge/Pygame-2.5%2B-1f1f1f)
![License: MIT](https://img.shields.io/badge/License-MIT-green)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS%20%7C%20Linux-lightgrey)
![Status](https://img.shields.io/badge/Status-Active%20Development-blue)


## Quick Demo
<details>
  <summary><b>▶ Click to watch </b></summary>
  
https://github.com/user-attachments/assets/d381ae2f-4b01-4fbb-9070-9d1a7aeb0f99

</details>

## Features

### Manual tuning (interactive)
- Live adjustment of **Kp / Ki / Kd** and **Trim**
- Immediate feedback on:
  - tracking error (body + lookahead)
  - oscillation / overshoot
  - control effort and smoothness

### AUTO mode (v1): Shadow Simulation Search
- Maintains a **Champion** (best-known gains) and proposes **Challengers**
- Candidate evaluation pipeline:
  - **Quick horizon cost** (fast screening)
  - **Full-cycle cost** (validation across one track wavelength)
- Swaps only if the challenger is meaningfully better (configurable margin)

### Visualization / instrumentation
- **Error chart:** true body error + noisy measured lookahead error
- **Improvement chart:** cycle-average error vs best-so-far + swap markers
- Smooth HUD readouts + cycle progress in AUTO

---


## Requirements
- Python **3.10+**
- `pygame` (see `requirements.txt`)

---

## Installation

    python -m venv .venv
    source .venv/bin/activate   # macOS/Linux
    # .venv\Scripts\activate    # Windows
    pip install -r requirements.txt



---

## Run

    python main.py

---

## Controls

### Global
- `ESC` — quit
- `SPACE` — pause / resume
- `R` — reset simulation
- `A` — toggle AUTO (`OFF` ↔ `LEARN`)

### Manual mode (AUTO = OFF)
- `UP / DOWN` — select parameter **B / P / I / D / T**
- `LEFT / RIGHT` — adjust selected parameter

**Parameter mapping**
- `B` = Base PWM (speed baseline)
- `P` = Kp
- `I` = Ki
- `D` = Kd
- `T` = Trim (left/right bias compensation)

---

## How AUTO Works (Shadow Simulation Search)

AUTO is intentionally designed to be simple and transparent so learners can read and modify the method.

At the end of each “cycle” (≈ one track wavelength), the algorithm:
1. Samples candidate gain sets around the current best **Champion** (and current gains)
2. Uses a **short-horizon shadow simulation** to quickly choose a promising **Challenger**
3. Runs a **full-cycle shadow validation** for both champion and challenger
4. Swaps only if the challenger is better by a margin:

    pred_chall < pred_champ * SWAP_MARGIN

### Cost function (high level)
The shadow simulator accumulates weighted penalties for:
- tracking error (squared)
- excessive spin (`omega^2`)
- spin rate normalized by speed
- control magnitude (`u^2`)
- control change (`(Δu)^2`)

Weights you can tune in the code:
- `W_E2, W_SPIN2, W_SPINR2, W_U2, W_DU2`

---

## Suggested Project Structure

    PID-Trainer/
    ├─ pid_trainer/
    │  ├─ __init__.py
    │  ├─ __main__.py
    │  ├─ app.py
    │  ├─ settings.py
    │  ├─ utils.py
    │  ├─ track.py
    │  ├─ sim.py
    │  ├─ shadow.py
    │  └─ auto.py
    ├─ README.md
    ├─ LICENSE
    ├─ requirements.txt
    └─ assets/
       └─ demo.gif

---

## Roadmap
Planned additions (future updates):
- Additional **auto PID search/tuning methods**, such as:
  - random search baselines (for comparison)
  - grid / Bayesian-style search
  - gradient-free optimizers (CMA-ES / NES-style)
  - classical heuristics (e.g., Ziegler–Nichols variants) adapted to this simulation
- More environments / tracks:
  - step disturbances, sharper turns, different track shapes
  - speed schedules and constraints
- Better reproducibility & logging:
  - deterministic seeds
  - export gain history + metrics

---

## Educational Notes
PID Trainer highlights common real-world issues that affect tuning:
- noisy measurements (`SENSOR_NOISE_STD`)
- actuator dynamics (`ACTUATOR_LAG_TAU`)
- small systematic biases (`STEERING_BIAS`)
- trade-offs between aggressiveness and smoothness

This project is **not** intended as a physically exact model of a specific robot platform. It is a controllable sandbox that behaves “realistically enough” for learning and algorithm comparisons.

---

## Contributing
Contributions are welcome:
- new auto-tuning strategies
- improved cost design / metrics
- UI/UX improvements
- refactoring into modules + tests

Suggested workflow:
1. Fork the repo
2. Create a feature branch
3. Open a PR with:
   - summary of changes
   - before/after screenshots or a short GIF for UI changes

---

## Citation
If you use this project in teaching, demos, or research prototypes, please cite:

    @software{pid_trainer,
      title        = {PID Trainer},
      author       = {Jacky Li},
      year         = {2026},
      note         = {Interactive PID tuning sandbox},
    }

---

## License
Recommended options:
- **MIT** (simple, permissive)
- **Apache-2.0** (includes explicit patent grant)

Add a `LICENSE` file accordingly.

---

## Acknowledgements
- Built with **Pygame**
- Inspired by classic control education labs and practical tuning workflows
