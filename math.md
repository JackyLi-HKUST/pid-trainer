<!-- math.md -->
# PID Trainer — Math Notes (PID Sandbox)

This file explains the **math behind the PID sandbox** and how it maps to the simulation code.

---

## 1. Discrete-Time Control Loop (per frame)

At each frame with timestep `dt` (seconds):

1. Measure the robot state and target reference  
2. Compute tracking error `e`
3. Compute PID output `u`
4. Apply realism (noise, bias, lag, saturation)
5. Update robot motion (plant dynamics)
6. Log metrics (error chart, improvement chart)

---

## 2. Error Definition (Tracking Objective)

### 2.1 Centerline / Path Tracking (lateral error)
A common definition is the robot’s lateral offset from a target line:

\[
e(t)=y_{\text{robot}} - y_{\text{target}}(x_{\text{robot}})
\]

- `e > 0` means robot is above the line (depending on your coordinate convention).
- `e < 0` means robot is below the line.

### 2.2 General Reference Tracking
If the target is a reference signal \(r(t)\) and the measured output is \(y(t)\):

\[
e(t) = r(t) - y(t)
\]

---

## 3. PID Controller (Continuous Form)

The PID control law:

\[
u(t)=K_p e(t) + K_i \int_0^t e(\tau)\, d\tau + K_d \frac{de(t)}{dt}
\]

Where:
- \(K_p\): proportional gain
- \(K_i\): integral gain
- \(K_d\): derivative gain
- \(u(t)\): control output (steering / turn command)

---

## 4. PID Controller (Discrete Implementation)

The sandbox runs in discrete time (frame-by-frame).

### 4.1 Integral term (accumulated error)
\[
I_k = I_{k-1} + e_k \, dt
\]

### 4.2 Derivative term (rate of change)
\[
D_k = \frac{e_k - e_{k-1}}{dt}
\]

### 4.3 Output (PID sum)
\[
u_k = K_p e_k + K_i I_k + K_d D_k
\]

---

## 5. Saturation and Clamping (Actuator Limits)

Real actuators have a maximum steering magnitude. Clamp output:

\[
u_k \leftarrow \text{clip}(u_k, -u_{\max}, u_{\max})
\]

Optional (recommended): integral clamping to avoid windup:

\[
I_k \leftarrow \text{clip}(I_k, -I_{\max}, I_{\max})
\]

---

## 6. Integral Windup and Anti-Windup (Practical Detail)

If the output saturates, the integral can keep growing and cause overshoot.

Two common anti-windup approaches:

### 6.1 Integral clamping
\[
I_k = \text{clip}(I_k, -I_{\max}, I_{\max})
\]

### 6.2 Conditional integration (only integrate when not saturated)
If \(u_k\) is saturated, skip updating \(I_k\) (or integrate only if it helps de-saturate).

---

## 7. Sensor Noise (Measurement Imperfection)

The measured error is often noisy:

\[
e^{\text{meas}}_k = e_k + \eta_k
\]

Where \(\eta_k\) is random noise (often Gaussian):

\[
\eta_k \sim \mathcal{N}(0,\sigma^2)
\]

Noise especially affects the derivative term (because it amplifies rapid changes).

---

## 8. Steering Bias (Systematic Imperfection)

A constant steering bias models misalignment / calibration error:

\[
u^{\text{real}}_k = u_k + b
\]

This creates steady-state error that **P/PD** may not fully eliminate.  
The **I** term is typically needed to remove it.

---

## 9. Actuator Lag / First-Order Response (Your `ease_towards`)

Instead of instantly applying \(u_k\), the steering state \(s_k\) follows with lag.

### 9.1 Continuous first-order model
\[
\dot{s}(t) = \frac{u(t) - s(t)}{\tau}
\]

- \(s(t)\): actual achieved steering
- \(u(t)\): commanded steering
- \(\tau\): time constant (bigger → slower response)

### 9.2 Discrete update (exact solution form)
\[
s_k = s_{k-1} + \alpha \left(u_k - s_{k-1}\right)
\]
\[
\alpha = 1 - e^{-dt/\tau}
\]

This matches the code pattern:
- `s = ease_towards(s, u, dt, tau)`

---

## 10. Plant Dynamics (Robot Motion Model)

A simple kinematic update:

### 10.1 Heading update
\[
\theta_k = \theta_{k-1} + \omega_k\, dt
\]

Where \(\omega_k\) is turn rate derived from steering (implementation-dependent).

### 10.2 Position update
\[
x_k = x_{k-1} + v \cos(\theta_k)\, dt
\]
\[
y_k = y_{k-1} + v \sin(\theta_k)\, dt
\]

- \(v\): forward speed
- \((x,y)\): robot position
- \(\theta\): heading angle

---

## 11. Error Chart (What You Plot)

The error chart typically shows \(e_k\) over time.

Useful interpretations:
- Large oscillations → \(K_p\) too high, or \(K_d\) too low
- Slow convergence / steady offset → \(K_i\) too low (or bias exists)
- Noisy spikes → derivative too high or sensor noise too large

---

## 12. Performance / Improvement Metric (Cycle Score)

To compare “how good” a PID setting is, define a cost \(J\).

### 12.1 Mean absolute error (MAE)
\[
J = \frac{1}{N}\sum_{k=1}^{N} |e_k|
\]

### 12.2 Root mean square error (RMSE)
\[
J = \sqrt{\frac{1}{N}\sum_{k=1}^{N} e_k^2}
\]

Lower \(J\) means better tracking.

### 12.3 Best-so-far tracking
\[
J_{\text{best}} \leftarrow \min(J_{\text{best}}, J_{\text{current}})
\]

This is what many “Improvement charts” visualize across cycles.

---

## 13. Auto Tuning: Shadow Simulation Search (Concept)

Auto-tuning often works like:

1. Propose candidate gains \((K_p, K_i, K_d)\)
2. Run a fast “shadow” simulation
3. Compute a cost \(J\)
4. Keep the best candidate
5. Repeat and refine

Formally:
\[
(K_p^*,K_i^*,K_d^*) = \arg\min_{(K_p,K_i,K_d)} J(K_p,K_i,K_d)
\]

The search method can be random search, hill climbing, evolutionary strategies, etc.

---

## 14. Practical Tuning Intuition (Quick Reference)

- Increase \(K_p\): faster response, more overshoot/oscillation risk
- Increase \(K_d\): damps oscillation, sensitive to noise
- Increase \(K_i\): removes steady-state error, can cause windup/slow oscillations

A typical workflow:
1. Tune \(K_p\) until it tracks but starts to oscillate
2. Add \(K_d\) to damp oscillation
3. Add a small \(K_i\) to eliminate bias / steady error

---

## 15. Mapping Guide (Math → Code Variables)

Common variable mapping in the sandbox:

- `error` → \(e_k\)
- `integral` → \(I_k\)
- `prev_error` → \(e_{k-1}\)
- `derivative` → \(D_k\)
- `output` / `u` → \(u_k\)
- `steer` / `steering_state` → \(s_k\)
- `tau` → \(\tau\)
- `dt` → \(dt\)
- `clamp` → \(\text{clip}(\cdot)\)

---

## 16. If You Want This to Match Your Exact Code

If you paste the exact PID update snippet (where you compute `error`, `integral`, `derivative`, `output`, and apply `ease_towards`), we can add a section:

- **“Line-by-line annotation: PID step”**
- **“Line-by-line annotation: plant update”**
- **“Line-by-line annotation: scoring / improvement chart”**
---