# Math Notes — PID Equation Tuning Trainer

This document explains the modeling and control math used in the simulator.

---

## 1. Coordinates and track model

The world is in pixel units. The track is defined as a scrolling sinusoid:

\[
y_{track}(x) = y_0 + A\sin\left(k(x+\phi)\right),\quad k=\frac{2\pi}{\lambda}
\]

- \(y_0\): vertical center (`TRACK_CENTER_Y`)
- \(A\): amplitude (`TRACK_AMP`)
- \(\lambda\): wavelength (`TRACK_WAVELEN`)
- \(\phi\): phase term, updated over time by `PHASE_SPEED`

The local track heading can be estimated by the slope:

\[
\frac{dy}{dx} = Ak\cos(k(x+\phi)),\quad \theta_{track}=\arctan\left(\frac{dy}{dx}\right)
\]

---

## 2. Lookahead tracking error

Robot pose:
\[
(x,y,\theta)
\]

Lookahead point at distance \(L\):
\[
x_{la}=x+L\cos\theta,\quad y_{la}=y+L\sin\theta
\]

Tracking error is vertical difference between the track centerline and the lookahead point:
\[
e = y_{track}(x_{la}) - y_{la}
\]

Why lookahead helps:
- It reduces “late corrections” that can cause oscillation.
- It behaves like a mild anticipatory action (similar to adding phase lead).

---

## 3. Discrete PID controller and sampling

Control is updated at a discrete rate (`CTRL_HZ`), so \(\Delta t = 1/\text{CTRL\_HZ}\).

Integral:
\[
I_k = I_{k-1} + e_{k}\Delta t
\]

Derivative (finite difference):
\[
D_k = \frac{e_k - e_{k-1}}{\Delta t}
\]

Control law:
\[
u_k = K_p e_k + K_i I_k + K_d D_k
\]

### Notes on stability and noise
- Differentiation amplifies high-frequency noise. If sensor noise is large, \(K_d\) can cause jitter.
- Integral can “wind up” if error persists. The simulator clamps integral state to reduce runaway:
  \[
  I \in [-I_{max}, I_{max}]
  \]

---

## 4. Noise model

Measured error:
\[
e_{meas} = e + n,\quad n \sim \mathcal{N}(0,\sigma^2)
\]

This captures the idea that sensors are not perfect. A key learning point:
- With higher noise, derivative terms often need to be reduced or filtered.

---

## 5. PWM mapping and trim

The controller output \(u\) acts as a steering command.

With base PWM \(B\) and trim:
\[
PWM_L = \text{clamp}(B - u + trim, 0, 255)
\]
\[
PWM_R = \text{clamp}(B + u - trim, 0, 255)
\]

Interpretation:
- If error says “move up,” \(u\) pushes the robot to turn by changing differential wheel speeds.
- `trim` is like a constant bias to counter systematic drift.

---

## 6. Actuator lag (first-order system)

Motors do not instantly reach the target PWM. The simulator models this using exponential smoothing:

\[
PWM \leftarrow PWM + \alpha(PWM_{target} - PWM),\quad \alpha = 1-e^{-\Delta t/\tau}
\]

- \(\tau\) is a time constant (`ACTUATOR_LAG_TAU`)
- Larger \(\tau\) means slower response

Learning point:
- A controller tuned for a “perfect actuator” might oscillate when actuator lag exists.

---

## 7. Differential-drive kinematics

Let wheel speeds be \(v_L, v_R\), wheelbase be \(b\).

Forward speed:
\[
v = \frac{v_L + v_R}{2}
\]

Yaw rate:
\[
\omega = \frac{v_R - v_L}{b}
\]

Pose update:
\[
\theta \leftarrow \theta + \omega\Delta t
\]
\[
x \leftarrow x + v\cos\theta\Delta t,\quad y \leftarrow y + v\sin\theta\Delta t
\]

---

## 8. “Body error” (front and rear reference points)

The simulator also measures error at the robot body front and rear points and averages them:

\[
e_F = y_{track}(x_F) - y_F,\quad e_R = y_{track}(x_R) - y_R
\]
\[
e_{body} = \frac{e_F + e_R}{2}
\]

This gives a more “physical” sense of how the body is aligned with the track.

---

## 9. Shadow Simulation Search (auto tuning)

AUTO mode proposes candidate gains and tests them in simulation rollouts.

### 9.1 Two-stage evaluation
1) Quick rollout: short horizon to cheaply find promising candidates  
2) Full-cycle rollout: longer validation to reduce false improvements

### 9.2 Cost function meaning
The cost used in rollout is a weighted sum:

- \(e^2\): tracking error energy
- \(\omega^2\): penalize fast spins
- \((\omega/(|v|+\epsilon))^2\): penalize spinning when not moving forward (common unstable behavior)
- \(u^2\): penalize large control actions
- \((\Delta u)^2\): penalize rapid changes (jitter)

A typical form:
\[
J = w_e e^2 + w_{\omega}\omega^2 + w_r\left(\frac{\omega}{|v|+\epsilon}\right)^2 + w_u u^2 + w_{\Delta u}(\Delta u)^2
\]

Lower is better.

### 9.3 Champion vs challenger swap rule
The challenger must be better by a margin:
\[
J_{chall} < \text{margin}\cdot J_{champ}
\]
This avoids swapping due to noise or tiny differences.

---

## 10. What to experiment with (lab-style questions)

1) Increase sensor noise: how do optimal \(K_d\) values change?  
2) Increase actuator lag: what happens to overshoot and oscillation?  
3) Try P-only, PD, PI, PID: which feels most stable and why?  
4) Change lookahead distance \(L\): does it trade responsiveness for stability?  
5) Change cost weights: can you make auto prefer “smooth” over “aggressive”?

---