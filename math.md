<!-- math.md -->
# PID Trainer — Math

> This page is written to render cleanly on GitHub using LaTeX math blocks.
> If your GitHub view doesn’t render math, see **Appendix: Fallback (SVG)** at the end.

---

## 0. Notation

- Discrete time index: \(k = 0,1,2,\dots\)  
- Timestep: \(dt > 0\) (seconds per frame)  
- Error: \(e_k\)  
- Integral state: \(I_k\)  
- Derivative estimate: \(D_k\)  
- Control output (command): \(u_k\)  
- Applied steering after lag: \(s_k\)

---

## 1. Closed-loop system (what the sandbox simulates)

A typical sandbox loop is:

$$
\text{Sensor} \rightarrow e_k \rightarrow \text{PID}(K_p,K_i,K_d) \rightarrow u_k
\rightarrow \text{Actuator dynamics} \rightarrow s_k \rightarrow \text{Robot dynamics} \rightarrow \text{Sensor}
$$

---

## 2. Error definition (tracking objective)

### 2.1 Reference tracking (generic)
If the desired reference is \(r_k\) and the measured output is \(y_k\):

$$
e_k = r_k - y_k
$$

### 2.2 Centerline / path tracking (common in the sandbox)
If the robot is at \((x_k, y_k)\) and the target centerline is \(y = y_\text{ref}(x)\):

$$
e_k = y_k - y_\text{ref}(x_k)
$$

> Depending on your coordinate system, the sign convention may flip.
> The important part is consistency: “positive error” should mean “steer one way.”

---

## 3. PID controller (continuous-time form)

The classic PID law:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}
$$

- \(K_p\): proportional gain  
- \(K_i\): integral gain  
- \(K_d\): derivative gain

---

## 4. PID controller (discrete implementation)

### 4.1 Integral update
A standard rectangular (Euler) integration:

$$
I_k = I_{k-1} + e_k \, dt
$$

### 4.2 Derivative estimate
A first-order backward difference:

$$
D_k = \frac{e_k - e_{k-1}}{dt}
$$

### 4.3 PID output
The discrete PID output:

$$
u_k = K_p e_k + K_i I_k + K_d D_k
$$

---

## 5. Saturation (steering limits) and clamping

Real steering/turning has limits. Clamp the output:

$$
u_k \leftarrow \mathrm{clip}(u_k,\,-u_{\max},\,u_{\max})
$$

where

$$
\mathrm{clip}(x,a,b)=\max(a,\min(x,b))
$$

### 5.1 Integral windup (why it happens)
If \(u_k\) is saturated but the integrator keeps accumulating, then \(I_k\) can grow large and cause overshoot.

### 5.2 Anti-windup (simple and effective)

**Method A — integral clamping**
$$
I_k \leftarrow \mathrm{clip}(I_k,\,-I_{\max},\,I_{\max})
$$

**Method B — conditional integration (only integrate when it helps)**
One practical idea:

- If \(u_k\) is saturated at \(+u_{\max}\) and \(e_k > 0\), skip integration  
- If \(u_k\) is saturated at \(-u_{\max}\) and \(e_k < 0\), skip integration  
- Otherwise integrate normally

This reduces runaway integral growth.

---

## 6. Sensor noise (measurement imperfection)

Often you measure a noisy error:

$$
e_k^{\text{meas}} = e_k + \eta_k
$$

A common noise model:

$$
\eta_k \sim \mathcal{N}(0,\sigma^2)
$$

**Important:** Derivatives amplify noise, so a noisy \(e_k\) can make \(D_k\) jittery.

---

## 7. Steering bias (systematic imperfection)

A fixed bias term models misalignment or calibration errors:

$$
u_k^{\text{real}} = u_k + b
$$

Bias often causes **steady-state error** if you use only P or PD.  
A small \(K_i\) is usually what removes it.

---

## 8. Actuator lag (first-order steering dynamics)

In many sandboxes, the steering doesn’t change instantly.  
A standard actuator model is first-order lag:

### 8.1 Continuous-time actuator model
$$
\dot{s}(t)=\frac{u(t)-s(t)}{\tau}
$$

- \(s(t)\): actual steering state  
- \(u(t)\): commanded steering  
- \(\tau\): time constant (larger \(\tau\) = slower response)

### 8.2 Discrete-time exact update (matches “ease_towards”)
Solving the ODE over a timestep \(dt\) gives:

$$
s_k = s_{k-1} + \alpha\,(u_k - s_{k-1})
$$

where

$$
\alpha = 1 - e^{-dt/\tau}
$$

This is why your code often looks like:
- `s = ease_towards(s, u, dt, tau)`

---

## 9. Robot motion (plant dynamics)

A common “unicycle / kinematic” model:

### 9.1 Heading update
If \(s_k\) acts like turn rate \(\omega_k\) (or proportional to it):

$$
\theta_k = \theta_{k-1} + \omega_k\,dt
$$

### 9.2 Position update
With forward speed \(v\):

$$
x_k = x_{k-1} + v\cos(\theta_k)\,dt
$$
$$
y_k = y_{k-1} + v\sin(\theta_k)\,dt
$$

> Some sandboxes use “steering angle” rather than “turn rate”. The idea is the same:
> steering influences heading change; heading plus speed updates position.

---

## 10. Error chart (what you see and how to read it)

You plot \(e_k\) over time.

Typical patterns:
- Large oscillations: \(K_p\) too high, or \(K_d\) too low  
- Slow correction: \(K_p\) too low  
- Steady offset / drift: bias exists and \(K_i\) too low  
- Noisy spikes: sensor noise high, or \(K_d\) too high  

---

## 11. Performance metrics (improvement chart)

To compare PID settings, define a cost \(J\) over a time window of \(N\) frames.

### 11.1 Mean Absolute Error (MAE)
$$
J_{\text{MAE}}=\frac{1}{N}\sum_{k=1}^{N}\left|e_k\right|
$$

### 11.2 Root Mean Square Error (RMSE)
$$
J_{\text{RMSE}}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_k^2}
$$

### 11.3 Best-so-far tracking
If each “cycle” produces one cost value \(J^{(c)}\):

$$
J_{\text{best}} \leftarrow \min\left(J_{\text{best}},\,J^{(c)}\right)
$$

A typical improvement chart plots:
- the current cycle score \(J^{(c)}\)
- the best score so far \(J_\text{best}\)

---

## 12. Auto-tuning idea: Shadow Simulation Search

The auto mode treats tuning as an optimization problem:

$$
(K_p^\*,K_i^\*,K_d^\*)=\arg\min_{(K_p,K_i,K_d)} J(K_p,K_i,K_d)
$$

A simple loop:
1. Sample candidate gains \((K_p,K_i,K_d)\)
2. Run a short simulation with those gains
3. Compute cost \(J\)
4. Keep the best and repeat

Common search styles:
- random search
- hill climbing
- evolutionary strategies

---

## 13. Quick tuning intuition (practical)

- Increase \(K_p\): faster response, more overshoot risk  
- Increase \(K_d\): reduces oscillation, but can amplify noise  
- Increase \(K_i\): removes steady-state error (bias), but can cause windup  

A simple workflow:
1. Tune \(K_p\) until it responds quickly but starts oscillating  
2. Add \(K_d\) to damp oscillations  
3. Add a small \(K_i\) to remove steady drift / bias  

---
