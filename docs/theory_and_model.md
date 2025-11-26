# Theory & Mathematical Model

This document summarizes the main modeling assumptions behind the 3D missile–aircraft pursuit simulation.

---

## 1. Coordinate System

- A right-handed **Cartesian** frame is used:  
  - \( x \): forward (initial aircraft direction)  
  - \( y \): lateral  
  - \( z \): vertical (up)

All positions and velocities are expressed in this inertial frame.

---

## 2. Aircraft Kinematics

The aircraft is modeled as a **point mass** moving at constant speed \( V_a \) with time-varying yaw and pitch angles:

- Yaw angle: \( \psi_a(t) \)  
- Pitch angle: \( \theta_a(t) \)

At each time step, the **direction unit vector** of the aircraft is:

\[
\begin{aligned}
u_{ax} &= \cos\theta_a \cos\psi_a \\
u_{ay} &= \cos\theta_a \sin\psi_a \\
u_{az} &= \sin\theta_a
\end{aligned}
\]

The aircraft position is integrated using simple forward Euler:

\[
\mathbf{p}_a(k) = \mathbf{p}_a(k-1) + V_a\, \mathbf{u}_a(k-1)\, \Delta t
\]

where:
- \( \mathbf{p}_a = [x_a, y_a, z_a]^T \)
- \( \Delta t \) is the simulation time step.

The code defines three phases:

1. **Straight & level** flight (\( t < t_1 \))  
2. **Mild weaving & pitch maneuvers** (\( t_1 \le t < t_2 \))  
3. **90° left turn** over a finite duration with a small pitch bump (\( t_2 \le t < t_2 + T_{\text{turn}} \))  
4. **Straight flight** on the new heading afterwards.

---

## 3. Missile Initial Conditions

The missile is also modeled as a point mass with constant speed \( V_m \):

- Initially placed **behind, offset, and below** the aircraft:
  \[
  \mathbf{p}_m(0) = [-4000,\ -1500,\ -800]^T \ (\text{meters})
  \]
- Initial direction is chosen to point from the missile position toward the aircraft’s initial position:
  \[
  \mathbf{u}_m(0) = \frac{\mathbf{p}_a(0) - \mathbf{p}_m(0)}{\|\mathbf{p}_a(0) - \mathbf{p}_m(0)\|}
  \]

Missile motion is integrated as:

\[
\mathbf{v}_m(k) = V_m\, \mathbf{u}_m(k), \quad
\mathbf{p}_m(k) = \mathbf{p}_m(k-1) + \mathbf{v}_m(k)\, \Delta t
\]

The missile **does not move** before the launch time \( t_{\text{launch}} \).

---

## 4. Toy IR Seeker Model

The seeker is modeled as a noisy, lagged measurement of the **line-of-sight (LOS) direction** from missile to aircraft.

1. **True LOS direction**
   \[
   \mathbf{r} = \mathbf{p}_a - \mathbf{p}_m, \quad
   \mathbf{u}_{LOS} = \frac{\mathbf{r}}{\|\mathbf{r}\|}
   \]

2. **Measurement noise**  
   A small random noise vector is added:
   \[
   \mathbf{m} = \mathbf{u}_{LOS} + \sigma \,\mathbf{n}, \quad \mathbf{n} \sim \mathcal{N}(0, I)
   \]
   then re-normalized.

3. **First-order lag**  
   The seeker direction is updated as:
   \[
   \mathbf{s}_{k} = \alpha\, \mathbf{s}_{k-1} + (1-\alpha)\, \mathbf{m}_{k}
   \]
   where \( 0 < \alpha < 1 \) controls the lag (values close to 1 mean more lag).

This captures **imperfect sensing**: the seeker does not instantly align with the true LOS and is affected by noise.

---

## 5. Missile Turn-Rate Limitation

Instead of directly setting the missile direction equal to the seeker direction, the missile has a **maximum turn rate** derived from a lateral acceleration limit:

- Maximum lateral acceleration:
  \[
  a_{n,\text{max}} = 25 g
  \]
- Missile speed: \( V_m \)
- Maximum turn rate:
  \[
  \dot{\theta}_{\text{max}} = \frac{a_{n,\text{max}}}{V_m} \quad [\text{rad/s}]
  \]
- Maximum heading change per time step:
  \[
  \Delta\theta_{\text{max}} = \dot{\theta}_{\text{max}} \Delta t
  \]

At each step, the missile direction unit vector \( \mathbf{u}_m \) is rotated **toward** the seeker direction \( \mathbf{s} \), but the angle of rotation is limited to \( \Delta\theta_{\text{max}} \). The rotation is performed using **Rodrigues’ rotation formula** around the axis:

\[
\mathbf{a} = \frac{\mathbf{u}_m \times \mathbf{s}}{\|\mathbf{u}_m \times \mathbf{s}\|}
\]

---

## 6. Intercept / Kill Radius

An intercept (“hit”) occurs if the separation distance drops below a predefined kill radius \( R_{\text{kill}} \):

\[
\|\mathbf{p}_a - \mathbf{p}_m\| < R_{\text{kill}}
\]

- If this condition is met, the simulation stops and the hit point is marked.
- If the simulation time ends without reaching \( R_{\text{kill}} \), the missile has effectively **missed**.

---

## 7. Numerical Method

- Time integration uses **fixed-time-step Euler**:
  - Simple and sufficient for this toy model.
  - Good for real-time animation and code clarity.
- The time step \( \Delta t = 0.01\ \text{s} \) is chosen as a compromise between resolution and runtime.

This model is not meant to represent any specific real system; it is a **conceptual demonstration** of constrained pursuit dynamics with an imperfect seeker in 3D.
