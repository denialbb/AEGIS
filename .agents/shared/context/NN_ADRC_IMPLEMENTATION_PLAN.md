# NN-ADRC Integration Implementation Plan

This document details the comprehensive implementation plan for migrating the AEGIS architecture to a Neural Network-assisted Active Disturbance Rejection Controller (NN-ADRC).

## 1. Upgrading the State Estimator

The current Discrete-Time Kalman Filter tracks a 6-DOF state vector: \(x = [x, y, z, v_x, v_y, v_z]^T\). We will extend this to process noisy telemetry into clean position, velocity, and acceleration estimates, while also introducing the Extended State Observer (ESO) for disturbance estimation.

### 1.1 Extended Kalman Filter (EKF)
The EKF will fuse telemetry. The key prediction and update steps derived from the reference literature are:

**Prediction:**
\[ x_{t|t-1} = A x_{t-1|t-1} \]
\[ P_{t|t-1} = A P_{t-1|t-1} A^T + Q \]

**Update:**
\[ y_t = z_t - H x_{t|t-1} \]
\[ S_t = H P_{t|t-1} H^T + R_t \]
\[ K_t = P_{t|t-1} H^T S_t^{-1} \]
\[ x_{t|t} = x_{t|t-1} + K_t y_t \]
\[ P_{t|t} = (I - K_t H) P_{t|t-1} \]

### 1.2 Extended State Observer (ESO)
To estimate the total unknown disturbances (\(z_3\)), we implement the ESO.

**Equations:**
\[ \dot{z}_1 = z_2 - \beta_{01} \text{fal}(e, \alpha_1, \delta) \]
\[ \dot{z}_2 = z_3 + b_0 u - \beta_{02} \text{fal}(e, \alpha_1, \delta) \]
\[ \dot{z}_3 = -\beta_{03} \text{fal}(e, \alpha_2, \delta) \]
where \(e = z_1 - y\) is the estimation error.

**Implementation Snippet:**
```python
def fal(e: float, alpha: float, delta: float) -> float:
    """Non-linear smoothing function to mitigate chattering."""
    if abs(e) <= delta:
        return e / (delta ** (1 - alpha))
    else:
        return (abs(e) ** alpha) * np.sign(e)

def update_eso(z: np.ndarray, y: float, u: float, dt: float, 
               beta: tuple, alpha: tuple, delta: float, b0: float) -> np.ndarray:
    """Example ESO update step for a single axis."""
    beta01, beta02, beta03 = beta
    alpha1, alpha2 = alpha
    
    e = z[0] - y
    fe1 = fal(e, alpha1, delta)
    fe2 = fal(e, alpha2, delta)
    
    z_dot = np.array([
        z[1] - beta01 * fe1,
        z[2] + b0 * u - beta02 * fe1,
        -beta03 * fe2
    ])
    return z + z_dot * dt
```

## 2. Implementing the NN-ADRC Core

The control logic replaces the traditional guidance solver with an ADRC block augmented by a Neural Network.

### 2.1 ADRC Components
1. **Transient Profile Generator (TG):** Generates a smoothed desired trajectory (\(v_1\)) and its derivative (\(v_2\)).
2. **Weighted State Error Feedback (WSEF):**
   Calculates the baseline reference control based on the TG and ESO outputs:
   \[ e_1 = v_1 - z_1 \]
   \[ e_2 = v_2 - z_2 \]
   \[ u_0 = k_1 e_1 + k_2 e_2 \]
   \[ u = \frac{u_0 - z_3}{b_0} \]

### 2.2 Neural Network Compensator
The NN operates in parallel. Taking position, velocity, and acceleration as input, it predicts the compensatory acceleration vector:
\[ \Delta \ddot{\mathbf{r}} = [\Delta \ddot{x}, \Delta \ddot{y}, \Delta \ddot{z}, \Delta \ddot{\psi}, \Delta \ddot{\theta}, \Delta \ddot{\phi}]^T \]
*(Expanded to 6-DOF from the 2-axis gimbal equivalent \(\Delta \ddot{\mathbf{r}} = [\Delta \ddot{\psi}_a, \Delta \ddot{\theta}_m]^T\))*.

This delta is added to the baseline reference acceleration.

## 3. Attitude Tracking with Quaternions

Aggressive maneuvers (like hard aborts) risk gimbal lock. We use quaternions for attitude tracking instead of Euler angles.

**Error Quaternion Definition:**
\[ \delta q = q_c^{-1} \otimes q \]
Where \(q\) is the current attitude and \(q_c\) is the commanded attitude.

**Implementation Snippet:**
```python
def calculate_attitude_error(q_c: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Calculates attitude error quaternion dq = q_c_inv * q"""
    # Assuming quaternion_inverse and quaternion_multiply are available
    q_c_inv = quaternion_inverse(q_c)
    dq = quaternion_multiply(q_c_inv, q)
    return dq
```

Conversion to Euler angles (Roll, Pitch, Yaw) will be strictly maintained for logging and UI telemetry using standard `atan2` and `arcsin` transformations.

## 4. Control Allocator Integration

The Control Allocator maps the commanded Wrench vector to individual engine throttles.

**Wrench Vector:**
\[ W = [F_x, F_y, F_z, \tau_x, \tau_y, \tau_z]^T \]

**Allocation:**
The combined reference acceleration (WSEF + NN) is scaled by the vehicle's mass and inertia tensor to generate the target Wrench. The pseudo-inverse maps this to the remaining engines.

```python
# B_matrix is the 6xN engine configuration matrix (mapping thrusts to 6-DOF Wrench)
# W is the commanded 6x1 Wrench vector
# T contains the computed optimal engine throttles
B_pinv = np.linalg.pinv(B_matrix)
T = B_pinv @ W
```
*Note: We must constantly monitor the condition number of `B_matrix` to detect degenerate allocations (\(cond > 1e4\)).*

## 5. Adapting the FDI Module

Currently, the Fault Detection & Isolation (FDI) triggers if actual vs. expected acceleration deviates by \(> 0.5 \text{ m/s}^2\). 
With NN-ADRC, the neural network quickly masks this drop by immediately commanding surviving engines to compensate. 

**Updated FDI Logic:**
Instead of checking raw physical acceleration deviations, the FDI will now monitor the internal variables of the ADRC:
1. **ESO Disturbance Estimate (\(z_3\)):** A sudden large spike in \(z_3\) implies a discrete system failure.
2. **NN Compensatory Output (\(\Delta \ddot{\mathbf{r}}\)):** A large steady-state output indicates a persistent failure (e.g., engine loss), as the NN is continuously providing compensatory commands to counteract the missing engine.

## 6. Neural Network Training (Gremlin)

To generate data for the NN, we will script a background "Gremlin". This script will simulate asymmetric thrust failures during automated KSP descents by randomly forcing an engine's `thrustLimit` to 0. 

The dataset will map the real-time state \([x, v, a]\) to the difference between the ideal Computed Torque Model (CTM) and the failing plant, producing the exact target compensatory acceleration (\(\Delta \ddot{\mathbf{r}}\)) values required for backpropagation training.
