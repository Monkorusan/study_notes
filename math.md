# Mathematical Analysis of Controller Classes

This document provides a comprehensive mathematical analysis of each method in the controller.py file, including the underlying equations, optimization formulations, and physical interpretations.

## Overview

The controller system implements a **multi-agent coverage control system** with **Control Barrier Functions (CBFs)** for safety constraints and **optimization-based control** for different operational modes. The system uses **convex quadratic programming** to solve real-time control problems.

---

## Base Controller Class

### `setup_optimization(self, u_nom=None)`

**Mathematical Formulation:**

This sets up the **Quadratic Programming (QP)** problem:

$$\min_{u,s} \frac{1}{2} \mathbf{x}^T \mathbf{P} \mathbf{x} + \mathbf{q}^T \mathbf{x}$$

where $\mathbf{x} = [u_x, u_y, s]^T$ (control inputs and slack variable)

**Matrices:**
- **P matrix:** $\mathbf{P} = \text{diag}(1, 1, w_{slack})$ where $w_{slack}$ is the slack weight
- **q vector:** $\mathbf{q} = [-u_{nom,x}, -u_{nom,y}, 0]^T$ if nominal input provided, else $\mathbf{q} = \mathbf{0}$

**Physical Interpretation:**
- Minimizes deviation from nominal control input
- Slack variable allows constraint relaxation with penalty
- **Requires knowing:** nominal control input $u_{nom}$, slack weight parameter

---

### `solve(self, P, q, G, h)`

**Mathematical Formulation:**

Solves the **cone quadratic program:**

$$\begin{align}
\min_{u,s} \quad & \frac{1}{2} \mathbf{x}^T \mathbf{P} \mathbf{x} + \mathbf{q}^T \mathbf{x} \\
\text{s.t.} \quad & \mathbf{G}\mathbf{x} + \mathbf{s} = \mathbf{h} \\
& \mathbf{s} \geq 0
\end{align}$$

Equivalently: $-\mathbf{G}\mathbf{x} + \mathbf{h} \geq 0$

**Physical Interpretation:**
- Finds optimal control input satisfying all safety constraints
- Uses **CVXOPT solver** for convex optimization
- **Requires knowing:** constraint matrices G, h from CBF formulations

---

### `cbf_limit_speed(self, G, h)`

**Mathematical Formulation:**

Implements **box constraints** on control inputs:

$$\begin{align}
u_x &\leq u_{max} \\
u_y &\leq u_{max} \\
-u_x &\leq u_{max} \\
-u_y &\leq u_{max}
\end{align}$$

**Matrix Form:**
$$\mathbf{G} = \begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
-1 & 0 & 0 \\
0 & -1 & 0
\end{bmatrix}, \quad \mathbf{h} = \begin{bmatrix}
u_{max} \\
u_{max} \\
u_{max} \\
u_{max}
\end{bmatrix}$$

**Physical Interpretation:**
- Enforces actuator saturation limits
- Ensures $\|u\|_\infty \leq u_{max}$
- **Requires knowing:** maximum control input $u_{max}$

---

### `cbf_avoid_collision(self, G, h)`

**Mathematical Formulation:**

Implements **Control Barrier Function** for collision avoidance:

For each neighboring agent $j$:
$$h_{ij}(\mathbf{p}) = \|\mathbf{p}_i - \mathbf{p}_j\|^2 - d_{safe}^2$$

**CBF Constraint:**
$$\dot{h}_{ij} + \alpha h_{ij} \geq 0$$

**Derivative Calculation:**
$$\dot{h}_{ij} = 2(\mathbf{p}_i - \mathbf{p}_j)^T(\dot{\mathbf{p}}_i - \dot{\mathbf{p}}_j) = 2(\mathbf{p}_i - \mathbf{p}_j)^T \mathbf{u}_i$$

(assuming $\dot{\mathbf{p}}_j = 0$ for neighboring agents)

**Linear Constraint:**
$$-2(\mathbf{p}_i - \mathbf{p}_j)^T \mathbf{u}_i \leq \alpha(\|\mathbf{p}_i - \mathbf{p}_j\|^2 - d_{safe}^2)$$

**Physical Interpretation:**
- Maintains minimum separation distance between agents
- Uses **exponential CBF** with convergence rate $\alpha$
- **Requires knowing:** agent positions, safety distance $d_{safe}$, CBF parameter $\alpha$

---

### `cbf_inside_field(self, G, h)`

**Mathematical Formulation:**

Implements **CBF constraints** for field boundaries:

For field with boundaries $[x_{min}, x_{max}] \times [y_{min}, y_{max}]$:

$$\begin{align}
h_1 &= x_{max} - x \geq 0 \quad \Rightarrow \quad \dot{h}_1 = -u_x \geq -\alpha_f(x_{max} - x) \\
h_2 &= x - x_{min} \geq 0 \quad \Rightarrow \quad \dot{h}_2 = u_x \geq -\alpha_f(x - x_{min}) \\
h_3 &= y_{max} - y \geq 0 \quad \Rightarrow \quad \dot{h}_3 = -u_y \geq -\alpha_f(y_{max} - y) \\
h_4 &= y - y_{min} \geq 0 \quad \Rightarrow \quad \dot{h}_4 = u_y \geq -\alpha_f(y - y_{min})
\end{align}$$

**Matrix Form:**
$$\mathbf{G} = \begin{bmatrix}
1 & 0 & 0 \\
-1 & 0 & 0 \\
0 & 1 & 0 \\
0 & -1 & 0
\end{bmatrix}, \quad \mathbf{h} = \alpha_f \begin{bmatrix}
x_{max} - x \\
x - x_{min} \\
y_{max} - y \\
y - y_{min}
\end{bmatrix}$$

**Physical Interpretation:**
- Keeps agent within field boundaries
- Uses **linear CBF** with field constraint parameter $\alpha_f$
- **Requires knowing:** current position $(x,y)$, field boundaries, CBF parameter $\alpha_f$

---

## StayController Class

### `optimize(self, u_nom=None)`

**Mathematical Formulation:**

Solves the complete QP problem:

$$\begin{align}
\min_{u,s} \quad & \frac{1}{2} \|u - u_{nom}\|^2 + w_{slack} s^2 \\
\text{s.t.} \quad & \text{Collision avoidance CBFs} \\
& \text{Speed limit constraints} \\
& \text{Field boundary CBFs}
\end{align}$$

**Physical Interpretation:**
- **Primary Goal:** Stay in place (minimize control effort)
- **Safety:** Avoid collisions with other agents
- **Constraints:** Respect speed limits and field boundaries
- **Calculation Process:** Combines all CBF constraints and solves QP
- **Requires knowing:** Agent positions, field boundaries, safety parameters

---

## TransitionController Class

### `get_u_towards_dest()`

**Mathematical Formulation:**

Implements **proportional feedback control**:

$$\mathbf{u}_{nom} = K_p (\mathbf{p}_{dest} - \mathbf{p}_{current})$$

where:
- $K_p$ is the feedback gain
- $\mathbf{p}_{dest}$ is the destination center
- $\mathbf{p}_{current}$ is the current agent position

**Physical Interpretation:**
- Simple proportional controller towards destination
- **Requires knowing:** destination position, current position, feedback gain $K_p$

---

### `cap_by_umax(self, u, keep_dir=False)`

**Mathematical Formulation:**

Two capping strategies:

**Direction-preserving capping:**
$$\mathbf{u}_{capped} = \begin{cases}
\mathbf{u} & \text{if } \|\mathbf{u}\| \leq u_{max} \\
\frac{u_{max}}{\|\mathbf{u}\|} \mathbf{u} & \text{if } \|\mathbf{u}\| > u_{max}
\end{cases}$$

**Component-wise capping:**
$$u_{capped,i} = \text{clip}(u_i, -u_{max}, u_{max})$$

**Physical Interpretation:**
- Direction-preserving maintains heading but limits speed
- Component-wise limits each axis independently
- **Requires knowing:** maximum speed $u_{max}$

---

### `optimize(self, u_nom=None)`

**Mathematical Formulation:**

Similar to StayController but **without field boundary constraints**:

$$\begin{align}
\min_{u,s} \quad & \frac{1}{2} \|u - u_{nom}\|^2 + w_{slack} s^2 \\
\text{s.t.} \quad & \text{Collision avoidance CBFs} \\
& \text{Speed limit constraints}
\end{align}$$

**Physical Interpretation:**
- **Primary Goal:** Follow nominal input towards destination
- **Safety:** Avoid collisions
- **No field constraints:** Allows transition between fields
- **Requires knowing:** Nominal input, collision avoidance parameters

---

## PersistentController Class

### `calc_u_nom()`

**Mathematical Formulation:**

Implements **Persistent Coverage Control** using:

$$\mathbf{u}_{nom} = 2\begin{bmatrix} u_x \\ u_y \end{bmatrix}$$

where:

$$u_x = -2m(p_x - c_{m,x}) - \frac{R^2 + B}{R} \int_{\partial S} (q_x - p_x) \phi(q) dq$$

$$u_y = -2m(p_y - c_{m,y}) - \frac{R^2 + B}{R} \int_{\partial S} (q_y - p_y) \phi(q) dq$$

**Terms Explained:**
- $m = \int_S \phi(q) dq$ : **mass** of importance in sight region
- $(c_{m,x}, c_{m,y})$ : **center of mass** of importance in sight
- $S$ : **sight region** (circle of radius $R$)
- $\partial S$ : **peripheral region** (annulus around sight boundary)
- $\phi(q)$ : **importance function** at position $q$
- $B$ : **tuning parameter**

**Center of Mass Calculation:**
$$c_{m,x} = \frac{1}{m} \int_S q_x \phi(q) dq, \quad c_{m,y} = \frac{1}{m} \int_S q_y \phi(q) dq$$

**Physical Interpretation:**
- **First term:** Move towards center of mass (exploration)
- **Second term:** Move away from peripheral regions (boundary repulsion)
- **Overall goal:** Maximize coverage of important regions
- **Requires knowing:** Current position, sight radius $R$, importance map $\phi$, parameter $B$

---

## AngleAwareController Class

### `calc_angle_aware_coeff()`

**Mathematical Formulation:**

Implements **Angle-Aware Coverage Control** with performance function:

$$p(\mathbf{p}, \boldsymbol{\zeta}) = e^{-\frac{\|\mathbf{p} - \boldsymbol{\zeta}\|^2}{2\sigma^2}}$$

**CBF Coefficient Calculation:**

$$\xi_1 = \begin{bmatrix} \xi_{1,x} \\ \xi_{1,y} \end{bmatrix}$$

where:

$$\xi_{1,x} = \sum_{grid} \left(-\frac{1}{\sigma^2} I_i(q) (p_x - \zeta_x(q))\right)$$

$$\xi_{1,y} = \sum_{grid} \left(-\frac{1}{\sigma^2} I_i(q) (p_y - \zeta_y(q))\right)$$

$$\xi_2 = -\alpha_{aa} \gamma + \sum_{grid} \left((-\delta_{dec}^2 p^2 + \alpha_{aa} \delta_{dec} p) \phi(q)\right) A$$

**Terms:**
- $I_i(q) = \delta_{dec} \cdot p(\mathbf{p}, \boldsymbol{\zeta}(q)) \cdot \phi(q) \cdot A$ : **instantaneous coverage rate**
- $\delta_{dec}$ : **coverage decay rate**
- $\gamma$ : **objective decay rate**
- $\alpha_{aa}$ : **angle-aware CBF parameter**
- $A$ : **grid unit area**

**CBF Constraint:**
$$-\boldsymbol{\xi}_1^T \mathbf{u} + \xi_2 \geq 0$$

**Physical Interpretation:**
- Ensures coverage rate exceeds decay rate
- Considers camera projection geometry through $\boldsymbol{\zeta}$
- Performance decreases with distance (Gaussian kernel)
- **Requires knowing:** Agent position, crop grid, performance parameters, projection mapping

---

### `optimize(self, u_nom=None)`

**Mathematical Formulation:**

$$\begin{align}
\min_{u,s} \quad & \frac{1}{2} \|u\|^2 + w_{slack} s^2 \\
\text{s.t.} \quad & -\boldsymbol{\xi}_1^T \mathbf{u} + s \geq \xi_2 \quad \text{(Angle-aware CBF)} \\
& \text{Speed limit constraints} \\
& \text{Collision avoidance CBFs}
\end{align}$$

**Physical Interpretation:**
- **Primary Goal:** Minimize control effort while maintaining coverage
- **Key Constraint:** Ensure coverage rate meets requirements
- **Safety:** Avoid collisions and respect speed limits
- **Requires knowing:** Angle-aware coefficients, collision parameters

---

## Summary of Calculation Processes

### Overall System Architecture
1. **Multi-agent system** with different operational modes
2. **CBF-based safety** ensures collision avoidance and boundary compliance
3. **Optimization-based control** balances objectives with constraints
4. **Real-time quadratic programming** for control computation

### Key Mathematical Concepts
- **Control Barrier Functions (CBFs):** Ensure safety constraints
- **Quadratic Programming:** Optimize control while respecting constraints
- **Persistent Coverage Control:** Exploration with importance-weighted regions
- **Angle-Aware Control:** Camera-based coverage with projection geometry
- **Proportional Control:** Simple feedback for transitions

### Required Knowledge for Each Controller
- **StayController:** Position, collision parameters, field boundaries
- **TransitionController:** Destination, feedback gains, collision parameters
- **PersistentController:** Importance map, sight radius, mass calculations
- **AngleAwareController:** Crop parameters, projection geometry, performance functions

This controller system elegantly combines **multi-objective optimization**, **safety-critical control**, and **coverage control theory** into a unified framework for autonomous multi-agent systems.