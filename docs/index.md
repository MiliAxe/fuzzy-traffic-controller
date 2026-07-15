# Adaptive Traffic Light Controller (ATLC) Technical Manual

This manual details the architecture, mathematical formulation, and empirical performance of the Mamdani Fuzzy Logic Controller (FLC) optimized via Particle Swarm Optimization (PSO) and Ant Colony Optimization (ACO).

---

## 1. Mathematical Foundations & Formulations

The optimization is centered on a discrete-time intersection simulation. The target is to minimize a multi-objective cost function $C$, defined as:

$$C = \alpha W + \beta Q + \gamma S$$

Where:
* $W$ is the average waiting time per vehicle (in seconds).
* $Q$ is the average queue length across the intersection (number of vehicles).
* $S$ is the average number of stops per vehicle (dimensionless ratio $\in [0, 1]$).
* $\alpha, \beta, \gamma$ are the weighting coefficients. In this configuration, they are set to $\alpha = 0.5$, $\beta = 0.3$, and $\gamma = 0.2$.

### 1.1 Mamdani Fuzzy Inference System (FLC)

The intersection states are fuzzified from crisp queue lengths $Q_1, Q_2$ into three linguistic terms: **Low ($L$)**, **Medium ($M$)**, and **High ($H$)**. The output green-light duration $G$ is categorized into **Short ($S$)**, **Medium ($Med$)**, and **Long ($Lng$)**.

#### Fuzzification (Triangular and Trapezoidal MFs)
For parameters $0 \le p_1 \le p_2 \le p_3 \le Q_{\max}$:
* **Low**:
  $$\mu_{\text{Low}}(y) = \begin{cases} 
      1 & y \le p_1 \\
      \frac{p_2 - y}{p_2 - p_1} & p_1 < y < p_2 \\
      0 & y \ge p_2 
  \end{cases}$$
* **Medium**:
  $$\mu_{\text{Medium}}(y) = \begin{cases}
      0 & y \le p_1 \text{ or } y \ge p_3 \\
      \frac{y - p_1}{p_2 - p_1} & p_1 < y < p_2 \\
      1 & y = p_2 \\
      \frac{p_3 - y}{p_3 - p_2} & p_2 < y < p_3
  \end{cases}$$
* **High**:
  $$\mu_{\text{High}}(y) = \begin{cases}
      0 & y \le p_2 \\
      \frac{y - p_2}{p_3 - p_2} & p_2 < y < p_3 \\
      1 & y \ge p_3
  \end{cases}$$

#### Rule Activation & Min-Max Inference
The rules are of the form:
$$\text{IF } Q_1 \text{ is } A_i \text{ AND } Q_2 \text{ is } B_j \text{ THEN } G_1 \text{ is } C_k \text{ with weight } w_r$$

For each of the $r \in \{1, \dots, 9\}$ rules, the firing strength $\theta_r$ is:
$$\theta_r = w_r \cdot \min(\mu_{1, A_i}(Q_1), \mu_{2, B_j}(Q_2))$$

The aggregated output membership function $\mu_{agg}(z)$ for green time $z$ is:
$$\mu_{agg}(z) = \max_{C \in \{S, Med, Lng\}} \left[ \min\left( \max_{r: C_r = C} \theta_r, \mu_{\text{output}, C}(z) \right) \right]$$

#### Defuzzification (Centroid Method)
To extract the crisp green-light duration $z^*$, the Center of Gravity (Centroid) is calculated:
$$z^* = \frac{\int_{G_{\min}}^{G_{\max}} z \cdot \mu_{agg}(z) \, dz}{\int_{G_{\min}}^{G_{\max}} \mu_{agg}(z) \, dz}$$

---

### 1.2 Metaheuristic Optimization Algorithms

#### Particle Swarm Optimization (PSO)
In a 15-dimensional search space, each particle $i$ updates its velocity $v_i$ and position $x_i$ at step $t$ according to:
$$v_{id}^{t+1} = w v_{id}^t + c_1 r_1 (p_{id}^t - x_{id}^t) + c_2 r_2 (g_d^t - x_{id}^t)$$
$$x_{id}^{t+1} = x_{id}^t + v_{id}^{t+1}$$

Where:
* $w$ is the inertia weight, dynamically reduced from $0.729$ to $0.4$ across generations.
* $c_1, c_2$ are cognitive and social acceleration coefficients (set to $1.494$).
* $r_1, r_2 \sim U(0,1)$ are uniform random numbers.
* $p_i$ is the particle's personal best position, and $g$ is the swarm's global best position.

#### Ant Colony Optimization (ACO)
The continuous domain is discretized into $M = 30$ bins per dimension. Pheromones $\tau_{ij}$ are maintained for each dimension $i$ and bin $j$.
* **Selection Probability**:
  $$P_{ij} = \frac{\tau_{ij}^\alpha}{\sum_{k=1}^M \tau_{ik}^\alpha}$$
* **Stochastic Discretization**:
  $$x_i \sim U(c_{ij} - \frac{w_i}{2}, c_{ij} + \frac{w_i}{2})$$
  Where $c_{ij}$ is the center of bin $j$ for dimension $i$ and $w_i$ is the bin width.
* **Pheromone Evaporation and Update**:
  $$\tau_{ij} \leftarrow (1 - \rho)\tau_{ij} + \sum_{a=1}^{N_{ants}} \Delta \tau_{ij}^a$$
  $$\Delta \tau_{ij}^a = \begin{cases} \frac{10}{C_a} & \text{if ant } a \text{ chose bin } j \text{ in dimension } i \\ 0 & \text{otherwise} \end{cases}$$
  Where $\rho = 0.1$ is the evaporation rate, and $C_a$ is the evaluation cost for ant $a$.

---

## 2. Empirical Performance Evaluation

The optimization algorithms were evaluated using a population size of 30 over 50 generations with a simulation duration of 1200 seconds.

### 2.1 Comprehensive Performance Matrix

| Metric / Parameter | Baseline (Initial) | PSO Optimized | ACO Optimized | PSO Imp. | ACO Imp. |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Absolute Min Cost ($C$)** | 5.4224 | 3.6275 | 3.6644 | **+33.1%** | **+32.4%** |
| **Avg Vehicle Wait Time ($W$)** | 9.8937 s | 6.5197 s | 6.5906 s | **+34.1%** | **+33.4%** |
| **Avg Queue Length ($Q$)** | 1.0471 veh | 0.6900 veh | 0.6975 veh | **+34.1%** | **+33.4%** |
| **Avg Stops per Vehicle ($S$)** | 80.71% | 80.31% | 79.92% | **+0.5%** | **+1.0%** |
| **Wall-clock Convergence Velocity**| — | 0.798 sec/gen | 0.737 sec/gen | — | — |
| **Path Stability (Std Dev / Mean)**| — | Excellent | Excellent | — | — |

---

## 3. Visual Reports & Analysis

### 3.1 Optimization Convergence Profile
The convergence chart shows the reduction of the objective cost function $C$ across 50 generations for both metaheuristics. Both PSO and ACO converge steadily and robustly, reaching final costs of 3.6275 and 3.6644 respectively, outperforming the baseline (5.4224).

![Convergence Profiles](assets/images/convergence.png)

### 3.2 Fuzzy Membership Boundary Evolutions
This figure shows the initial, PSO-optimized, and ACO-optimized membership function shapes for the input queue length and output green time. Notice how the optimization shifts boundaries to fit the traffic demand.

![Fuzzy Membership Boundaries](assets/images/fuzzy_boundaries.png)

### 3.3 Intersection Queue State Rollouts
This time-series rollout shows the queue lengths on both directions over the 1200-second simulation. Shaded areas represent the green light phases for Route 1 (blue) and Route 2 (red).

![Simulation Rollouts](assets/images/simulation_rollout.png)
