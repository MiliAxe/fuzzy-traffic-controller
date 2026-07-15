# Adaptive Traffic Light Controller (ATLC) Fuzzy System Optimization

This repository implements a production-grade, highly optimized **Adaptive Traffic Light Controller (ATLC)** for a city intersection. The system uses a **Mamdani Fuzzy Logic Controller (FLC)** whose parameters (membership function shapes and rule weights) are optimized using two distinct population-based metaheuristic algorithms: **Particle Swarm Optimization (PSO)** and **Ant Colony Optimization (ACO)**.

The core objective is to minimize a multi-objective cost function representing vehicle queue lengths, delays/wait times, and stops.

---

## 📁 Repository Directory Structure

```text
traffic_control_project/
│
├── docs/                      # Technical manual and site source files
│   └── index.md               # Overview, math foundations, and full analysis
├── mkdocs.yml                 # Modern Documentation Framework config
│
├── src/
│   ├── __init__.py
│   ├── main.py                # Click-based unified command-line entry point
│   ├── simulation.py          # Discrete-time traffic simulator engine
│   ├── fuzzy_controller.py    # Mamdani FLC Engine (Fuzzification to Centroid)
│   ├── pso_optimizer.py       # Particle Swarm Optimization implementation
│   └── aco_optimizer.py       # Ant Colony Optimization implementation
│
├── requirements.txt           # Explicitly pinned Python dependencies
└── README.md                  # Bootstrap and execution guide
```

---

## 🛠️ Installation & Setup

1. Ensure Python 3.8+ is installed on your system.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
   *(Note: On Arch Linux, you can also install python-numpy, python-matplotlib, and python-click via pacman for system-wide availability.)*

---

## 🚀 Execution & Command-Line Interface

All operations are unified under the `src/main.py` entry point. Run `python3 src/main.py --help` to see all options.

### 1. Run Baseline Simulation
Executes the simulation using default, hand-tuned membership functions and symmetric rule weights:
```bash
python3 src/main.py sim-baseline --duration 1200 --seed 42
```

### 2. Run Particle Swarm Optimization (PSO)
Optimizes the 15 FLC parameters using PSO, printing convergence progress and saving parameters to `pso_params.json`:
```bash
python3 src/main.py optimize-pso --particles 30 --iterations 50 --duration 1200 --seed 42
```

### 3. Run Ant Colony Optimization (ACO)
Optimizes the 15 FLC parameters using continuous discretized ACO, saving parameters to `aco_params.json`:
```bash
python3 src/main.py optimize-aco --ants 30 --iterations 50 --bins 30 --duration 1200 --seed 42
```

### 4. Compare Results and Generate Charts
Evaluates all three controllers (Baseline vs. PSO vs. ACO) on a test seed, outputs a comparison matrix table in the terminal, and exports high-fidelity visualization charts to `docs/assets/images/`:
```bash
python3 src/main.py compare --duration 1200 --seed 42
```

---

## 📊 Generated Visual Assets

When the `compare` command is run, it generates the following charts:
- `docs/assets/images/convergence.png`: Optimization cost curvature profiles.
- `docs/assets/images/fuzzy_boundaries.png`: Membership function shapes before and after optimization.
- `docs/assets/images/simulation_rollout.png`: Second-by-second traffic queue lengths ($Q_1, Q_2$) under baseline vs. optimized controllers.

---

## 📚 Technical Documentation Site

To build and view the local documentation site:
```bash
mkdocs serve
```
Then navigate to `http://127.0.0.1:8000` in your web browser.
