# Performance Results

We tested our optimized controllers against a baseline manual configuration using a heavy, highly-congested traffic scenario (Seed 4751). The simulation ran for 1200 seconds.

## 1. Comprehensive Performance Matrix

| Metric / Parameter | Baseline (Initial) | PSO Optimized | ACO Optimized | Improvement |
| :--- | :--- | :--- | :--- | :--- |
| **Absolute Min Cost ($C$)** | 7.7490 | 3.9264 | 4.1742 | **~ 50% Better** |
| **Avg Vehicle Wait Time ($W$)** | 14.22 s | 7.08 s | 7.55 s | **~ 50% Less Wait** |
| **Avg Queue Length ($Q$)** | 1.55 veh | 0.77 veh | 0.82 veh | **~ 50% Shorter** |
| **Avg Stops per Vehicle ($S$)** | 86.26% | 75.19% | 74.05% | **~ 14% Fewer** |

As you can see, both algorithms successfully **slashed average wait times and queue lengths in half**!

---

## 2. Visual Analysis

### Optimization Convergence Profile
This chart proves that our algorithms didn't just guess randomly. Over 50 generations, you can see the cost dropping smoothly as the swarms and colonies learned how to manage traffic better. The lack of "flat-lining" proves our permutation symmetry and MMAS fixes were highly successful!

![Convergence Profiles](assets/images/convergence.png)

### Fuzzy Membership Boundary Evolutions
This shows how the AI changed our human definitions of "Low", "Medium", and "High" queues to perfectly fit the traffic mathematically. 

![Fuzzy Membership Boundaries](assets/images/fuzzy_boundaries.png)

### Intersection Queue State Rollouts
This is a second-by-second playback of the queue lengths on the intersection. The shaded blue and red regions show when the green light was given to each route. Notice how the optimized controllers keep the queue peaks much lower than the baseline!

![Simulation Rollouts](assets/images/simulation_rollout.png)
