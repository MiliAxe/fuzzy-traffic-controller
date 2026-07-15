# Adaptive Traffic Light Controller (ATLC)
Welcome to the ATLC project! 

This project aims to solve a very common problem: **traffic jams at intersections.** 
Traditional traffic lights use fixed timers. No matter if a road is completely empty or packed with hundreds of cars, the green light stays on for the same amount of time. This is incredibly inefficient.

Instead, we built a **Smart Traffic Light System** that looks at the lengths of the queues and adapts the green light duration in real-time. 

To achieve this, we use a **Fuzzy Logic Controller**, which mimics human decision-making (e.g., "If the queue is very long, give it a long green light"). To make this controller as perfect as possible, we train it using two AI optimization algorithms: **Particle Swarm Optimization (PSO)** and **Ant Colony Optimization (ACO)**.

In this documentation, you will find:
1. **How It Works:** A simple explanation of our Fuzzy Logic system.
2. **Optimization:** How we used PSO and ACO to train our system.
3. **Performance Results:** The final showcase proving our system cuts wait times by 50%.
4. **Code Reference:** Auto-generated documentation for developers.
