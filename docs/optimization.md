# optimization algorithms

to make the fuzzy logic controller run perfectly, we had to tweak 15 different internal parameters. these numbers decide the shapes of the fuzzy boundaries and how important each rule is.

i ended up testing two different population-based metaheuristic algorithms. they sound fancy, but they're basically inspired by nature and use a "population" of agents working together to find the best setup.

## 1. particle swarm optimization (pso)
this one is inspired by a flock of birds. pso makes a swarm of 30 "particles." each particle is basically a guess for how the traffic light should be set up.

* the particles fly around trying out different math combinations.
* they remember their own personal best score.
* they also talk to the rest of the swarm to see the global best score.
* over 50 rounds (generations), they tweak their flight paths until they hone in on the absolute perfect setup.

## 2. ant colony optimization (aco)
this one is inspired by how ants find the shortest path to food. aco uses 30 "ants" to build the setup step-by-step.

* i chopped up the search space into a grid (30 bins for each parameter).
* when an ant finds a good setup (meaning less traffic), it leaves "pheromones" on those bins.
* the next ants are more likely to pick bins with lots of pheromones.
* over time, the bad choices evaporate, and the whole colony agrees on the best setup.

---

## a cool technical fix we did: breaking permutation symmetry
while messing around with this, i ran into a weird math problem: the fuzzy parameters have permutation symmetry. this means swapping the boundaries for "low" and "medium" gives the exact same result. if you just leave it like that, it totally confuses the ants and birds! they end up leaving conflicting pheromones or just getting stuck.

we fixed this by forcing the math coordinates into a sorted wedge ($x_0 \le x_1 \le x_2$). it's a pretty simple fix but it made both algorithms run way faster and smoother, and completely stopped them from getting stuck early on.

**watch the ai learn!** here's an animation of the pso swarm over 20 generations. you can see the particles (blue dots) quickly trapping themselves in the valid sorted area (the red wedge) and converging on the best answer.

![swarm convergence](assets/images/swarm_convergence.gif)
