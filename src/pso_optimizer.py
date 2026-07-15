import numpy as np

class PSOOptimizer:
    def __init__(self, cost_func, num_dimensions=15, num_particles=30, max_iter=50,
                 w=0.729, c1=1.494, c2=1.494, lb=None, ub=None):
        """
        Particle Swarm Optimization (PSO) for Continuous Parameter Optimization.
        
        Args:
            cost_func (callable): The multi-objective cost function to minimize.
            num_dimensions (int): Dimension of search space (15 for FLC parameters).
            num_particles (int): Number of particles in the swarm.
            max_iter (int): Maximum number of iterations.
            w (float): Initial structural inertia weight.
            c1 (float): Cognitive acceleration coefficient.
            c2 (float): Social acceleration coefficient.
            lb (np.ndarray): Lower bound array.
            ub (np.ndarray): Upper bound array.
        """
        self.cost_func = cost_func
        self.num_dimensions = num_dimensions
        self.num_particles = num_particles
        self.max_iter = max_iter
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.lb = np.array(lb)
        self.ub = np.array(ub)
        
        # Swarm Position Initialization: Uniformly distributed within boundaries
        self.X = np.random.uniform(self.lb, self.ub, (self.num_particles, self.num_dimensions))
        
        # Seed the first particle with the default baseline FLC parameters to guarantee optimization starts from a good state
        from fuzzy_controller import FuzzyController
        default_ctrl = FuzzyController()
        default_p = np.array([default_ctrl.p1, default_ctrl.p2, default_ctrl.p3])
        default_q = np.array([default_ctrl.q1, default_ctrl.q2, default_ctrl.q3])
        default_w = default_ctrl.rule_weights
        default_x = np.concatenate([default_p, default_q, default_w])
        self.X[0] = default_x
        
        # Sort membership parameters for all particles to keep them aligned
        for i in range(self.num_particles):
            self.X[i, 0:3] = np.sort(self.X[i, 0:3])
            self.X[i, 3:6] = np.sort(self.X[i, 3:6])
        
        # Velocity Initialization: Clamped to 20% of the dimension search range
        self.v_max = 0.2 * (self.ub - self.lb)
        self.V = np.random.uniform(-self.v_max, self.v_max, (self.num_particles, self.num_dimensions))
        
        # Personal best tracking
        self.P = np.copy(self.X)
        self.p_best_costs = np.array([float('inf')] * self.num_particles)
        
        # Global best tracking
        self.g_best = None
        self.g_best_cost = float('inf')
        
    def optimize(self, callback=None):
        """
        Runs the PSO optimization routine.
        
        Args:
            callback (callable): A progress callback function (iteration, g_best_cost, g_best).
            
        Returns:
            g_best (np.ndarray): The optimized parameter array.
            g_best_cost (float): The minimum cost reached.
            convergence_curve (list): Cost profile across generations.
        """
        convergence_curve = []
        
        # Initial evaluation of the population
        for i in range(self.num_particles):
            cost = self.cost_func(self.X[i])
            self.p_best_costs[i] = cost
            if cost < self.g_best_cost:
                self.g_best_cost = cost
                self.g_best = np.copy(self.X[i])
                
        convergence_curve.append(self.g_best_cost)
        
        for it in range(self.max_iter):
            # Dynamic inertia weight reduction for better fine-tuning at later stages
            w_curr = self.w - (self.w - 0.4) * (it / self.max_iter)
            
            for i in range(self.num_particles):
                # Stochastic vectors
                r1 = np.random.uniform(0.0, 1.0, self.num_dimensions)
                r2 = np.random.uniform(0.0, 1.0, self.num_dimensions)
                
                # Velocity update formula
                self.V[i] = (w_curr * self.V[i] 
                             + self.c1 * r1 * (self.P[i] - self.X[i]) 
                             + self.c2 * r2 * (self.g_best - self.X[i]))
                
                # Boundary clamping on velocity
                self.V[i] = np.clip(self.V[i], -self.v_max, self.v_max)
                
                # Position update formula
                self.X[i] = self.X[i] + self.V[i]
                
                # Boundary clamping on position
                self.X[i] = np.clip(self.X[i], self.lb, self.ub)
                
                # Sort coordinates to prevent permutation collapse
                self.X[i, 0:3] = np.sort(self.X[i, 0:3])
                self.X[i, 3:6] = np.sort(self.X[i, 3:6])
                
                # Evaluation
                cost = self.cost_func(self.X[i])
                
                # Update personal best
                if cost < self.p_best_costs[i]:
                    self.P[i] = np.copy(self.X[i])
                    self.p_best_costs[i] = cost
                    
                    # Update global best
                    if cost < self.g_best_cost:
                        self.g_best_cost = cost
                        self.g_best = np.copy(self.X[i])
                        
            convergence_curve.append(self.g_best_cost)
            
            if callback:
                callback(it, self.g_best_cost, self.g_best)
                
        return self.g_best, self.g_best_cost, convergence_curve
