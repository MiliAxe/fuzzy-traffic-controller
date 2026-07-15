import numpy as np

class ACOOptimizer:
    def __init__(self, cost_func, num_dimensions=15, num_ants=30, max_iter=50,
                 rho=0.1, alpha=1.0, beta=0.0, num_bins=30, lb=None, ub=None):
        """
        Ant Colony Optimization (ACO) for continuous domains using a discretized grid.
        
        Args:
            cost_func (callable): The multi-objective cost function to minimize.
            num_dimensions (int): Dimension of search space (15 for FLC parameters).
            num_ants (int): Number of ants in the colony.
            max_iter (int): Maximum number of iterations.
            rho (float): Pheromone evaporation rate.
            alpha (float): Pheromone exponent.
            beta (float): Heuristic exponent (unused in standard real-domain discretized ACO).
            num_bins (int): Number of discretization bins per dimension.
            lb (np.ndarray): Lower bound array.
            ub (np.ndarray): Upper bound array.
        """
        self.cost_func = cost_func
        self.num_dimensions = num_dimensions
        self.num_ants = num_ants
        self.max_iter = max_iter
        self.rho = rho
        self.alpha = alpha
        self.beta = beta
        self.num_bins = num_bins
        self.lb = np.array(lb)
        self.ub = np.array(ub)
        
        # Discretize boundaries into bins for each dimension
        self.bin_widths = (self.ub - self.lb) / self.num_bins
        self.bin_centers = np.zeros((self.num_dimensions, self.num_bins))
        for d in range(self.num_dimensions):
            self.bin_centers[d] = self.lb[d] + (np.arange(self.num_bins) + 0.5) * self.bin_widths[d]
            
        # Pheromone Matrix: tau[dimension_index, bin_index]
        self.tau = np.ones((self.num_dimensions, self.num_bins))
        
        # Global best tracking
        self.g_best = None
        self.g_best_cost = float('inf')
        
    def optimize(self, callback=None):
        """
        Runs the ACO optimization routine.
        
        Args:
            callback (callable): A progress callback function (iteration, g_best_cost, g_best).
            
        Returns:
            g_best (np.ndarray): The optimized parameter array.
            g_best_cost (float): The minimum cost reached.
            convergence_curve (list): Cost profile across generations.
        """
        convergence_curve = []
        
        for it in range(self.max_iter):
            ant_positions = np.zeros((self.num_ants, self.num_dimensions))
            ant_bin_choices = np.zeros((self.num_ants, self.num_dimensions), dtype=int)
            ant_costs = np.zeros(self.num_ants)
            
            for a in range(self.num_ants):
                if a == 0:
                    # Seed the first ant with the default baseline FLC parameters
                    from fuzzy_controller import FuzzyController
                    default_ctrl = FuzzyController()
                    default_p = np.array([default_ctrl.p1, default_ctrl.p2, default_ctrl.p3])
                    default_q = np.array([default_ctrl.q1, default_ctrl.q2, default_ctrl.q3])
                    default_w = default_ctrl.rule_weights
                    default_x = np.concatenate([default_p, default_q, default_w])
                    ant_positions[a] = default_x
                    
                    # Compute nearest bins for the baseline parameters
                    for d in range(self.num_dimensions):
                        diffs = np.abs(self.bin_centers[d] - default_x[d])
                        ant_bin_choices[a, d] = np.argmin(diffs)
                else:
                    for d in range(self.num_dimensions):
                        # Selection probability calculation (weighted by pheromones)
                        p_tau = self.tau[d] ** self.alpha
                        sum_p = np.sum(p_tau)
                        probs = p_tau / sum_p if sum_p > 0.0 else np.ones(self.num_bins) / self.num_bins
                        
                        # Choose a bin index
                        bin_choice = np.random.choice(self.num_bins, p=probs)
                        ant_bin_choices[a, d] = bin_choice
                        
                        # Continuous sampling inside the chosen bin (stochastic discretization)
                        center = self.bin_centers[d, bin_choice]
                        width = self.bin_widths[d]
                        val = np.random.uniform(center - width/2.0, center + width/2.0)
                        
                        # Clip positions to search bounds
                        ant_positions[a, d] = np.clip(val, self.lb[d], self.ub[d])
                        
                if a > 0:
                    # Enforce sorted coordinates to prevent permutation symmetry and contradictory pheromones
                    ant_positions[a, 0:3] = np.sort(ant_positions[a, 0:3])
                    ant_positions[a, 3:6] = np.sort(ant_positions[a, 3:6])
                    
                    # Recompute bin choices for the sorted dimensions so pheromones are deposited correctly
                    for d in range(6):
                        diffs = np.abs(self.bin_centers[d] - ant_positions[a, d])
                        ant_bin_choices[a, d] = np.argmin(diffs)
                    
                # Evaluate the solution constructed by the ant
                ant_costs[a] = self.cost_func(ant_positions[a])
                
                # Update global best
                if ant_costs[a] < self.g_best_cost:
                    self.g_best_cost = ant_costs[a]
                    self.g_best = np.copy(ant_positions[a])
                    
            convergence_curve.append(self.g_best_cost)
            
            # Pheromone Evaporation Step
            self.tau = (1.0 - self.rho) * self.tau
            
            # Pheromone Deposit Step: Reward only the top 3 best ants of the generation (Elitist ACO)
            sorted_indices = np.argsort(ant_costs)
            for rank in range(min(3, self.num_ants)):
                idx = sorted_indices[rank]
                cost = ant_costs[idx]
                # Rank 0 gets multiplier 3, rank 1 gets multiplier 2, rank 2 gets multiplier 1
                weight = 3 - rank
                deposit = (10.0 / (cost + 1e-5)) * weight
                for d in range(self.num_dimensions):
                    bin_choice = ant_bin_choices[idx, d]
                    self.tau[d, bin_choice] += deposit
                    
            # MMAS Pheromone Limits: Clip pheromones to keep exploration active and prevent stagnation
            self.tau = np.clip(self.tau, 0.1, 5.0)
            
            if callback:
                callback(it, self.g_best_cost, self.g_best)
                
        return self.g_best, self.g_best_cost, convergence_curve
