import numpy as np

class PSOOptimizer:
    def __init__(self, cost_func, num_dimensions=15, num_particles=30, max_iter=50,
                 w=0.729, c1=1.494, c2=1.494, lb=None, ub=None):
        self.cost_func = cost_func
        self.num_dimensions = num_dimensions
        self.num_particles = num_particles
        self.max_iter = max_iter
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.lb = np.array(lb)
        self.ub = np.array(ub)
        
        self.X = np.random.uniform(self.lb, self.ub, (self.num_particles, self.num_dimensions))
        
        from fuzzy_controller import FuzzyController
        default_ctrl = FuzzyController()
        default_p = np.array([default_ctrl.p1, default_ctrl.p2, default_ctrl.p3])
        default_q = np.array([default_ctrl.q1, default_ctrl.q2, default_ctrl.q3])
        default_w = default_ctrl.rule_weights
        default_x = np.concatenate([default_p, default_q, default_w])
        self.X[0] = default_x
        
        for i in range(self.num_particles):
            self.X[i, 0:3] = np.sort(self.X[i, 0:3])
            self.X[i, 3:6] = np.sort(self.X[i, 3:6])
        
        self.v_max = 0.2 * (self.ub - self.lb)
        self.V = np.random.uniform(-self.v_max, self.v_max, (self.num_particles, self.num_dimensions))
        
        self.P = np.copy(self.X)
        self.p_best_costs = np.array([float('inf')] * self.num_particles)
        
        self.g_best = None
        self.g_best_cost = float('inf')
        
    def optimize(self, callback=None):
        convergence_curve = []
        
        for i in range(self.num_particles):
            cost = self.cost_func(self.X[i])
            self.p_best_costs[i] = cost
            if cost < self.g_best_cost:
                self.g_best_cost = cost
                self.g_best = np.copy(self.X[i])
                
        convergence_curve.append(self.g_best_cost)
        
        for it in range(self.max_iter):
            w_curr = self.w - (self.w - 0.4) * (it / self.max_iter)
            
            for i in range(self.num_particles):
                r1 = np.random.uniform(0.0, 1.0, self.num_dimensions)
                r2 = np.random.uniform(0.0, 1.0, self.num_dimensions)
                
                self.V[i] = (w_curr * self.V[i] 
                             + self.c1 * r1 * (self.P[i] - self.X[i]) 
                             + self.c2 * r2 * (self.g_best - self.X[i]))
                
                self.V[i] = np.clip(self.V[i], -self.v_max, self.v_max)
                self.X[i] = self.X[i] + self.V[i]
                self.X[i] = np.clip(self.X[i], self.lb, self.ub)
                
                self.X[i, 0:3] = np.sort(self.X[i, 0:3])
                self.X[i, 3:6] = np.sort(self.X[i, 3:6])
                
                cost = self.cost_func(self.X[i])
                
                if cost < self.p_best_costs[i]:
                    self.P[i] = np.copy(self.X[i])
                    self.p_best_costs[i] = cost
                    
                    if cost < self.g_best_cost:
                        self.g_best_cost = cost
                        self.g_best = np.copy(self.X[i])
                        
            convergence_curve.append(self.g_best_cost)
            
            if callback:
                callback(it, self.g_best_cost, self.g_best)
                
        return self.g_best, self.g_best_cost, convergence_curve
