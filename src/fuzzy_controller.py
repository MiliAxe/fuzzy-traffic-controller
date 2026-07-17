import numpy as np

class FuzzyController:
    def __init__(self, x=None, Q_max=50.0, G_min=5.0, G_max=60.0):
        self.Q_max = Q_max
        self.G_min = G_min
        self.G_max = G_max
        
        self.rule_consequents = np.array([
            [0, 0, 0],
            [1, 1, 0],
            [2, 2, 1]
        ])
        
        if x is not None:
            self.set_parameters(x)
        else:
            default_p = np.array([12.0, 24.0, 38.0])
            default_q = np.array([15.0, 30.0, 45.0])
            default_w = np.ones(9)
            self.set_parameters(np.concatenate([default_p, default_q, default_w]))

    def set_parameters(self, x):
        x = np.array(x, dtype=float)
        assert len(x) == 15
        
        p = np.sort(x[0:3])
        p = np.clip(p, 0.0, self.Q_max)
        self.p1, self.p2, self.p3 = p[0], p[1], p[2]
        
        q = np.sort(x[3:6])
        q = np.clip(q, self.G_min, self.G_max)
        self.q1, self.q2, self.q3 = q[0], q[1], q[2]
        
        self.rule_weights = np.clip(x[6:15], 0.0, 1.0)
        
        self.num_points = 100
        self.z_vals = np.linspace(self.G_min, self.G_max, self.num_points)
        self.mu_short_arr = np.array([self._membership_low(z, self.q1, self.q2, self.G_max) for z in self.z_vals])
        self.mu_med_arr = np.array([self._membership_medium(z, self.q1, self.q2, self.q3) for z in self.z_vals])
        self.mu_long_arr = np.array([self._membership_high(z, self.q2, self.q3, self.G_max) for z in self.z_vals])

    def _membership_low(self, y, p1, p2, max_val):
        if y <= p1:
            return 1.0
        elif y >= p2:
            return 0.0
        else:
            denom = p2 - p1
            return (p2 - y) / denom if denom > 1e-5 else 0.0

    def _membership_medium(self, y, p1, p2, p3):
        if y <= p1 or y >= p3:
            return 0.0
        elif y == p2:
            return 1.0
        elif p1 < y < p2:
            denom = p2 - p1
            return (y - p1) / denom if denom > 1e-5 else 0.0
        else:
            denom = p3 - p2
            return (p3 - y) / denom if denom > 1e-5 else 0.0

    def _membership_high(self, y, p2, p3, max_val):
        if y <= p2:
            return 0.0
        elif y >= p3:
            return 1.0
        else:
            denom = p3 - p2
            return (y - p2) / denom if denom > 1e-5 else 0.0

    def fuzzify(self, y, is_input=True):
        if is_input:
            p1, p2, p3, max_val = self.p1, self.p2, self.p3, self.Q_max
        else:
            p1, p2, p3, max_val = self.q1, self.q2, self.q3, self.G_max
            
        mu_low = self._membership_low(y, p1, p2, max_val)
        mu_med = self._membership_medium(y, p1, p2, p3)
        mu_high = self._membership_high(y, p2, p3, max_val)
        
        return np.array([mu_low, mu_med, mu_high])

    def evaluate(self, Q1, Q2):
        Q1_clipped = np.clip(float(Q1), 0.0, self.Q_max)
        Q2_clipped = np.clip(float(Q2), 0.0, self.Q_max)
        
        mu_q1 = self.fuzzify(Q1_clipped, is_input=True)
        mu_q2 = self.fuzzify(Q2_clipped, is_input=True)
        
        rule_activations = np.zeros(9)
        rule_consequents_flat = self.rule_consequents.flatten()
        
        for idx_q1 in range(3):
            for idx_q2 in range(3):
                flat_idx = idx_q1 * 3 + idx_q2
                strength = min(mu_q1[idx_q1], mu_q2[idx_q2])
                rule_activations[flat_idx] = strength * self.rule_weights[flat_idx]
                
        set_activations = np.zeros(3)
        for i in range(3):
            matching = rule_activations[rule_consequents_flat == i]
            set_activations[i] = np.max(matching) if len(matching) > 0 else 0.0
            
        mu_agg = np.maximum(
            np.minimum(set_activations[0], self.mu_short_arr),
            np.maximum(
                np.minimum(set_activations[1], self.mu_med_arr),
                np.minimum(set_activations[2], self.mu_long_arr)
            )
        )
            
        sum_mu = np.sum(mu_agg)
        if sum_mu > 1e-5:
            centroid = np.sum(self.z_vals * mu_agg) / sum_mu
        else:
            centroid = (self.G_min + self.G_max) / 2.0
            
        return centroid
