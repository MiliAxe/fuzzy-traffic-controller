import numpy as np

class FuzzyController:
    def __init__(self, x=None, Q_max=50.0, G_min=5.0, G_max=60.0):
        """
        Mamdani Fuzzy Logic Controller (FLC) for Adaptive Traffic Light Control.
        
        Args:
            x (list or np.ndarray): Flat vector of 15 parameters:
                                     - x[0:3]: Input queue membership bounds (Low, Medium, High)
                                     - x[3:6]: Output green duration membership bounds (Short, Medium, Long)
                                     - x[6:15]: Rule weights for the 9 IF-THEN rules
            Q_max (float): Maximum queue length boundary.
            G_min (float): Minimum green duration boundary.
            G_max (float): Maximum green duration boundary.
        """
        self.Q_max = Q_max
        self.G_min = G_min
        self.G_max = G_max
        
        # Rule Base Consequents
        # Rows represent Q1 (0: Low, 1: Medium, 2: High)
        # Cols represent Q2 (0: Low, 1: Medium, 2: High)
        # Values represent G1 Consequent (0: Short, 1: Medium, 2: Long)
        self.rule_consequents = np.array([
            [0, 0, 0],  # Q1=Low:  Q2=Low => Short, Q2=Medium => Short,  Q2=High => Short
            [1, 1, 0],  # Q1=Med:  Q2=Low => Medium, Q2=Medium => Medium, Q2=High => Short
            [2, 2, 1]   # Q1=High: Q2=Low => Long,   Q2=Medium => Long,   Q2=High => Medium
        ])
        
        if x is not None:
            self.set_parameters(x)
        else:
            # Default hand-tuned parameters (symmetric guess)
            default_p = np.array([12.0, 24.0, 38.0])
            default_q = np.array([15.0, 30.0, 45.0])
            default_w = np.ones(9)
            self.set_parameters(np.concatenate([default_p, default_q, default_w]))

    def set_parameters(self, x):
        """
        Maps a flat 15-dimensional vector to internal membership and rule parameters.
        Ensures sorting for membership ordering and clips parameters to physical ranges.
        """
        x = np.array(x, dtype=float)
        assert len(x) == 15, f"FuzzyController expects exactly 15 parameters, got {len(x)}"
        
        # 1. Input Membership parameters
        p = np.sort(x[0:3])
        p = np.clip(p, 0.0, self.Q_max)
        self.p1, self.p2, self.p3 = p[0], p[1], p[2]
        
        # 2. Output Membership parameters
        q = np.sort(x[3:6])
        q = np.clip(q, self.G_min, self.G_max)
        self.q1, self.q2, self.q3 = q[0], q[1], q[2]
        
        # 3. Rule weights
        self.rule_weights = np.clip(x[6:15], 0.0, 1.0)
        
        # 4. Precompute Output Membership Arrays for fast defuzzification
        self.num_points = 100
        self.z_vals = np.linspace(self.G_min, self.G_max, self.num_points)
        self.mu_short_arr = np.array([self._membership_low(z, self.q1, self.q2, self.G_max) for z in self.z_vals])
        self.mu_med_arr = np.array([self._membership_medium(z, self.q1, self.q2, self.q3) for z in self.z_vals])
        self.mu_long_arr = np.array([self._membership_high(z, self.q2, self.q3, self.G_max) for z in self.z_vals])

    def _membership_low(self, y, p1, p2, max_val):
        """Trapezoidal membership function for Low / Short."""
        if y <= p1:
            return 1.0
        elif y >= p2:
            return 0.0
        else:
            denom = p2 - p1
            return (p2 - y) / denom if denom > 1e-5 else 0.0

    def _membership_medium(self, y, p1, p2, p3):
        """Triangular membership function for Medium."""
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
        """Trapezoidal membership function for High / Long."""
        if y <= p2:
            return 0.0
        elif y >= p3:
            return 1.0
        else:
            denom = p3 - p2
            return (y - p2) / denom if denom > 1e-5 else 0.0

    def fuzzify(self, y, is_input=True):
        """Calculates membership degrees for [Low, Medium, High] or [Short, Medium, Long]."""
        if is_input:
            p1, p2, p3, max_val = self.p1, self.p2, self.p3, self.Q_max
        else:
            p1, p2, p3, max_val = self.q1, self.q2, self.q3, self.G_max
            
        mu_low = self._membership_low(y, p1, p2, max_val)
        mu_med = self._membership_medium(y, p1, p2, p3)
        mu_high = self._membership_high(y, p2, p3, max_val)
        
        return np.array([mu_low, mu_med, mu_high])

    def evaluate(self, Q1, Q2):
        """
        Computes the green light duration for Route 1 given current queue lengths.
        
        Args:
            Q1 (float): Queue length of Route 1.
            Q2 (float): Queue length of Route 2.
            
        Returns:
            green_time (float): Crisp green light duration (seconds).
        """
        Q1_clipped = np.clip(float(Q1), 0.0, self.Q_max)
        Q2_clipped = np.clip(float(Q2), 0.0, self.Q_max)
        
        # Fuzzify inputs
        mu_q1 = self.fuzzify(Q1_clipped, is_input=True)  # [Low, Medium, High]
        mu_q2 = self.fuzzify(Q2_clipped, is_input=True)  # [Low, Medium, High]
        
        # Inference Step: Mamdani Min-Max Composition
        rule_activations = np.zeros(9)
        rule_consequents_flat = self.rule_consequents.flatten()
        
        for idx_q1 in range(3):
            for idx_q2 in range(3):
                flat_idx = idx_q1 * 3 + idx_q2
                # AND condition -> Min composition
                strength = min(mu_q1[idx_q1], mu_q2[idx_q2])
                # Apply rule weight
                rule_activations[flat_idx] = strength * self.rule_weights[flat_idx]
                
        # Aggregate activations for output classes (0: Short, 1: Medium, 2: Long)
        set_activations = np.zeros(3)
        for i in range(3):
            matching = rule_activations[rule_consequents_flat == i]
            set_activations[i] = np.max(matching) if len(matching) > 0 else 0.0
            
        # Defuzzification: Center of Gravity (Centroid)
        # Aggregated fuzzy output membership function: max of min(activation, membership)
        mu_agg = np.maximum(
            np.minimum(set_activations[0], self.mu_short_arr),
            np.maximum(
                np.minimum(set_activations[1], self.mu_med_arr),
                np.minimum(set_activations[2], self.mu_long_arr)
            )
        )
            
        # Mathematical Centroid integration (discrete approximation)
        sum_mu = np.sum(mu_agg)
        if sum_mu > 1e-5:
            centroid = np.sum(self.z_vals * mu_agg) / sum_mu
        else:
            # Fallback to center of the output range if no rules fire
            centroid = (self.G_min + self.G_max) / 2.0
            
        return centroid
