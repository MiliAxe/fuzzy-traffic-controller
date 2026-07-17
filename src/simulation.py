import numpy as np

class TrafficSimulation:
    def __init__(self, lambda1=0.12, lambda2=0.08, discharge_rate=0.5, yellow_time=3,
                 G_min=5, G_max=60, sim_duration=1200, alpha=0.5, beta=0.3, gamma=0.001):
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.discharge_rate = discharge_rate
        self.yellow_time = yellow_time
        self.G_min = G_min
        self.G_max = G_max
        self.sim_duration = sim_duration
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

    def run(self, controller, seed=42):
        rng = np.random.default_rng(seed)
        
        q1 = []
        q2 = []
        
        history = {
            'time': [],
            'q1_len': [],
            'q2_len': [],
            'light_phase': [],
            'g1_val': [],
            'g2_val': []
        }
        
        all_vehicles = []
        
        current_time = 0
        credit1 = 0.0
        credit2 = 0.0
        vehicle_id_counter = 0
        
        while current_time < self.sim_duration:
            len_q1 = len(q1)
            len_q2 = len(q2)
            g1 = controller.evaluate(len_q1, len_q2)
            g1 = np.clip(g1, self.G_min, self.G_max)
            g1_int = int(round(g1))
            
            for _ in range(g1_int):
                if current_time >= self.sim_duration:
                    break
                
                arr1 = rng.poisson(self.lambda1)
                for _ in range(arr1):
                    vehicle = {'id': vehicle_id_counter, 'route': 1, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q1.append(vehicle)
                    vehicle_id_counter += 1
                
                arr2 = rng.poisson(self.lambda2)
                for _ in range(arr2):
                    vehicle = {'id': vehicle_id_counter, 'route': 2, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q2.append(vehicle)
                    vehicle_id_counter += 1
                
                credit1 += self.discharge_rate
                while credit1 >= 1.0 and len(q1) > 0:
                    veh = q1.pop(0)
                    veh['t_dep'] = current_time
                    credit1 -= 1.0
                
                credit2 = 0.0
                
                history['time'].append(current_time)
                history['q1_len'].append(len(q1))
                history['q2_len'].append(len(q2))
                history['light_phase'].append(0)
                history['g1_val'].append(g1)
                history['g2_val'].append(0.0)
                
                current_time += 1
                
            for _ in range(self.yellow_time):
                if current_time >= self.sim_duration:
                    break
                
                arr1 = rng.poisson(self.lambda1)
                for _ in range(arr1):
                    vehicle = {'id': vehicle_id_counter, 'route': 1, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q1.append(vehicle)
                    vehicle_id_counter += 1
                
                arr2 = rng.poisson(self.lambda2)
                for _ in range(arr2):
                    vehicle = {'id': vehicle_id_counter, 'route': 2, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q2.append(vehicle)
                    vehicle_id_counter += 1
                
                credit1 = 0.0
                credit2 = 0.0
                
                history['time'].append(current_time)
                history['q1_len'].append(len(q1))
                history['q2_len'].append(len(q2))
                history['light_phase'].append(1)
                history['g1_val'].append(0.0)
                history['g2_val'].append(0.0)
                
                current_time += 1
                
            len_q1 = len(q1)
            len_q2 = len(q2)
            g2 = controller.evaluate(len_q2, len_q1)
            g2 = np.clip(g2, self.G_min, self.G_max)
            g2_int = int(round(g2))
            
            for _ in range(g2_int):
                if current_time >= self.sim_duration:
                    break
                
                arr1 = rng.poisson(self.lambda1)
                for _ in range(arr1):
                    vehicle = {'id': vehicle_id_counter, 'route': 1, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q1.append(vehicle)
                    vehicle_id_counter += 1
                
                arr2 = rng.poisson(self.lambda2)
                for _ in range(arr2):
                    vehicle = {'id': vehicle_id_counter, 'route': 2, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q2.append(vehicle)
                    vehicle_id_counter += 1
                
                credit2 += self.discharge_rate
                while credit2 >= 1.0 and len(q2) > 0:
                    veh = q2.pop(0)
                    veh['t_dep'] = current_time
                    credit2 -= 1.0
                
                credit1 = 0.0
                
                history['time'].append(current_time)
                history['q1_len'].append(len(q1))
                history['q2_len'].append(len(q2))
                history['light_phase'].append(2)
                history['g1_val'].append(0.0)
                history['g2_val'].append(g2)
                
                current_time += 1
                
            for _ in range(self.yellow_time):
                if current_time >= self.sim_duration:
                    break
                
                arr1 = rng.poisson(self.lambda1)
                for _ in range(arr1):
                    vehicle = {'id': vehicle_id_counter, 'route': 1, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q1.append(vehicle)
                    vehicle_id_counter += 1
                
                arr2 = rng.poisson(self.lambda2)
                for _ in range(arr2):
                    vehicle = {'id': vehicle_id_counter, 'route': 2, 't_arr': current_time, 't_dep': None}
                    all_vehicles.append(vehicle)
                    q2.append(vehicle)
                    vehicle_id_counter += 1
                
                credit1 = 0.0
                credit2 = 0.0
                
                history['time'].append(current_time)
                history['q1_len'].append(len(q1))
                history['q2_len'].append(len(q2))
                history['light_phase'].append(3)
                history['g1_val'].append(0.0)
                history['g2_val'].append(0.0)
                
                current_time += 1

        wait_times = []
        stops_count = 0

        for veh in all_vehicles:
            t_arr = veh['t_arr']
            t_dep = veh['t_dep']
            if t_dep is not None:
                w = float(t_dep - t_arr)
            else:
                w = float(self.sim_duration - t_arr)

            wait_times.append(w)

            if w > 0.0:
                stops_count += 1

        num_vehicles = len(all_vehicles)
        avg_w = np.mean(wait_times) if num_vehicles > 0 else 0.0

        stop_count = stops_count
        avg_stops_rate = (stops_count / num_vehicles) if num_vehicles > 0 else 0.0

        if len(history['time']) > 0:
            avg_q = np.mean(np.array(history['q1_len']) + np.array(history['q2_len'])) / 2.0
        else:
            avg_q = 0.0

        cost = self.alpha * avg_w + self.beta * avg_q + self.gamma * stop_count

        metrics = {
            'avg_wait': avg_w,
            'avg_queue': avg_q,
            'avg_stops': avg_stops_rate,
            'stop_count': stop_count,
            'total_vehicles': num_vehicles,
            'history': history
        }

        return cost, metrics