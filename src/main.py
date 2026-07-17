import os
import json
import click
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from simulation import TrafficSimulation
from fuzzy_controller import FuzzyController
from pso_optimizer import PSOOptimizer
from aco_optimizer import ACOOptimizer

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS_IMG_DIR = os.path.join(BASE_DIR, 'docs', 'assets', 'images')
PSO_PARAMS_FILE = os.path.join(BASE_DIR, 'pso_params.json')
ACO_PARAMS_FILE = os.path.join(BASE_DIR, 'aco_params.json')

def ensure_directories():
    os.makedirs(DOCS_IMG_DIR, exist_ok=True)

@click.group()
def cli():
    ensure_directories()

@cli.command('sim-baseline')
@click.option('--duration', default=1200)
@click.option('--seed', default=42)
def sim_baseline(duration, seed):
    click.echo("Running baseline simulation with default hand-tuned fuzzy controller...")
    
    controller = FuzzyController()
    sim = TrafficSimulation(sim_duration=duration)
    cost, metrics = sim.run(controller, seed=seed)
    
    click.echo("\n--- BASELINE METRICS ---")
    click.echo(f"Simulation Duration : {duration} seconds")
    click.echo(f"Total Vehicles      : {metrics['total_vehicles']}")
    click.echo(f"Avg Wait Time (W)   : {metrics['avg_wait']:.4f} seconds")
    click.echo(f"Avg Queue Length (Q): {metrics['avg_queue']:.4f} vehicles")
    click.echo(f"Vehicle Stops (S)   : {metrics['stop_count']} stops "
               f"({metrics['avg_stops']*100:.2f}%)")
    click.echo(f"Total Evaluated Cost: {cost:.4f}")

@cli.command('optimize-pso')
@click.option('--particles', default=30)
@click.option('--iterations', default=50)
@click.option('--duration', default=1200)
@click.option('--seed', default=42)
def optimize_pso(particles, iterations, duration, seed):
    click.echo(f"Starting PSO Optimization...")
    
    sim = TrafficSimulation(sim_duration=duration)
    
    lb = np.array([0.0]*3 + [5.0]*3 + [0.0]*9)
    ub = np.array([50.0]*3 + [60.0]*3 + [1.0]*9)
    
    def cost_func(x):
        controller = FuzzyController(x)
        cost, _ = sim.run(controller, seed=seed)
        return cost
        
    optimizer = PSOOptimizer(
        cost_func=cost_func,
        num_dimensions=15,
        num_particles=particles,
        max_iter=iterations,
        lb=lb,
        ub=ub
    )
    
    def progress_callback(it, best_cost, best_pos):
        click.echo(f"Iteration {it+1}/{iterations} | Best Cost: {best_cost:.4f}")
        
    g_best, g_best_cost, conv_curve = optimizer.optimize(callback=progress_callback)
    
    pso_data = {
        'best_cost': float(g_best_cost),
        'params': g_best.tolist(),
        'convergence': [float(c) for c in conv_curve]
    }
    
    with open(PSO_PARAMS_FILE, 'w') as f:
        json.dump(pso_data, f, indent=4)
        
    click.echo(f"\nPSO Optimization Finished!")
    click.echo(f"Best Cost: {g_best_cost:.4f}")
    click.echo(f"Parameters saved to {PSO_PARAMS_FILE}")

@cli.command('optimize-aco')
@click.option('--ants', default=30)
@click.option('--iterations', default=50)
@click.option('--bins', default=30)
@click.option('--duration', default=1200)
@click.option('--seed', default=42)
def optimize_aco(ants, iterations, bins, duration, seed):
    click.echo(f"Starting ACO Optimization...")
    
    sim = TrafficSimulation(sim_duration=duration)
    
    lb = np.array([0.0]*3 + [5.0]*3 + [0.0]*9)
    ub = np.array([50.0]*3 + [60.0]*3 + [1.0]*9)
    
    def cost_func(x):
        controller = FuzzyController(x)
        cost, _ = sim.run(controller, seed=seed)
        return cost
        
    optimizer = ACOOptimizer(
        cost_func=cost_func,
        num_dimensions=15,
        num_ants=ants,
        max_iter=iterations,
        num_bins=bins,
        lb=lb,
        ub=ub
    )
    
    def progress_callback(it, best_cost, best_pos):
        click.echo(f"Iteration {it+1}/{iterations} | Best Cost: {best_cost:.4f}")
        
    g_best, g_best_cost, conv_curve = optimizer.optimize(callback=progress_callback)
    
    aco_data = {
        'best_cost': float(g_best_cost),
        'params': g_best.tolist(),
        'convergence': [float(c) for c in conv_curve]
    }
    
    with open(ACO_PARAMS_FILE, 'w') as f:
        json.dump(aco_data, f, indent=4)
        
    click.echo(f"\nACO Optimization Finished!")
    click.echo(f"Best Cost: {g_best_cost:.4f}")
    click.echo(f"Parameters saved to {ACO_PARAMS_FILE}")

@cli.command('compare')
@click.option('--duration', default=1200)
@click.option('--seed', default=42)
def compare(duration, seed):
    click.echo(f"Running comparative evaluation on seed {seed}...")
    
    if not os.path.exists(PSO_PARAMS_FILE):
        click.echo("PSO parameters file not found.")
        pso_params = None
        pso_conv = None
    else:
        with open(PSO_PARAMS_FILE, 'r') as f:
            data = json.load(f)
            pso_params = np.array(data['params'])
            pso_conv = data['convergence']
            
    if not os.path.exists(ACO_PARAMS_FILE):
        click.echo("ACO parameters file not found.")
        aco_params = None
        aco_conv = None
    else:
        with open(ACO_PARAMS_FILE, 'r') as f:
            data = json.load(f)
            aco_params = np.array(data['params'])
            aco_conv = data['convergence']
            
    ctrl_base = FuzzyController()
    ctrl_pso = FuzzyController(pso_params) if pso_params is not None else ctrl_base
    ctrl_aco = FuzzyController(aco_params) if aco_params is not None else ctrl_base
    
    sim = TrafficSimulation(sim_duration=duration)
    
    cost_base, metrics_base = sim.run(ctrl_base, seed=seed)
    cost_pso, metrics_pso = sim.run(ctrl_pso, seed=seed)
    cost_aco, metrics_aco = sim.run(ctrl_aco, seed=seed)
    
    headers = ["Metric", "Baseline", "PSO", "ACO", "PSO Imp.", "ACO Imp."]
    
    def get_row(name, val_b, val_p, val_a, is_pct=False, is_count=False):
        imp_p = (val_b - val_p) / val_b * 100.0 if val_b > 0.0 else 0.0
        imp_a = (val_b - val_a) / val_b * 100.0 if val_b > 0.0 else 0.0
        
        sign_p = "+" if imp_p >= 0 else ""
        sign_a = "+" if imp_a >= 0 else ""
        
        if is_pct:
            return [name, f"{val_b * 100.0:.2f}%", f"{val_p * 100.0:.2f}%", f"{val_a * 100.0:.2f}%", f"{sign_p}{imp_p:.1f}%", f"{sign_a}{imp_a:.1f}%"]
        elif is_count:
            return [name, f"{val_b:.0f}", f"{val_p:.0f}", f"{val_a:.0f}", f"{sign_p}{imp_p:.1f}%", f"{sign_a}{imp_a:.1f}%"]
        else:
            return [name, f"{val_b:.4f}", f"{val_p:.4f}", f"{val_a:.4f}", f"{sign_p}{imp_p:.1f}%", f"{sign_a}{imp_a:.1f}%"]
            
    rows = [
        get_row("Avg Wait W (s)", metrics_base['avg_wait'], metrics_pso['avg_wait'], metrics_aco['avg_wait']),
        get_row("Avg Queue Q", metrics_base['avg_queue'], metrics_pso['avg_queue'], metrics_aco['avg_queue']),
        get_row("Vehicle Stops S (count)", metrics_base['stop_count'], metrics_pso['stop_count'], metrics_aco['stop_count'], is_count=True),
        get_row("Stop Rate", metrics_base['avg_stops'], metrics_pso['avg_stops'], metrics_aco['avg_stops'], is_pct=True),
        get_row("Evaluation Cost C", cost_base, cost_pso, cost_aco)
    ]
    
    col_widths = [max(len(str(item)) for item in col) for col in zip(headers, *rows)]
    sep = "+" + "+".join("-" * (w + 2) for w in col_widths) + "+"
    
    click.echo("\n" + "="*33 + " RESULTS " + "="*33)
    click.echo(sep)
    click.echo("| " + " | ".join(f"{h:<{col_widths[i]}}" for i, h in enumerate(headers)) + " |")
    click.echo(sep)
    for row in rows:
        click.echo("| " + " | ".join(f"{item:<{col_widths[i]}}" for i, item in enumerate(row)) + " |")
    click.echo(sep)
    click.echo("="*75)
    
    ensure_directories()
    click.echo("\nGenerating visual assets...")
    
    if pso_conv is not None or aco_conv is not None:
        plt.figure(figsize=(10, 5))
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
        
        if pso_conv is not None:
            plt.plot(pso_conv, label='PSO Optimization', color='#1f77b4', linewidth=2.5, marker='o', markevery=5)
        if aco_conv is not None:
            plt.plot(aco_conv, label='ACO Optimization', color='#ff7f0e', linewidth=2.5, marker='s', markevery=5)
            
        plt.title('Optimization Convergence Profile (Cost C vs Generations)', fontsize=14, fontweight='bold', pad=15)
        plt.xlabel('Iteration / Generation', fontsize=12)
        plt.ylabel('Minimum Cost (C)', fontsize=12)
        plt.legend(frameon=True, facecolor='white', framealpha=0.9, fontsize=11)
        plt.tight_layout()
        
        conv_path = os.path.join(DOCS_IMG_DIR, 'convergence.png')
        plt.savefig(conv_path, dpi=300)
        plt.close()
        
    plt.figure(figsize=(15, 10))
    
    controllers = [
        ("Baseline", ctrl_base),
        ("PSO", ctrl_pso),
        ("ACO", ctrl_aco)
    ]
    
    q_vals = np.linspace(0, ctrl_base.Q_max, 200)
    g_vals = np.linspace(ctrl_base.G_min, ctrl_base.G_max, 200)
    
    for idx, (label, ctrl) in enumerate(controllers):
        plt.subplot(2, 3, idx + 1)
        mu_L = [ctrl._membership_low(y, ctrl.p1, ctrl.p2, ctrl.Q_max) for y in q_vals]
        mu_M = [ctrl._membership_medium(y, ctrl.p1, ctrl.p2, ctrl.p3) for y in q_vals]
        mu_H = [ctrl._membership_high(y, ctrl.p2, ctrl.p3, ctrl.Q_max) for y in q_vals]
        
        plt.plot(q_vals, mu_L, label='Low', color='#d62728', linewidth=2)
        plt.plot(q_vals, mu_M, label='Medium', color='#2ca02c', linewidth=2)
        plt.plot(q_vals, mu_H, label='High', color='#1f77b4', linewidth=2)
        plt.title(f"{label} - Queue MF", fontsize=11, fontweight='bold')
        plt.xlabel('Queue Length (vehicles)')
        plt.ylabel('Membership Degree')
        plt.ylim(-0.05, 1.05)
        plt.legend(loc='upper right')
        
        plt.subplot(2, 3, idx + 4)
        mu_S = [ctrl._membership_low(z, ctrl.q1, ctrl.q2, ctrl.G_max) for z in g_vals]
        mu_Med = [ctrl._membership_medium(z, ctrl.q1, ctrl.q2, ctrl.q3) for z in g_vals]
        mu_Long = [ctrl._membership_high(z, ctrl.q2, ctrl.q3, ctrl.G_max) for z in g_vals]
        
        plt.plot(g_vals, mu_S, label='Short', color='#9467bd', linewidth=2)
        plt.plot(g_vals, mu_Med, label='Medium', color='#bcbd22', linewidth=2)
        plt.plot(g_vals, mu_Long, label='Long', color='#17becf', linewidth=2)
        plt.title(f"{label} - Green Time MF", fontsize=11, fontweight='bold')
        plt.xlabel('Green Light Duration (sec)')
        plt.ylabel('Membership Degree')
        plt.ylim(-0.05, 1.05)
        plt.legend(loc='upper right')
        
    plt.suptitle("Fuzzy Logic Controller Membership Function Evolutions", fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    mf_path = os.path.join(DOCS_IMG_DIR, 'fuzzy_boundaries.png')
    plt.savefig(mf_path, dpi=300)
    plt.close()
    
    plt.figure(figsize=(12, 9))
    
    rollouts = [
        ("Baseline", metrics_base['history']),
        ("PSO Optimized", metrics_pso['history']),
        ("ACO Optimized", metrics_aco['history'])
    ]
    
    global_max_q = max(
        max(max(metrics_base['history']['q1_len']), max(metrics_base['history']['q2_len'])),
        max(max(metrics_pso['history']['q1_len']), max(metrics_pso['history']['q2_len'])),
        max(max(metrics_aco['history']['q1_len']), max(metrics_aco['history']['q2_len']))
    )
    y_limit = global_max_q + 2
    
    for idx, (label, hist) in enumerate(rollouts):
        plt.subplot(3, 1, idx + 1)
        t = hist['time']
        q1 = hist['q1_len']
        q2 = hist['q2_len']
        
        plt.plot(t, q1, label='Queue Route 1', color='#4a90e2', linewidth=1.5)
        plt.plot(t, q2, label='Queue Route 2', color='#e24a4a', linewidth=1.5)
        
        light_phase = hist['light_phase']
        phase_arr = np.array(light_phase)
        time_arr = np.array(t)
        
        plt.fill_between(time_arr, 0, y_limit + 5, where=(phase_arr == 0),
                         color='#4a90e2', alpha=0.08, label='Route 1 Green Phase')
        plt.fill_between(time_arr, 0, y_limit + 5, where=(phase_arr == 2),
                         color='#e24a4a', alpha=0.08, label='Route 2 Green Phase')
        
        plt.title(f"{label} - Queue State Rollout", fontsize=11, fontweight='bold')
        plt.xlabel('Simulation Time (seconds)')
        plt.ylabel('Queue Length (vehicles)')
        plt.xlim(0, duration)
        plt.ylim(0, y_limit)
        plt.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9, fontsize=9)
        plt.grid(True, linestyle='--', alpha=0.5)
        
    plt.suptitle("Intersection Simulation Queue Length Rollouts (Q1, Q2) Over Time", fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    rollout_path = os.path.join(DOCS_IMG_DIR, 'simulation_rollout.png')
    plt.savefig(rollout_path, dpi=300)
    plt.close()
    
    click.echo("\nVisual assets generated!")

if __name__ == '__main__':
    cli()
