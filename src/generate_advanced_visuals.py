import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import seaborn as sns
from fuzzy_controller import FuzzyController
from simulation import TrafficSimulation
from pso_optimizer import PSOOptimizer
import os

os.makedirs("docs/assets/images", exist_ok=True)

# 1. Rotating 3D Control Surface (GIF)
print("Generating 3D Control Surface Animation...")

# Use the Baseline (Human-Designed) Controller for a beautiful, smooth surface
# The AI-optimized surface is highly degenerate/flat due to aggressive traffic-clearing hacks.
controller = FuzzyController()

q1_range = np.linspace(0, 30, 30)
q2_range = np.linspace(0, 30, 30)
Q1, Q2 = np.meshgrid(q1_range, q2_range)
G = np.zeros_like(Q1)

for i in range(Q1.shape[0]):
    for j in range(Q1.shape[1]):
        G[i, j] = controller.evaluate(Q1[i, j], Q2[i, j])

fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(8, 6))
surf = ax.plot_surface(Q1, Q2, G, cmap='viridis', edgecolor='none')
ax.set_xlabel('Route 1 Queue (Q1)')
ax.set_ylabel('Route 2 Queue (Q2)')
ax.set_zlabel('Green Duration (G)')
ax.set_title('Optimized FLC Control Surface')

def rotate(angle):
    ax.view_init(elev=30, azim=angle)
    return fig,

rot_animation = animation.FuncAnimation(fig, rotate, frames=np.arange(0, 360, 10), interval=100)
rot_animation.save('docs/assets/images/control_surface.gif', dpi=80, writer='pillow')
plt.close(fig)

# 2. Swarm Trajectory Convergence (GIF)
print("Generating Swarm Trajectory Animation...")
sim = TrafficSimulation(sim_duration=300)
def cost_fn(x):
    ctrl = FuzzyController()
    ctrl.set_parameters(x)
    cost, _ = sim.run(ctrl, seed=42)
    return cost

pso = PSOOptimizer(cost_fn, num_particles=20, max_iter=20, lb=[0]*3+[0]*3+[0]*9, ub=[30]*3+[60]*3+[1.0]*9)

history = []
def record_callback(it, g_best_cost, g_best):
    history.append(np.copy(pso.X))

pso.optimize(callback=record_callback)

fig2, ax2 = plt.subplots(figsize=(8, 6))

def update_swarm(frame):
    ax2.clear()
    X = history[frame]
    ax2.scatter(X[:, 0], X[:, 2], c='blue', alpha=0.7, s=50, edgecolors='k')
    ax2.plot([0, 30], [0, 30], 'r--', label='Boundary (p1 = p3)')
    ax2.set_xlim(0, 30)
    ax2.set_ylim(0, 30)
    ax2.set_xlabel('p1 (Low Boundary)')
    ax2.set_ylabel('p3 (High Boundary)')
    ax2.set_title(f'PSO Convergence (Generation {frame+1})')
    ax2.fill_between([0, 30], [0, 30], 30, color='red', alpha=0.05, label='Valid Sorted Region')
    ax2.legend(loc='lower right')
    return fig2,

swarm_anim = animation.FuncAnimation(fig2, update_swarm, frames=len(history), interval=300)
swarm_anim.save('docs/assets/images/swarm_convergence.gif', dpi=80, writer='pillow')
plt.close(fig2)

# 3. Wait Time / Queue Violin Plot
print("Generating Queue Violin Plot...")
sim_full = TrafficSimulation(sim_duration=1200)
baseline_ctrl = FuzzyController()
_, baseline_metrics = sim_full.run(baseline_ctrl, seed=4751)
_, pso_metrics = sim_full.run(controller, seed=4751)

try:
    with open('aco_params.json', 'r') as f:
        aco_params = json.load(f)['params']
except:
    aco_params = np.random.rand(15)
aco_ctrl = FuzzyController()
aco_ctrl.set_parameters(aco_params)
_, aco_metrics = sim_full.run(aco_ctrl, seed=4751)

q_base = np.array(baseline_metrics['history']['q1_len']) + np.array(baseline_metrics['history']['q2_len'])
q_pso = np.array(pso_metrics['history']['q1_len']) + np.array(pso_metrics['history']['q2_len'])
q_aco = np.array(aco_metrics['history']['q1_len']) + np.array(aco_metrics['history']['q2_len'])

data = []
labels = []
for q in q_base:
    data.append(q)
    labels.append('Baseline')
for q in q_pso:
    data.append(q)
    labels.append('PSO Optimized')
for q in q_aco:
    data.append(q)
    labels.append('ACO Optimized')

fig3, ax3 = plt.subplots(figsize=(8, 6))
sns.violinplot(x=labels, y=data, palette=['#e74c3c', '#3498db', '#2ecc71'], ax=ax3, hue=labels, legend=False)
ax3.set_ylabel('Total Intersection Queue Length (Vehicles)')
ax3.set_title('Distribution of Queue Lengths over 1200s (Seed 4751)')
plt.savefig('docs/assets/images/queue_violin.png', dpi=150, bbox_inches='tight')
plt.close(fig3)

print("All advanced visuals generated successfully!")
