"""Phase 6 の図（最適化収束曲線・最適応答比較・パレート図）を生成する
入力: data/results/phase6_convergence_ga.csv (列: generation, best_cost)
      data/results/phase6_convergence_fminsearch.csv (列: iteration, fval)
      data/results/phase6_candidates.csv (label,Kp_pos,Kp_vel,Ki_vel,cost)
      data/results/phase6_top3_responses.csv (列: t, theta_1, theta_2, theta_3)
      data/results/phase6_pareto_data.csv (列: settling_time, overshoot_pct)
出力: figures/phase6_optimization_convergence.png
      figures/phase6_optimal_response.png
      figures/phase6_pareto.png
作成日: 2026-10-03
"""
import csv
import math
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
import matplotlib.pyplot as plt

def read_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.reader(f):
            rows.append(row)
    return rows

def to_float(x):
    try:
        return float(x)
    except ValueError:
        return math.nan

# --- 図1: 最適化収束曲線 ---
ga_rows = read_csv("data/results/phase6_convergence_ga.csv")
fmin_rows = read_csv("data/results/phase6_convergence_fminsearch.csv")
ga_gen = [to_float(r[0]) for r in ga_rows]
ga_cost = [to_float(r[1]) for r in ga_rows]
fmin_iter = [to_float(r[0]) for r in fmin_rows]
fmin_cost = [to_float(r[1]) for r in fmin_rows]

fig1, ax = plt.subplots(figsize=(8, 5))
ax.plot(ga_gen, ga_cost, '-', label='GA (世代)')
ax.plot(fmin_iter, fmin_cost, '--', label='fminsearch (反復)')
ax.set_xlabel('世代数 / 反復数')
ax.set_ylabel('コスト')
ax.set_title('Phase 6: 最適化収束曲線')
ax.legend()
ax.grid(True)
fig1.tight_layout()
fig1.savefig('figures/phase6_optimization_convergence.png', dpi=150)
plt.close(fig1)

# --- 図2: 上位3候補でのPTP応答比較 ---
cand_rows = read_csv("data/results/phase6_candidates.csv")[1:]
resp_rows = read_csv("data/results/phase6_top3_responses.csv")
t = [to_float(r[0]) for r in resp_rows]
n_series = len(resp_rows[0]) - 1

fig2, ax = plt.subplots(figsize=(9, 5))
for i in range(n_series):
    theta_i = [to_float(r[i+1]) for r in resp_rows]
    label = cand_rows[i][0] if i < len(cand_rows) else f'候補{i+1}'
    gains_str = f"Kp_pos={to_float(cand_rows[i][1]):.1f}, Kp_vel={to_float(cand_rows[i][2]):.2f}, Ki_vel={to_float(cand_rows[i][3]):.1f}" if i < len(cand_rows) else ""
    ax.plot(t, theta_i, label=f'{label}\n({gains_str})')
ax.axhline(0.7854, color='gray', linestyle=':', linewidth=1, label='目標(45deg)')
ax.set_xlabel('時間 [s]')
ax.set_ylabel('theta_a [rad]')
ax.set_title('Phase 6: 最適ゲイン候補でのPTP応答比較（上位3候補）')
ax.legend(fontsize=7, loc='lower right')
ax.grid(True)
fig2.tight_layout()
fig2.savefig('figures/phase6_optimal_response.png', dpi=150)
plt.close(fig2)

# --- 図3: パレート図（整定時間 vs オーバーシュート） ---
pareto_rows = read_csv("data/results/phase6_pareto_data.csv")
settling = [to_float(r[0]) for r in pareto_rows]
overshoot = [to_float(r[1]) for r in pareto_rows]
cand_labels = [r[0] for r in cand_rows]

fig3, ax = plt.subplots(figsize=(7, 6))
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
for i, (s, o, lbl) in enumerate(zip(settling, overshoot, cand_labels)):
    ax.scatter(s, o, s=80, color=colors[i % len(colors)], label=lbl, zorder=3)
ax.set_xlabel('整定時間 [s]')
ax.set_ylabel('オーバーシュート [%]')
ax.set_title('Phase 6: パレート図（整定時間 vs オーバーシュート）')
ax.legend(fontsize=7)
ax.grid(True)
fig3.tight_layout()
fig3.savefig('figures/phase6_pareto.png', dpi=150)
plt.close(fig3)

print("done")
