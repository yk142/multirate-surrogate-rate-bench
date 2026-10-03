"""Phase 7 の図（最適化前後比較・最終性能指標）を生成する
入力: data/results/phase7_initial_gains.csv (列: t, theta_a, tau)
      data/results/phase7_optimal_gains.csv (列: t, theta_a, tau)
      data/results/phase7_metrics.csv (config,settling_time,overshoot_pct,final_err_pct)
出力: figures/phase7_before_after.png
      figures/phase7_final_metrics.png
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

init_rows = read_csv("data/results/phase7_initial_gains.csv")
opt_rows = read_csv("data/results/phase7_optimal_gains.csv")
t_init = [to_float(r[0]) for r in init_rows]
theta_init = [to_float(r[1]) for r in init_rows]
tau_init = [to_float(r[2]) for r in init_rows]
t_opt = [to_float(r[0]) for r in opt_rows]
theta_opt = [to_float(r[1]) for r in opt_rows]
tau_opt = [to_float(r[2]) for r in opt_rows]

# --- 図1: 最適化前後のPTP応答比較 ---
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax1.plot(t_init, theta_init, '-', label='初期ゲイン (Kp_pos=50, Kp_vel=0.5, Ki_vel=5.0)')
ax1.plot(t_opt, theta_opt, '--', label='最適ゲイン (GA)')
ax1.axhline(0.7854, color='gray', linestyle=':', linewidth=1, label='目標(45deg)')
ax1.set_ylabel('theta_a [rad]')
ax1.set_title('Phase 7: 最適化前後のPTP応答比較（物理モデル）')
ax1.legend(fontsize=8)
ax1.grid(True)

ax2.plot(t_init, tau_init, '-', label='初期ゲイン')
ax2.plot(t_opt, tau_opt, '--', label='最適ゲイン')
ax2.set_ylabel('tau [N*m]')
ax2.set_xlabel('時間 [s]')
ax2.legend(fontsize=8)
ax2.grid(True)

fig1.tight_layout()
fig1.savefig('figures/phase7_before_after.png', dpi=150)
plt.close(fig1)

# --- 図2: 最終性能指標の棒グラフ ---
metric_rows = read_csv("data/results/phase7_metrics.csv")[1:]
configs = [r[0] for r in metric_rows]
settling = [to_float(r[1]) for r in metric_rows]
overshoot = [to_float(r[2]) for r in metric_rows]
final_err = [to_float(r[3]) for r in metric_rows]

fig2, axes = plt.subplots(1, 3, figsize=(13, 5))
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
titles = ['整定時間 [s]', 'オーバーシュート [%]', '終端追従誤差 [%]']
datasets = [settling, overshoot, final_err]
for ax, data, title in zip(axes, datasets, titles):
    bars = ax.bar(configs, data, color=colors[:len(configs)])
    ax.set_title(title)
    ax.tick_params(axis='x', rotation=15)
    for b, v in zip(bars, data):
        ax.text(b.get_x()+b.get_width()/2, v, f'{v:.3f}', ha='center', va='bottom', fontsize=8)
    ax.grid(True, axis='y')

fig2.suptitle('Phase 7: 最終性能指標比較（物理モデル）')
fig2.tight_layout()
fig2.savefig('figures/phase7_final_metrics.png', dpi=150)
plt.close(fig2)

print("done")
