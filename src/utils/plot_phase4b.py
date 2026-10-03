"""Phase 4(マルコフ完備版再実験)の図を生成する
入力: data/results/phase4b_comparison_table.csv
      data/results/phase4b_rollout_rate_*.csv
出力: figures/phase4b_rmse_vs_rate.png
      figures/phase4b_phase_rollout_vs_rate.png
      figures/phase4b_rollout_comparison.png
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

rate_ids = ['rate_500hz', 'rate_1khz', 'rate_2khz', 'rate_8khz']
rate_labels = ['500Hz', '1kHz', '2kHz', '8kHz']

rows = read_csv("data/results/phase4b_comparison_table.csv")[1:]
pos_rmse = [to_float(r[1]) for r in rows]
vel_rmse = [to_float(r[2]) for r in rows]
r2_pos = [to_float(r[3]) for r in rows]
phase_err = [to_float(r[5]) for r in rows]
rollout_rmse = [to_float(r[6]) for r in rows]

colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

# --- 図1: RMSE vs レート（対数軸, Phase4との比較用） ---
fig1, (axp, axv) = plt.subplots(1, 2, figsize=(11, 5))
axp.semilogy(rate_labels, pos_rmse, 'o-', color=colors[0])
axp.axhline(1.0, color='r', linestyle='--', label='合格基準 1mrad')
axp.set_ylabel('位置RMSE [mrad] (log scale)')
axp.set_title('レート vs 位置RMSE（マルコフ完備版）')
axp.legend()
axp.grid(True, which='both')

axv.semilogy(rate_labels, vel_rmse, 'o-', color=colors[1])
axv.axhline(10.0, color='r', linestyle='--', label='合格基準 10mrad/s')
axv.set_ylabel('速度RMSE [mrad/s] (log scale)')
axv.set_title('レート vs 速度RMSE（マルコフ完備版）')
axv.legend()
axv.grid(True, which='both')

fig1.suptitle('Phase 4再実験: レート別RMSE比較（マルコフ完備な特徴量、線形回帰）')
fig1.tight_layout()
fig1.savefig('figures/phase4b_rmse_vs_rate.png', dpi=150)
plt.close(fig1)

# --- 図2: 位相誤差・ロールアウトRMSE vs レート ---
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))
ax1.plot(rate_labels, phase_err, 'o-', color=colors[2])
ax1.axhline(20.0, color='r', linestyle='--', label='合格基準 20deg')
ax1.set_ylabel('30Hz位相誤差 [deg]')
ax1.set_title('レート vs 30Hz位相誤差')
ax1.legend()
ax1.grid(True)

ax2.semilogy(rate_labels, rollout_rmse, 'o-', color=colors[3])
ax2.set_ylabel('ロールアウトRMSE [rad] (log scale, 2s)')
ax2.set_title('レート vs ロールアウト誤差（全レート発散せず）')
ax2.grid(True, which='both')

fig2.suptitle('Phase 4再実験: 位相誤差・ロールアウト安定性（マルコフ完備な特徴量）')
fig2.tight_layout()
fig2.savefig('figures/phase4b_phase_rollout_vs_rate.png', dpi=150)
plt.close(fig2)

# --- 図3: ロールアウト比較（4レート, 30Hz正弦応答） ---
fig3, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
for i, (rid, label) in enumerate(zip(rate_ids, rate_labels)):
    ax = axes[i // 2][i % 2]
    r = read_csv(f"data/results/phase4b_rollout_{rid}.csv")
    t = [to_float(x[0]) for x in r]
    true_y = [to_float(x[1]) for x in r]
    roll_y = [to_float(x[2]) for x in r]
    ax.plot(t, true_y, '-', label='真値', linewidth=1)
    ax.plot(t, roll_y, '--', label='サロゲートロールアウト', linewidth=1)
    ax.legend(fontsize=8)
    ax.set_title(label)
    ax.grid(True)
fig3.supxlabel('時間 [s]')
fig3.supylabel('theta_a [rad]')
fig3.suptitle('Phase 4再実験: レート別ロールアウト比較（30Hz正弦応答, 2秒間）')
fig3.tight_layout()
fig3.savefig('figures/phase4b_rollout_comparison.png', dpi=150)
plt.close(fig3)

print("done")
