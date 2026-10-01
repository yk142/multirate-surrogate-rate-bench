"""Phase 4 の図（学習曲線・RMSE/位相誤差/ロールアウト比較・8kHz崩壊）を生成する
入力: data/results/phase4_comparison_table.csv
      data/results/phase4_training_curves.csv
      data/results/phase4_rollout_rate_*.csv
      data/results/phase4_8khz_scatter.csv, phase4_1khz_scatter.csv
出力: figures/phase4_training_curve_all_rates.png
      figures/phase4_rmse_vs_rate.png
      figures/phase4_phase_error_vs_rate.png
      figures/phase4_rollout_comparison.png
      figures/phase4_8khz_collapse.png
作成日: 2026-09-28
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

# --- 比較表 ---
table_rows = read_csv("data/results/phase4_comparison_table.csv")[1:]
rates_hz = [to_float(r[0]) for r in table_rows]
pos_rmse = [to_float(r[1]) for r in table_rows]
vel_rmse = [to_float(r[2]) for r in table_rows]
r2_pos = [to_float(r[3]) for r in table_rows]
phase_err = [to_float(r[4]) for r in table_rows]
rollout_rmse = [to_float(r[5]) for r in table_rows]
diverged = [int(to_float(r[6])) for r in table_rows]

# --- 図1: 学習曲線（4レート重ね描き） ---
curve_rows = read_csv("data/results/phase4_training_curves.csv")
curve_cols = list(zip(*[[to_float(v) for v in row] for row in curve_rows]))

fig1, ax = plt.subplots(figsize=(8, 6))
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
for i, label in enumerate(rate_labels):
    train_col = curve_cols[2*i]
    val_col = curve_cols[2*i+1]
    train_col = [v for v in train_col if not math.isnan(v)]
    val_col = [v for v in val_col if not math.isnan(v)]
    ax.plot(range(len(train_col)), train_col, '-', color=colors[i % len(colors)], label=f'{label} train')
    ax.plot(range(len(val_col)), val_col, '--', color=colors[i % len(colors)], label=f'{label} val')
ax.set_yscale('log')
ax.set_xlabel('エポック')
ax.set_ylabel('MSE損失 (log scale)')
ax.set_title('Phase 4: レート別学習曲線 (Train/Validation)')
ax.legend(fontsize=8, ncol=2)
ax.grid(True, which='both')
fig1.tight_layout()
fig1.savefig('figures/phase4_training_curve_all_rates.png', dpi=150)
plt.close(fig1)

# --- 図2: RMSE vs レート（主要結果） ---
fig2, (axp, axv) = plt.subplots(1, 2, figsize=(11, 5))
bars1 = axp.bar(rate_labels, pos_rmse, color=colors[0])
axp.axhline(1.0, color='r', linestyle='--', label='合格基準 1mrad')
axp.set_ylabel('位置RMSE [mrad]')
axp.set_title('レート vs 位置RMSE')
axp.legend()
for b, v in zip(bars1, pos_rmse):
    axp.text(b.get_x()+b.get_width()/2, v, f'{v:.3f}', ha='center', va='bottom', fontsize=8)

bars2 = axv.bar(rate_labels, vel_rmse, color=colors[1])
axv.axhline(10.0, color='r', linestyle='--', label='合格基準 10mrad/s')
axv.set_ylabel('速度RMSE [mrad/s]')
axv.set_title('レート vs 速度RMSE')
axv.legend()
for b, v in zip(bars2, vel_rmse):
    axv.text(b.get_x()+b.get_width()/2, v, f'{v:.3f}', ha='center', va='bottom', fontsize=8)

fig2.suptitle('Phase 4: レート別RMSE比較（主要結果）')
fig2.tight_layout()
fig2.savefig('figures/phase4_rmse_vs_rate.png', dpi=150)
plt.close(fig2)

# --- 図3: 位相誤差 vs レート ---
# NaN(ロールアウト発散のため評価不能)は0を代入して描画し、x軸が1カテゴリに
# 自動ズームされる問題を回避したうえで"N/A"と明示する
phase_err_plot = [0.0 if math.isnan(v) else v for v in phase_err]
fig3, ax3 = plt.subplots(figsize=(7, 5))
bars3 = ax3.bar(rate_labels, phase_err_plot, color=colors[2])
ax3.axhline(20.0, color='r', linestyle='--', label='合格基準 20deg')
ax3.set_ylabel('30Hz位相誤差 [deg]')
ax3.set_title('レート vs 30Hz位相誤差（ロールアウト発散時はN/A）')
ax3.set_xlim(-0.5, len(rate_labels) - 0.5)
ax3.legend()
for b, v, vplot in zip(bars3, phase_err, phase_err_plot):
    label = 'N/A(発散)' if math.isnan(v) else f'{v:.1f}'
    ax3.text(b.get_x()+b.get_width()/2, vplot, label, ha='center', va='bottom', fontsize=8)
fig3.tight_layout()
fig3.savefig('figures/phase4_phase_error_vs_rate.png', dpi=150)
plt.close(fig3)

# --- 図4: ロールアウト比較（4レート, 30Hz正弦応答） ---
fig4, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
for i, (rid, label) in enumerate(zip(rate_ids, rate_labels)):
    ax = axes[i // 2][i % 2]
    try:
        rows = read_csv(f"data/results/phase4_rollout_{rid}.csv")
        t = [to_float(r[0]) for r in rows]
        true_y = [to_float(r[1]) for r in rows]
        roll_y = [to_float(r[2]) for r in rows]
        ax.plot(t, true_y, '-', label='真値')
        ax.plot(t, roll_y, '--', label='サロゲートロールアウト')
        ax.legend(fontsize=8)
    except FileNotFoundError:
        ax.text(0.5, 0.5, 'データなし', ha='center', va='center', transform=ax.transAxes)
    ax.set_title(f'{label}')
    ax.grid(True)
fig4.supxlabel('時間 [s]')
fig4.supylabel('theta_a [rad]')
fig4.suptitle('Phase 4: レート別ロールアウト比較（30Hz正弦応答）')
fig4.tight_layout()
fig4.savefig('figures/phase4_rollout_comparison.png', dpi=150)
plt.close(fig4)

# --- 図5: 8kHz崩壊（自明解への退化） ---
fig5, (ax5a, ax5b) = plt.subplots(1, 2, figsize=(11, 5))
for ax, path, title in [(ax5a, "data/results/phase4_1khz_scatter.csv", "1kHzモデル"),
                          (ax5b, "data/results/phase4_8khz_scatter.csv", "8kHzモデル")]:
    rows = read_csv(path)
    true_d = [to_float(r[0]) for r in rows]
    pred_d = [to_float(r[1]) for r in rows]
    ax.scatter(true_d, pred_d, s=2, alpha=0.3)
    lim = max(max(map(abs, true_d)), 1e-6)
    ax.plot([-lim, lim], [-lim, lim], 'r--', linewidth=1, label='理想(y=x)')
    ax.set_xlabel('真の delta_theta_a [rad]')
    ax.set_ylabel('予測 delta_theta_a [rad]')
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True)
fig5.suptitle('Phase 4: 8kHzモデルの自明解(delta_x≈0)への退化確認')
fig5.tight_layout()
fig5.savefig('figures/phase4_8khz_collapse.png', dpi=150)
plt.close(fig5)

print("done")
