"""Issue #4追加検証: 30Hz共振に対するオーバーサンプリング比と精度の関係を可視化
入力: data/results/phase4c_rate_sweep.csv
出力: figures/phase4c_oversampling_vs_accuracy.png
作成日: 2026-10-04
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

rows = read_csv("data/results/phase4c_rate_sweep.csv")[1:]
rate_hz = [to_float(r[0]) for r in rows]
ratio = [to_float(r[1]) for r in rows]
pos_rmse = [to_float(r[2]) for r in rows]
phase_err = [to_float(r[5]) for r in rows]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

ax1.plot(ratio, phase_err, 'o-', color='green')
ax1.axhline(20.0, color='r', linestyle='--', label='合格基準 20deg')
for x, y, f in zip(ratio, phase_err, rate_hz):
    ax1.annotate(f'{int(f)}Hz', (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8)
ax1.set_xlabel('30Hzに対するオーバーサンプリング比 [倍]')
ax1.set_ylabel('30Hz位相誤差 [deg]')
ax1.set_title('位相誤差 vs オーバーサンプリング比\n（境界は約20倍付近）')
ax1.legend()
ax1.grid(True)

ax2.semilogy(ratio, pos_rmse, 'o-', color='blue')
ax2.axhline(1.0, color='r', linestyle='--', label='合格基準 1mrad')
for x, y, f in zip(ratio, pos_rmse, rate_hz):
    ax2.annotate(f'{int(f)}Hz', (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8)
ax2.set_xlabel('30Hzに対するオーバーサンプリング比 [倍]')
ax2.set_ylabel('位置RMSE [mrad] (log scale)')
ax2.set_title('位置RMSE vs オーバーサンプリング比\n（崩壊なし、単調改善）')
ax2.legend()
ax2.grid(True, which='both')

fig.suptitle('Issue #4追加検証: 500Hz-1kHz間の細かい走査（マルコフ完備な特徴量）')
fig.tight_layout()
fig.savefig('figures/phase4c_oversampling_vs_accuracy.png', dpi=150)
plt.close(fig)

print("done")
