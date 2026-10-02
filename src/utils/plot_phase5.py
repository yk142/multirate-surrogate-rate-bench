"""Phase 5 の図（物理モデル vs サロゲート応答比較、位置誤差）を生成する
入力: data/results/phase5_physical.csv (列: t, theta_a)
      data/results/phase5_surrogate.csv (列: t, theta_a)
      data/results/phase5_error.csv (列: t, theta_phys_on_sur, theta_sur, err, rel_err_pct)
出力: figures/phase5_comparison.png, figures/phase5_error.png
作成日: 2026-10-02
"""
import csv
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'
import matplotlib.pyplot as plt

def read_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.reader(f):
            rows.append([float(x) for x in row])
    return rows

phys = read_csv("data/results/phase5_physical.csv")
t_phys = [r[0] for r in phys]
theta_phys = [r[1] for r in phys]

sur = read_csv("data/results/phase5_surrogate.csv")
t_sur = [r[0] for r in sur]
theta_sur = [r[1] for r in sur]

err_rows = read_csv("data/results/phase5_error.csv")
t_e = [r[0] for r in err_rows]
err = [r[3] for r in err_rows]
rel_err_pct = [r[4] for r in err_rows]

# --- 図1: 物理モデル vs サロゲート応答比較 ---
fig1, ax = plt.subplots(figsize=(9, 5))
ax.plot(t_phys, theta_phys, '-', label='物理モデル', linewidth=1.5)
ax.plot(t_sur, theta_sur, '--', label='サロゲート(2kHz)', linewidth=1.5)
ax.axhline(0.7854, color='gray', linestyle=':', linewidth=1, label='目標位置(45deg)')
ax.set_xlabel('時間 [s]')
ax.set_ylabel('theta_a [rad]')
ax.set_title('Phase 5: 物理モデル vs サロゲート 閉ループ応答比較')
ax.legend()
ax.grid(True)
fig1.tight_layout()
fig1.savefig('figures/phase5_comparison.png', dpi=150)
plt.close(fig1)

# --- 図2: 位置誤差の時系列 ---
fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax1.plot(t_e, err)
ax1.set_ylabel('位置誤差 [rad]')
ax1.set_title('Phase 5: 物理モデルとサロゲートの位置誤差')
ax1.grid(True)

ax2.plot(t_e, rel_err_pct)
ax2.axhline(5.0, color='r', linestyle='--', label='合格基準 5%')
ax2.set_ylabel('相対誤差 [%]')
ax2.set_xlabel('時間 [s]')
ax2.legend()
ax2.grid(True)

fig2.tight_layout()
fig2.savefig('figures/phase5_error.png', dpi=150)
plt.close(fig2)

print("done")
