"""phase3_training_data_overview.png / phase3_input_spectrum.png を生成する
入力: data/results/phase3_overview.csv (列: t, tau, theta_a, omega_a)
      data/results/phase3_spectrum.csv (列: freq_hz, power)
出力: figures/phase3_training_data_overview.png, figures/phase3_input_spectrum.png
作成日: 2026-09-28
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

ov = read_csv("data/results/phase3_overview.csv")
t = [r[0] for r in ov]
tau = [r[1] for r in ov]
theta_a = [r[2] for r in ov]
omega_a = [r[3] for r in ov]

fig1, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
axes[0].plot(t, tau, linewidth=0.7)
axes[0].set_ylabel('tau [N*m]')
axes[0].set_title('Phase 3: 学習データ概観（前半: チャープ5s, 後半: PTP軌道1）')
axes[0].grid(True)

axes[1].plot(t, theta_a, linewidth=0.7)
axes[1].set_ylabel('theta_a [rad]')
axes[1].grid(True)

axes[2].plot(t, omega_a, linewidth=0.7)
axes[2].set_ylabel('omega_a [rad/s]')
axes[2].set_xlabel('時間 [s]')
axes[2].grid(True)

fig1.tight_layout()
fig1.savefig('figures/phase3_training_data_overview.png', dpi=150)
plt.close(fig1)

sp = read_csv("data/results/phase3_spectrum.csv")
freq = [r[0] for r in sp]
power = [r[1] for r in sp]

fig2, ax = plt.subplots(figsize=(8, 5))
ax.semilogy(freq, power)
ax.axvspan(0.5, 150, color='orange', alpha=0.15, label='チャープ帯域(0.5-150Hz)')
ax.axvline(30, color='r', linestyle=':', linewidth=1, label='30Hz共振')
ax.set_xlabel('周波数 [Hz]')
ax.set_ylabel('パワースペクトル密度')
ax.set_title('入力トルクのパワースペクトル (0-200Hz)')
ax.legend()
ax.grid(True)

fig2.tight_layout()
fig2.savefig('figures/phase3_input_spectrum.png', dpi=150)
plt.close(fig2)

print("done")
