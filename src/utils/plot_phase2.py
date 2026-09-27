"""phase2_ptp_response.png / phase2_notch_comparison.png を生成する
（MATLAB描画基盤の障害に対する回避策としてPythonで描画）
入力: data/results/phase2_notch_on.csv, data/results/phase2_notch_off.csv
      各列: t, theta_ref, theta_a, omega_a, tau
出力: figures/phase2_ptp_response.png, figures/phase2_notch_comparison.png
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

def cols(rows):
    t = [r[0] for r in rows]
    theta_ref = [r[1] for r in rows]
    theta_a = [r[2] for r in rows]
    omega_a = [r[3] for r in rows]
    tau = [r[4] for r in rows]
    return t, theta_ref, theta_a, omega_a, tau

on = cols(read_csv("data/results/phase2_notch_on.csv"))
off = cols(read_csv("data/results/phase2_notch_off.csv"))
t_on, ref_on, th_on, om_on, tau_on = on
t_off, ref_off, th_off, om_off, tau_off = off

# --- 出力図1: 位置・速度・トルク・追従誤差のタイムライン（4段組、notch_on） ---
err_on = [r - a for r, a in zip(ref_on, th_on)]

fig1, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
axes[0].plot(t_on, ref_on, '--', label='参照')
axes[0].plot(t_on, th_on, '-', label='応答')
axes[0].set_ylabel('theta [rad]')
axes[0].legend(loc='lower right')
axes[0].set_title('Phase 2: PTP応答（位置・速度・トルク・追従誤差）')
axes[0].grid(True)

axes[1].plot(t_on, om_on)
axes[1].set_ylabel('omega_a [rad/s]')
axes[1].grid(True)

axes[2].plot(t_on, tau_on)
axes[2].set_ylabel('tau [N*m]')
axes[2].grid(True)

axes[3].plot(t_on, err_on)
axes[3].set_ylabel('誤差 [rad]')
axes[3].set_xlabel('時間 [s]')
axes[3].grid(True)

fig1.tight_layout()
fig1.savefig('figures/phase2_ptp_response.png', dpi=150)
plt.close(fig1)

# --- 出力図2: ノッチフィルタあり/なしの比較 ---
fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6))
ax1.plot(t_on, th_on, '-', label='ノッチON')
ax1.plot(t_off, th_off, '--', label='ノッチOFF')
ax1.set_ylabel('theta_a [rad]')
ax1.set_title('ノッチフィルタ有無による応答比較')
ax1.legend(loc='upper left')
ax1.grid(True)

ax2.plot(t_on, tau_on, '-', label='ノッチON')
ax2.plot(t_off, tau_off, '--', label='ノッチOFF')
ax2.set_ylabel('tau [N*m]')
ax2.set_xlabel('時間 [s]')
ax2.legend(loc='upper right')
ax2.grid(True)

fig2.tight_layout()
fig2.savefig('figures/phase2_notch_comparison.png', dpi=150)
plt.close(fig2)

print("done")
