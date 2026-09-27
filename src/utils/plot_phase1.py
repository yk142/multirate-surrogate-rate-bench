"""phase1_step.png / phase1_bode.png を再生成する（MATLAB描画基盤の障害に対する回避策）
入力: data/results/phase1_bode_data.csv, data/results/phase1_step_data.csv
出力: figures/phase1_bode.png, figures/phase1_step.png
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

bode = read_csv("data/results/phase1_bode_data.csv")
freq_hz = [r[0] for r in bode]
mag = [r[1] for r in bode]
phase = [r[2] for r in bode]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
ax1.semilogx(freq_hz, mag)
ax1.axvline(30, color='r', linestyle=':', linewidth=1)
ax1.set_ylabel('振幅 [dB]')
ax1.set_title('周波数応答: tau -> theta_a (30Hz共振モード)')
ax1.grid(True, which='both')

ax2.semilogx(freq_hz, phase)
ax2.axvline(30, color='r', linestyle=':', linewidth=1)
ax2.set_ylabel('位相 [deg]')
ax2.set_xlabel('周波数 [Hz]')
ax2.grid(True, which='both')

fig.tight_layout()
fig.savefig('figures/phase1_bode.png', dpi=150)
plt.close(fig)

step = read_csv("data/results/phase1_step_data.csv")
t = [r[0] for r in step]
theta_a = [r[1] for r in step]
spring_defl = [r[2] for r in step]

fig2, (axa, axb) = plt.subplots(2, 1, figsize=(8, 7))
axa.plot(t, theta_a)
axa.set_ylabel('theta_a [rad]')
axa.set_title('ステップ応答 (tau = 1 N*m)')
axa.grid(True)

axb.plot(t, spring_defl)
axb.set_xlabel('時間 [s]')
axb.set_ylabel('関節ねじれ角 theta_m - theta_a [rad]')
axb.set_title('30Hz振動の確認（弾性関節のねじれ角、剛体成分を含まない）')
axb.grid(True)

fig2.tight_layout()
fig2.savefig('figures/phase1_step.png', dpi=150)
plt.close(fig2)

print("done")
