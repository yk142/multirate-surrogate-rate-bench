function phase_deg = fft_phase_at_freq(t, x, f)
% fft_phase_at_freq  時系列信号x(t)の周波数fにおけるFFT位相[deg]を求める
% 入力: t - 等間隔時刻ベクトル, x - 信号, f - 対象周波数[Hz]
% 出力: phase_deg - 位相 [deg]
% 作成日: 2026-09-28
    dt = t(2) - t(1);
    Fs = 1/dt;
    N = length(x);
    k = round(f*N/Fs) + 1; % 1-based index, DC=1
    X = fft(x(:) - mean(x));
    phase_deg = rad2deg(angle(X(k)));
end
