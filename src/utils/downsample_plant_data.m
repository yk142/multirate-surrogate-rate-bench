function s = downsample_plant_data(d, factor)
% downsample_plant_data  プラントデータ(Ts_plant)を指定倍率でダウンサンプリング
% 入力: d - struct with fields t, tau, theta_a, omega_a, theta_m (Ts_plantレート)
%       factor - ダウンサンプリング倍率（整数, 1ならそのまま）
% 出力: s - 同フィールドを持つダウンサンプリング後のstruct
%       （decimateによるアンチエイリアスIIRフィルタを適用。factor=1は素通し）
% 作成日: 2026-09-28
    if factor <= 1
        s = d;
        return;
    end
    s.t       = d.t(1:factor:end);
    s.tau     = decimate(d.tau, factor);
    s.theta_a = decimate(d.theta_a, factor);
    s.omega_a = decimate(d.omega_a, factor);
    s.theta_m = decimate(d.theta_m, factor);
    n = min([length(s.t), length(s.tau), length(s.theta_a), length(s.omega_a), length(s.theta_m)]);
    s.t = s.t(1:n); s.tau = s.tau(1:n); s.theta_a = s.theta_a(1:n);
    s.omega_a = s.omega_a(1:n); s.theta_m = s.theta_m(1:n);
end
