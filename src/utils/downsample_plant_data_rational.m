function s = downsample_plant_data_rational(d, target_rate_hz, source_rate_hz)
% downsample_plant_data_rational  任意の目標レートへのリサンプリング
% （8kHzの整数約数でないレート、例: 600/700/900Hzにも対応するため
% decimateではなくresample（有理数比p/q）を使用する）
% 入力: d - struct with fields t, tau, theta_a, omega_a, theta_m (source_rate_hzレート)
%       target_rate_hz - 目標サンプリングレート [Hz]
%       source_rate_hz - dの現在のサンプリングレート [Hz]
% 出力: s - 同フィールドを持つリサンプリング後のstruct
% 作成日: 2026-10-04
    [p, q] = rat(target_rate_hz / source_rate_hz, 1e-6);
    tau_r     = resample(d.tau, p, q);
    theta_a_r = resample(d.theta_a, p, q);
    omega_a_r = resample(d.omega_a, p, q);
    theta_m_r = resample(d.theta_m, p, q);
    n = min([length(tau_r), length(theta_a_r), length(omega_a_r), length(theta_m_r)]);

    % resampleのFIRアンチエイリアスフィルタは信号の急峻な立ち上がり(チャープ/PTP
    % 開始点)でリンギングを生じるため、先頭・末尾を一定時間トリムして除去する
    % （decimateのIIRフィルタでは目立たなかったがresampleでは顕著だったため、
    % Issue #4追加検証で発見・対応）
    trim_sec = 0.05;
    trim_n = round(trim_sec * target_rate_hz);
    keep = (trim_n+1):(n-trim_n);

    s.tau = tau_r(keep); s.theta_a = theta_a_r(keep);
    s.omega_a = omega_a_r(keep); s.theta_m = theta_m_r(keep);
    s.t = (0:numel(keep)-1)' / target_rate_hz;
end
