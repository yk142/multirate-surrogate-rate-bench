%% params.m
% 目的: multirate-surrogate-rate-bench プロジェクト共通パラメータ定義
% 入力: なし
% 出力: ワークスペース変数（物理パラメータ, 制御パラメータ, サンプルレート）
% 作成日: 2026-09-28

%% ロボットアームパラメータ（2慣性系, 弾性関節）
J_m  = 1e-3;      % モータ慣性モーメント [kg*m^2]
J_a  = 5e-3;      % アーム慣性モーメント [kg*m^2]
% 注: REQUIREMENTS.md記載の k_s=180 では f_res≈74Hzとなり「30Hz共振」の
% 要求と矛盾するため、30Hz共振になるよう k_s を補正している（Issue #1参照）。
k_s  = 29.61;     % 関節ばね剛性 [N*m/rad]  -> 共振: 約30 Hz
b_m  = 0.01;      % モータ粘性摩擦 [N*m*s/rad]
b_a  = 0.02;      % アーム粘性摩擦 [N*m*s/rad]
b_s  = 0.005;     % 関節ダンピング [N*m*s/rad]

% 確認: 共振周波数
f_res = (1/(2*pi)) * sqrt(k_s*(1/J_m + 1/J_a));
fprintf('共振周波数 f_res = %.2f Hz\n', f_res);

%% サンプルレート
Ts_plant = 1/8000;   % プラント（物理/サロゲート） [s] 8 kHz
Ts_vel   = 1/1000;   % 速度ループ [s] 1 kHz
Ts_pos   = 1/200;    % 位置ループ [s] 200 Hz

%% 制御器パラメータ（初期値）
% 位置ループ (P制御)
Kp_pos = 50;       % [rad/s / rad]

% 速度ループ (PI制御)
Kp_vel = 0.5;      % [N*m / (rad/s)]
Ki_vel = 5.0;      % [N*m / (rad/s*s)]

% トルク飽和
tau_max = 5.0;     % [N*m]

% ノッチフィルタ（30 Hz振動モード抑制）
f_notch  = 30;     % [Hz]
depth_dB = -20;    % [dB]
Q_notch  = 3;      % Q値
enable_notch = true;

%% NNサロゲート候補レート
surrogate_rates_hz = [500, 1000, 2000, 8000];

%% 最適化探索範囲（ゲイン最適化用）
gain_lb = [10,  0.1, 0.5];
gain_ub = [200, 5.0, 50.0];
