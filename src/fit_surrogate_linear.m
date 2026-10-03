%% fit_surrogate_linear.m
% 目的: サロゲートを「マルコフ完備な相対状態表現＋線形回帰」で学習する最終版。
%
% 背景（詳細はIssue #5参照）:
%   Phase4(1ステップMSE)、ロールアウト損失学習(K=20/40/60)、DAgger的追加学習を
%   試したが、いずれもPhase5の閉ループ検証で発散または大きな誤差が残った
%   （最良でも最大誤差56%）。
%
%   原因を調査した結果、REQUIREMENTS.md指定の入力 x=[theta_a,omega_a,theta_m,tau]
%   には、モータ角速度omega_m相当の情報が欠落しており、関節ねじれ角
%   (theta_m-theta_a)の将来予測に理論上必要な状態量が不足していた
%   （観測可能性・マルコフ性の欠如）。有限差分 spring_defl_rate =
%   (spring_defl(k)-spring_defl(k-1))/Ts を追加特徴量とし、
%   feat=[spring_defl, spring_defl_rate, omega_a, tau] の4次元相対状態表現に
%   拡張したところ、真の物理系が線形であることから、単純な線形回帰だけで
%   R^2=1.0000（完全な線形関係）を達成した。
%
% 出力: data/results/surrogate_best.mat
%       （surrogate_coef_spring, surrogate_coef_omega, surrogate_Ts）
%       src/generated_linear/surrogate_predict_linear.m
%         （Simulink MATLAB Functionブロックから呼び出すコード生成互換関数。
%          ただし実際のSimulinkブロックはチャートスクリプトに直接数式を
%          埋め込んでいる。外部関数呼び出しで原因不明の次元不一致エラーが
%          発生したため、models/surrogate/*.slx 側はインライン実装を採用）
%
% 本スクリプトは手順の記録用（対話実行で逐次検証しながら構築したため、
% 一括実行は想定していない）。
% 作成日: 2026-10-03

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils'); addpath('src');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

%% 1. 学習データ生成（チャープ x2種 + PTP x5, 2kHzへダウンサンプリング）
chirp_tamed = generate_chirp_data(0.5, 150, 30, 0.2, p);
chirp_wide  = generate_chirp_data(0.5, 150, 15, 3.0, p); % 飽和域カバー

targets_deg = [20, -30, 45, -60, 75];
move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
ptp_raw = cell(1,5);
for i = 1:5
    ptp_raw{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
end

factor = 4; % 8kHz -> 2kHz
Ts = 1/2000;
trajs = [{downsample_plant_data(chirp_tamed, factor)}, {downsample_plant_data(chirp_wide, factor)}];
for i = 1:5
    trajs{end+1} = downsample_plant_data(ptp_raw{i}, factor); %#ok<SAGROW>
end
for ti = 1:numel(trajs)
    trajs{ti}.spring_defl = trajs{ti}.theta_m - trajs{ti}.theta_a;
end

%% 2. 特徴量 [spring_defl, spring_defl_rate, omega_a, tau] と1ステップ先の
%%    delta_spring_defl, delta_omega_a を全軌道から収集
X_all = []; dSpring_all = []; dOmega_all = [];
for ti = 1:numel(trajs)
    tr = trajs{ti};
    N = length(tr.t);
    rate = [NaN; diff(tr.spring_defl)/Ts]; % s=1は計算不可なので後で除外
    X = [tr.spring_defl(2:N-1), rate(2:N-1), tr.omega_a(2:N-1), tr.tau(2:N-1)];
    dSpring = tr.spring_defl(3:N) - tr.spring_defl(2:N-1);
    dOmega  = tr.omega_a(3:N) - tr.omega_a(2:N-1);
    X_all = [X_all; X]; %#ok<AGROW>
    dSpring_all = [dSpring_all; dSpring]; %#ok<AGROW>
    dOmega_all = [dOmega_all; dOmega]; %#ok<AGROW>
end

%% 3. 線形回帰（最小二乗）
Xreg = [X_all, ones(size(X_all,1),1)];
surrogate_coef_spring = Xreg \ dSpring_all;
surrogate_coef_omega  = Xreg \ dOmega_all;

r2_spring = 1 - var(Xreg*surrogate_coef_spring - dSpring_all) / var(dSpring_all);
r2_omega  = 1 - var(Xreg*surrogate_coef_omega  - dOmega_all ) / var(dOmega_all);
fprintf('R2 spring_defl=%.4f, R2 omega_a=%.4f\n', r2_spring, r2_omega);

%% 4. Simulinkコード生成互換関数の書き出し・保存
surrogate_Ts = Ts;
mkdir('src/generated_linear');
export_linear_coefs(surrogate_coef_spring, surrogate_coef_omega, Ts, ...
    'src/generated_linear/surrogate_predict_linear.m');

best_rate_hz = 2000; %#ok<NASGU>
best_rate_id = 'rate_2khz_linear_markov'; %#ok<NASGU>
save('data/results/surrogate_best.mat', 'surrogate_coef_spring', 'surrogate_coef_omega', ...
    'surrogate_Ts', 'best_rate_hz', 'best_rate_id');

disp('線形サロゲート学習完了。models/surrogate/*.slx は手動でインライン実装済み（Issue #5参照）。');
