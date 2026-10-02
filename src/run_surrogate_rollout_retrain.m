%% run_surrogate_rollout_retrain.m
% 目的: Phase5で判明したサロゲートのロールアウト不安定性（Issue #5）を受け、
%       1ステップMSE学習から「ロールアウト損失学習＋相対状態表現」へ
%       サロゲートを再学習する。
%
% 背景（詳細はIssue #4, #5参照）:
%   - Phase4は1ステップMSEで学習したため、自己回帰ロールアウトで誤差が蓄積し
%     Phase5の閉ループ検証で2秒以内に発散（最大誤差307%）。
%   - 根本原因1: 学習時のチャープ振幅(0.2N*m)が閉ループ制御時の飽和トルク
%     (5N*m)をカバーしておらず、学習データ範囲外への外挿が発生。
%   - 根本原因2: 絶対角度theta_a/theta_mを入力に含めると、閉ループで
%     theta_aが学習データに現れない範囲までドリフトした際に外挿が破綻する
%     （theta_aの絶対値は物理的にdelta_theta_a=omega_aの運動学的関係のみで
%     決まり、ダイナミクスとは無関係であるにもかかわらず）。
%
% 対策:
%   1. チャープ振幅を3N*mまで拡大したデータを追加（飽和域もカバー）
%   2. 入力を[theta_m-theta_a(関節ねじれ角), omega_a, tau]の3次元相対表現に変更
%      （2出力: d_spring_defl, d_omega_a。delta_theta_aは台形積分で運動学的に導出）
%   3. K=20ステップ先までのロールアウト誤差を直接最小化する学習
%      （dlnetwork + カスタム学習ループ, Adam）
%
% 結果: Phase5再検証で最大誤差 307% -> 56.1%, 終端誤差 -> 9.1% に改善
%       （5%基準には未達だが、発散せず正しい方向に収束するようになった）
%       K=100への追加ファインチューニングは不安定化・悪化(672%)したため不採用。
%
% 出力: data/results/surrogate_best.mat
%       （surrogate_net_s, surrogate_net_o, x_center_r, x_scale_r, dy_scale_r）
%       src/generated_rollout_rel/surrogate_rel_spring.m, surrogate_rel_omega.m
%       （Simulink MATLAB Functionブロックから呼び出すコード生成互換関数）
%       models/surrogate/cascade_controller_surrogate.slx,
%       models/surrogate/surrogate_block.slx を新モデルに更新済み
%
% 本スクリプトは実行手順の記録用であり、MATLABの対話実行で逐次検証しながら
% 構築したため全体を通した自動実行は想定していない（各セクションを個別に
% 参照・再現する用途）。
% 作成日: 2026-10-02

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils'); addpath('src'); addpath('src/generated_rollout_rel');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

%% 1. 生データ生成（チャープ x2種 + PTP x5, 2kHzへダウンサンプリング）
chirp_tamed = generate_chirp_data(0.5, 150, 30, 0.2, p);   % Phase4と同一（通常域）
chirp_wide  = generate_chirp_data(0.5, 150, 15, 3.0, p);   % 追加: 飽和域カバー

targets_deg = [20, -30, 45, -60, 75];
move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
ptp_raw = cell(1,5);
for i = 1:5
    ptp_raw{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
end

factor = 4; % 8kHz -> 2kHz
chirp_2k = downsample_plant_data(chirp_tamed, factor);
chirp_wide_2k = downsample_plant_data(chirp_wide, factor);
ptp_2k = cell(1,5);
for i = 1:5
    ptp_2k{i} = downsample_plant_data(ptp_raw{i}, factor);
end

trajs = [{chirp_2k}, {chirp_wide_2k}, ptp_2k];
for ti = 1:numel(trajs)
    trajs{ti}.spring_defl = trajs{ti}.theta_m - trajs{ti}.theta_a;
end

%% 2. ロールアウト学習ウィンドウの構築 (K=20)
K = 20;
rng(99);
all_starts = [];
for ti = 1:numel(trajs)
    N = length(trajs{ti}.t);
    all_starts = [all_starts; ti*ones(N-K-1,1), (1:(N-K-1))']; %#ok<AGROW>
end
N_TRAIN_W = 8000; N_VAL_W = 1000;
idx_all = randperm(size(all_starts,1), N_TRAIN_W+N_VAL_W);
train_starts = all_starts(idx_all(1:N_TRAIN_W), :);
val_starts = all_starts(idx_all(N_TRAIN_W+1:end), :);

[X0_train, TauSeq_train, Targets_train] = local_build_windows(trajs, train_starts, K);
[X0_val, TauSeq_val, Targets_val] = local_build_windows(trajs, val_starts, K);

%% 3. 正規化定数の算出
X_concat = []; dX_concat = [];
for ti = 1:numel(trajs)
    Xt = [trajs{ti}.spring_defl, trajs{ti}.omega_a, trajs{ti}.tau];
    X_concat = [X_concat; Xt]; %#ok<AGROW>
    dX_concat = [dX_concat; diff(Xt(:,1:2))]; %#ok<AGROW>
end
x_center_r = mean(X_concat)'; x_scale_r = std(X_concat)' + 1e-8;
dy_scale_r = std(dX_concat)' + 1e-12;

%% 4. ロールアウト損失学習 (K=20, 50エポック, バッチ256)
[net_s_full, net_o_full, info] = train_surrogate_rollout_rel( ...
    X0_train, TauSeq_train, Targets_train, X0_val, TauSeq_val, Targets_val, ...
    x_center_r, x_scale_r, dy_scale_r, 50, 256);

%% 5. コード生成互換関数としてエクスポート
mkdir('src/generated_rollout_rel');
export_dlnet_function(net_s_full, x_center_r, x_scale_r, dy_scale_r(1), ...
    'src/generated_rollout_rel/surrogate_rel_spring.m', 'surrogate_rel_spring');
export_dlnet_function(net_o_full, x_center_r, x_scale_r, dy_scale_r(2), ...
    'src/generated_rollout_rel/surrogate_rel_omega.m', 'surrogate_rel_omega');

%% 6. 保存
surrogate_net_s = net_s_full; %#ok<NASGU>
surrogate_net_o = net_o_full; %#ok<NASGU>
best_rate_hz = 2000; %#ok<NASGU>
best_rate_id = 'rate_2khz_rollout_rel'; %#ok<NASGU>
save('data/results/surrogate_best.mat', 'surrogate_net_s', 'surrogate_net_o', ...
    'x_center_r', 'x_scale_r', 'dy_scale_r', 'best_rate_hz', 'best_rate_id');

disp('サロゲート再学習完了。models/surrogate/*.slx のMATLAB Functionブロックは');
disp('surrogate_rel_spring/surrogate_rel_omega を呼ぶよう手動更新済み（Issue #5参照）。');

function [X0, TauSeq, Targets] = local_build_windows(trajs, starts, K)
    Nw = size(starts,1);
    X0 = zeros(3, Nw);
    TauSeq = zeros(K, Nw);
    Targets = zeros(2, K, Nw);
    for w = 1:Nw
        ti = starts(w,1); s = starts(w,2);
        tr = trajs{ti};
        X0(:,w) = [tr.spring_defl(s); tr.omega_a(s); tr.tau(s)];
        TauSeq(:,w) = tr.tau(s+1:s+K);
        Targets(1,:,w) = tr.spring_defl(s+1:s+K);
        Targets(2,:,w) = tr.omega_a(s+1:s+K);
    end
end
