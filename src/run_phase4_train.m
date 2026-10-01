%% run_phase4_train.m
% 目的: Phase 4 NNサロゲート学習とレート別(500Hz/1kHz/2kHz/8kHz)精度比較
% 入力: src/params.m, src/train_surrogate.m
% 出力: data/results/surrogate_best.mat, data/results/phase4_*.mat/.csv
%       （図はsrc/utils/plot_phase4.pyで生成）
% 作成日: 2026-09-28

cd(fileparts(fileparts(mfilename('fullpath')))); % リポジトリルートに移動
run('src/params.m');
addpath('src/utils');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

%% 1. 生データ生成（8kHz: 物理モデルの計算レート、以後各レートへ間引き）
% 注記: REQUIREMENTS.md記載のチャープ振幅±2N*mは、本システムの慣性
% (J_m=1e-3, J_a=5e-3 kg*m^2)に対して過大であり、開ループ加振でtheta_aが
% 非現実的に大きくドリフトし（30秒で7rad超）、PTP運転領域(±1.3rad程度)と
% スケールが乖離してサロゲート学習が成立しなかった（Issue #4参照）。
% PTP運転時の最大トルク(Phase2実測 約0.17N*m)に整合する振幅0.2N*mに補正する。
fprintf('=== 生データ生成 (8kHz) ===\n');
chirp_raw = generate_chirp_data(0.5, 150, 30, 0.2, p);

targets_deg = [20, -30, 45, -60, 75];
move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
ptp_raw = cell(1,5);
for i = 1:5
    fprintf('PTPデータ生成中 (%d/5)\n', i);
    ptp_raw{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
end

% 位相・ロールアウト評価用: 30Hz純弦波 (2s, 振幅1N*m)
sine_raw = generate_chirp_data(30, 30, 2.0, 1.0, p);

%% 2. レート別学習・評価
rates_hz = [500, 1000, 2000, 8000];
rate_ids = {'rate_500hz','rate_1khz','rate_2khz','rate_8khz'};
plant_rate = 1/Ts_plant; % 8000Hz

N_SUBSAMPLE = 15000; % レート間で学習時間を揃えるためのサンプル上限
rng(123);

results = struct();
nets = struct();

% 30Hz真の位相（線形モデル, Phase1と同一のA,B,C,D）
A = [0,1,0,0; -k_s/J_m,-(b_m+b_s)/J_m,k_s/J_m,b_s/J_m; 0,0,0,1; k_s/J_a,b_s/J_a,-k_s/J_a,-(b_a+b_s)/J_a];
B = [0;1/J_m;0;0]; Cm = [0 0 1 0]; D = 0;
sys_true = ss(A,B,Cm,D);
[~,true_phase] = bode(sys_true, 2*pi*30);
true_phase_deg = squeeze(true_phase);

for ri = 1:numel(rates_hz)
    rate_hz = rates_hz(ri);
    rid = rate_ids{ri};
    fprintf('\n=== レート %dHz (%s) ===\n', rate_hz, rid);
    factor = round(plant_rate / rate_hz);

    chirp_ds = downsample_plant_data(chirp_raw, factor);
    ptp_ds = cell(1,5);
    for i = 1:5
        ptp_ds{i} = downsample_plant_data(ptp_raw{i}, factor);
    end
    sine_ds = downsample_plant_data(sine_raw, factor);

    % X, dX 構築
    [X_c, dX_c] = local_build_xy(chirp_ds);
    X_all = X_c; dX_all = dX_c;
    for i = 1:5
        [Xi, dXi] = local_build_xy(ptp_ds{i});
        X_all = [X_all; Xi]; %#ok<AGROW>
        dX_all = [dX_all; dXi]; %#ok<AGROW>
    end

    % サブサンプリング（レート間で学習時間を揃える）
    Nfull = size(X_all,1);
    Nsub = min(N_SUBSAMPLE, Nfull);
    idx_sub = randperm(Nfull, Nsub);
    X_all = X_all(idx_sub,:);
    dX_all = dX_all(idx_sub,:);

    % 80/10/10 分割
    idx = randperm(Nsub);
    nTrain = round(0.8*Nsub); nVal = round(0.1*Nsub);
    X_train = X_all(idx(1:nTrain),:); dX_train = dX_all(idx(1:nTrain),:);
    X_val   = X_all(idx(nTrain+1:nTrain+nVal),:); dX_val = dX_all(idx(nTrain+1:nTrain+nVal),:);
    X_test  = X_all(idx(nTrain+nVal+1:end),:); dX_test = dX_all(idx(nTrain+nVal+1:end),:);

    % 学習
    fprintf('学習中... (train=%d, val=%d, test=%d)\n', nTrain, nVal, Nsub-nTrain-nVal);
    [net, tr] = train_surrogate(X_train, dX_train, X_val, dX_val, X_test, dX_test, 200);
    nets.(rid) = net;

    % テスト精度評価
    dX_pred = net.predict(X_test);
    pos_rmse = sqrt(mean((dX_pred(:,1) - dX_test(:,1)).^2));       % rad (1ステップ位置誤差)
    vel_rmse = sqrt(mean((dX_pred(:,2) - dX_test(:,2)).^2));       % rad/s
    r2_pos = 1 - var(dX_pred(:,1)-dX_test(:,1)) / var(dX_test(:,1));

    % 30Hzロールアウト・位相評価
    x0 = [sine_ds.theta_a(1), sine_ds.omega_a(1), sine_ds.theta_m(1), sine_ds.tau(1)];
    x_traj = rollout_surrogate(net, x0, sine_ds.tau);
    n_skip = round(0.5 / (sine_ds.t(2)-sine_ds.t(1))); % 先頭0.5sは過渡として除外
    steady_idx = (n_skip+1):length(sine_ds.t);

    diverged = any(~isfinite(x_traj(:))) || max(abs(x_traj(:,1))) > 100;
    if diverged
        phase_err_deg = NaN;
        rollout_rmse = NaN;
    else
        surrogate_phase = fft_phase_at_freq(sine_ds.t(steady_idx), x_traj(steady_idx,1), 30);
        input_phase = fft_phase_at_freq(sine_ds.t(steady_idx), sine_ds.tau(steady_idx), 30);
        surrogate_phase_rel = surrogate_phase - input_phase;
        phase_diff = surrogate_phase_rel - true_phase_deg;
        phase_diff_wrapped = mod(phase_diff + 180, 360) - 180; % [-180,180]に正規化
        phase_err_deg = abs(phase_diff_wrapped);
        rollout_rmse = sqrt(mean((x_traj(steady_idx,1) - sine_ds.theta_a(steady_idx)).^2));
    end

    % train_perf/val_perfはtheta_a（位置）サブネットワークの学習曲線を代表値として記録
    results.(rid) = struct('rate_hz', rate_hz, 'pos_rmse', pos_rmse, 'vel_rmse', vel_rmse, ...
        'r2_pos', r2_pos, 'phase_err_deg', phase_err_deg, 'rollout_rmse', rollout_rmse, ...
        'diverged', diverged, 'train_perf', tr{1}.perf, 'val_perf', tr{1}.vperf, ...
        'dX_test_pos', dX_test(:,1), 'dX_pred_pos', dX_pred(:,1));

    fprintf('[%s] 位置RMSE=%.5f rad (%.3f mrad) 速度RMSE=%.5f rad/s (%.3f mrad/s) R2=%.4f 位相誤差=%.2f deg ロールアウトRMSE=%.5f diverged=%d\n', ...
        rid, pos_rmse, pos_rmse*1000, vel_rmse, vel_rmse*1000, r2_pos, phase_err_deg, rollout_rmse, diverged);

    % ロールアウト軌道をCSV出力（後でプロット用、代表として500Hz/8kHzのみ全長、他は間引き）
    writematrix([sine_ds.t(steady_idx), sine_ds.theta_a(steady_idx), x_traj(steady_idx,1)], ...
        sprintf('data/results/phase4_rollout_%s.csv', rid));
end

%% 3. 最良レートの選定・保存
rmse_list = arrayfun(@(i) results.(rate_ids{i}).pos_rmse, 1:numel(rate_ids));
valid = arrayfun(@(i) ~results.(rate_ids{i}).diverged, 1:numel(rate_ids));
rmse_list(~valid) = Inf;
[~, best_i] = min(rmse_list);
best_rate_id = rate_ids{best_i};
surrogate_net = nets.(best_rate_id); %#ok<NASGU>
best_rate_hz = rates_hz(best_i); %#ok<NASGU>
save('data/results/surrogate_best.mat', 'surrogate_net', 'best_rate_hz', 'best_rate_id');
fprintf('\n最良レート: %s\n', best_rate_id);

save('data/results/phase4_results.mat', 'results', 'rate_ids', 'rates_hz');

%% 4. 比較表・図用データのCSV出力
fid = fopen('data/results/phase4_comparison_table.csv', 'w');
fprintf(fid, 'rate_hz,pos_rmse_mrad,vel_rmse_mrad_s,r2_pos,phase_err_deg,rollout_rmse,diverged\n');
for ri = 1:numel(rate_ids)
    r = results.(rate_ids{ri});
    fprintf(fid, '%d,%.5f,%.5f,%.5f,%.3f,%.5f,%d\n', r.rate_hz, r.pos_rmse*1000, r.vel_rmse*1000, r.r2_pos, r.phase_err_deg, r.rollout_rmse, r.diverged);
end
fclose(fid);

% 学習曲線
max_len = max(arrayfun(@(i) length(results.(rate_ids{i}).train_perf), 1:numel(rate_ids)));
curve_mat = NaN(max_len, 2*numel(rate_ids));
for ri = 1:numel(rate_ids)
    r = results.(rate_ids{ri});
    curve_mat(1:length(r.train_perf), 2*ri-1) = r.train_perf(:);
    curve_mat(1:length(r.val_perf), 2*ri) = r.val_perf(:);
end
writematrix(curve_mat, 'data/results/phase4_training_curves.csv');

% 8kHz collapse: 予測 vs 真値の delta_theta_a 散布データ（8kHzと1kHzを比較用に出力）
writematrix([results.rate_8khz.dX_test_pos, results.rate_8khz.dX_pred_pos], 'data/results/phase4_8khz_scatter.csv');
writematrix([results.rate_1khz.dX_test_pos, results.rate_1khz.dX_pred_pos], 'data/results/phase4_1khz_scatter.csv');

disp('Phase4 学習・評価 完了');

%% ローカル関数
function [X, dX] = local_build_xy(d)
    X_full = [d.theta_a, d.omega_a, d.theta_m, d.tau];
    dX_full = [diff(X_full); zeros(1,4)];
    X = X_full(1:end-1,:);
    dX = dX_full(1:end-1,:);
end
