%% run_phase3_data.m
% 目的: Phase 3 学習データ生成（チャープ+PTP軌道）、1kHzダウンサンプリング、
%       学習/検証/テスト分割、統計確認用データのCSV出力
% 入力: src/params.m, models/controller/cascade_controller.slx
% 出力: data/training/surrogate_train_data.mat
%       data/results/phase3_overview.csv, phase3_spectrum.csv
% 作成日: 2026-09-28

cd(fileparts(fileparts(mfilename('fullpath')))); % リポジトリルートに移動
run('src/params.m');
addpath('src/utils');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

%% 1. チャープ信号 (0.5-150Hz, ±2N*m, 30s)
fprintf('チャープデータ生成中...\n');
chirp_data = generate_chirp_data(0.5, 150, 30, 2.0, p);

%% 2. PTP軌道 (5本, 20s each)
targets_deg = [20, -30, 45, -60, 75];
move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
ptp_datasets = cell(1,5);
for i = 1:5
    fprintf('PTPデータ生成中 (%d/5): target=%.0fdeg, move_time=%.2fs\n', i, targets_deg(i), move_times(i));
    ptp_datasets{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
end

%% 3. 1kHzへダウンサンプリング（アンチエイリアスフィルタ: decimateのIIRフィルタを使用）
ds_factor = round(Ts_vel / Ts_plant); % 8kHz -> 1kHz : factor 8

chirp_ds = downsample_data(chirp_data, ds_factor);
ptp_ds = cell(1,5);
for i = 1:5
    ptp_ds{i} = downsample_data(ptp_datasets{i}, ds_factor);
end

%% 4. X, delta_X の構築 [theta_a, omega_a, theta_m, tau]
[X_chirp, dX_chirp] = build_xy(chirp_ds);
X_all = X_chirp; dX_all = dX_chirp;
src_label = repmat({'chirp'}, size(X_chirp,1), 1);
for i = 1:5
    [Xi, dXi] = build_xy(ptp_ds{i});
    X_all = [X_all; Xi]; %#ok<AGROW>
    dX_all = [dX_all; dXi]; %#ok<AGROW>
    src_label = [src_label; repmat({sprintf('ptp%d',i)}, size(Xi,1), 1)]; %#ok<AGROW>
end

%% 5. 学習/検証/テスト分割 (80/10/10, ランダムシャッフル)
rng(42);
N = size(X_all,1);
idx = randperm(N);
n_train = round(0.8*N);
n_val   = round(0.1*N);

train_idx = idx(1:n_train);
val_idx   = idx(n_train+1:n_train+n_val);
test_idx  = idx(n_train+n_val+1:end);

X_train = X_all(train_idx,:); dX_train = dX_all(train_idx,:);
X_val   = X_all(val_idx,:);   dX_val   = dX_all(val_idx,:);
X_test  = X_all(test_idx,:);  dX_test  = dX_all(test_idx,:);

save('data/training/surrogate_train_data.mat', ...
    'X_train','dX_train','X_val','dX_val','X_test','dX_test', ...
    'X_all','dX_all','src_label','Ts_vel');

fprintf('データ生成完了: 全%dサンプル (train=%d, val=%d, test=%d)\n', N, numel(train_idx), numel(val_idx), numel(test_idx));

%% 6. 統計確認用データの出力（overview: chirp先頭5秒, PTP1本目全体）
overview_t   = [chirp_ds.t(1:5000); ptp_ds{1}.t + chirp_ds.t(5000) + Ts_vel];
overview_tau = [chirp_ds.tau(1:5000); ptp_ds{1}.tau];
overview_th  = [chirp_ds.theta_a(1:5000); ptp_ds{1}.theta_a];
overview_om  = [chirp_ds.omega_a(1:5000); ptp_ds{1}.omega_a];
writematrix([overview_t, overview_tau, overview_th, overview_om], 'data/results/phase3_overview.csv');

% 入力トルクのパワースペクトル（チャープ, 0-200Hz）
Fs = 1/Ts_vel;
tau_sig = chirp_ds.tau - mean(chirp_ds.tau);
Nfft = 2^nextpow2(length(tau_sig));
Yf = fft(tau_sig, Nfft);
Pxx = abs(Yf(1:Nfft/2+1)).^2 / (Fs*Nfft);
freq = Fs*(0:(Nfft/2))/Nfft;
mask = freq <= 200;
writematrix([freq(mask)', Pxx(mask)], 'data/results/phase3_spectrum.csv');

disp('Phase3データ生成・統計出力 完了');

%% ローカル関数
function s = downsample_data(d, factor)
    s.t       = d.t(1:factor:end);
    s.tau     = decimate(d.tau, factor);
    s.theta_a = decimate(d.theta_a, factor);
    s.omega_a = decimate(d.omega_a, factor);
    s.theta_m = decimate(d.theta_m, factor);
    n = min([length(s.t), length(s.tau), length(s.theta_a), length(s.omega_a), length(s.theta_m)]);
    s.t = s.t(1:n); s.tau = s.tau(1:n); s.theta_a = s.theta_a(1:n);
    s.omega_a = s.omega_a(1:n); s.theta_m = s.theta_m(1:n);
end

function [X, dX] = build_xy(d)
    X_full = [d.theta_a, d.omega_a, d.theta_m, d.tau];
    dX_full = [diff(X_full); zeros(1,4)];
    X = X_full(1:end-1,:);
    dX = dX_full(1:end-1,:);
end
