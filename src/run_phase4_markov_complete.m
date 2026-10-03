%% run_phase4_markov_complete.m
% 目的: Phase4のレート比較(500Hz/1kHz/2kHz/8kHz)を、Issue #5で発見した
%       マルコフ完備な相対状態表現 [spring_defl, spring_defl_rate, omega_a, tau]
%       を用いて再実行し、真のサンプリングレート効果を交絡要因なしで検証する。
%
% 背景: Phase4(x=[theta_a,omega_a,theta_m,tau])では、omega_m相当の情報欠落
%       により速度RMSE・位相誤差・ロールアウト安定性の評価が全レート共通の
%       交絡を受けていた（Issue #4追記, Issue #5参照）。本スクリプトでは
%       この欠陥を修正した特徴量で、真の系が線形であることを利用し
%       線形回帰でレート別に学習・評価する（NNではなく線形回帰を用いるのは、
%       学習アルゴリズムの最適化の巧拙による交絡も排除し、純粋にレートの
%       影響だけを見るため）。
%
% 出力: data/results/phase4b_comparison_table.csv
%       data/results/phase4b_rollout_rate_*.csv
%       （図はsrc/utils/plot_phase4b.pyで生成）
% 作成日: 2026-10-03

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils'); addpath('src');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

%% 1. 生データ生成（8kHz、Issue #5と同一の2種チャープ+5PTP）
fprintf('=== 生データ生成 (8kHz) ===\n');
chirp_tamed = generate_chirp_data(0.5, 150, 30, 0.2, p);
chirp_wide  = generate_chirp_data(0.5, 150, 15, 3.0, p);

targets_deg = [20, -30, 45, -60, 75];
move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
ptp_raw = cell(1,5);
for i = 1:5
    fprintf('PTPデータ生成中 (%d/5)\n', i);
    ptp_raw{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
end

sine_raw = generate_chirp_data(30, 30, 2.0, 1.0, p); % 位相・ロールアウト評価用

%% 2. レート別学習・評価
rates_hz = [500, 1000, 2000, 8000];
rate_ids = {'rate_500hz','rate_1khz','rate_2khz','rate_8khz'};
plant_rate = 1/Ts_plant;

A = [0,1,0,0; -k_s/J_m,-(b_m+b_s)/J_m,k_s/J_m,b_s/J_m; 0,0,0,1; k_s/J_a,b_s/J_a,-k_s/J_a,-(b_a+b_s)/J_a];
B = [0;1/J_m;0;0]; Cm = [0 0 1 0]; D = 0;
sys_true = ss(A,B,Cm,D);
[~,true_phase] = bode(sys_true, 2*pi*30);
true_phase_deg = squeeze(true_phase);

results = struct();

for ri = 1:numel(rates_hz)
    rate_hz = rates_hz(ri);
    rid = rate_ids{ri};
    Ts_r = 1/rate_hz;
    fprintf('\n=== レート %dHz (%s) ===\n', rate_hz, rid);
    factor = round(plant_rate / rate_hz);

    trajs = [{downsample_plant_data(chirp_tamed, factor)}, {downsample_plant_data(chirp_wide, factor)}];
    for i = 1:5
        trajs{end+1} = downsample_plant_data(ptp_raw{i}, factor); %#ok<SAGROW>
    end
    for ti = 1:numel(trajs)
        trajs{ti}.spring_defl = trajs{ti}.theta_m - trajs{ti}.theta_a;
    end
    sine_ds = downsample_plant_data(sine_raw, factor);
    sine_ds.spring_defl = sine_ds.theta_m - sine_ds.theta_a;

    % 特徴量収集: X=[spring_defl, spring_defl_rate, omega_a, tau]
    % -> [dSpring, dOmega, dTheta_a(真値)]
    X_all = []; dSpring_all = []; dOmega_all = []; dThetaA_all = [];
    for ti = 1:numel(trajs)
        tr = trajs{ti};
        N = length(tr.t);
        rate_fd = [NaN; diff(tr.spring_defl)/Ts_r];
        % インデックス s=2..N-1 を使用 (rate_fdが計算できs+1も存在する範囲)
        X = [tr.spring_defl(2:N-1), rate_fd(2:N-1), tr.omega_a(2:N-1), tr.tau(2:N-1)];
        dSpring = tr.spring_defl(3:N) - tr.spring_defl(2:N-1);
        dOmega  = tr.omega_a(3:N) - tr.omega_a(2:N-1);
        dThetaA = tr.theta_a(3:N) - tr.theta_a(2:N-1);
        X_all = [X_all; X]; %#ok<AGROW>
        dSpring_all = [dSpring_all; dSpring]; %#ok<AGROW>
        dOmega_all = [dOmega_all; dOmega]; %#ok<AGROW>
        dThetaA_all = [dThetaA_all; dThetaA]; %#ok<AGROW>
    end

    % 80/10/10分割
    rng(42);
    Nfull = size(X_all,1);
    idx = randperm(Nfull);
    nTrain = round(0.8*Nfull); nVal = round(0.1*Nfull);
    trainIdx = idx(1:nTrain); testIdx = idx(nTrain+nVal+1:end);

    Xreg = [X_all(trainIdx,:), ones(numel(trainIdx),1)];
    coef_spring = Xreg \ dSpring_all(trainIdx);
    coef_omega  = Xreg \ dOmega_all(trainIdx);

    Xtest = [X_all(testIdx,:), ones(numel(testIdx),1)];
    pred_dSpring = Xtest * coef_spring;
    pred_dOmega  = Xtest * coef_omega;
    true_dSpring = dSpring_all(testIdx);
    true_dOmega  = dOmega_all(testIdx);
    true_dThetaA = dThetaA_all(testIdx);

    omega_a_test = X_all(testIdx,3);
    pred_dThetaA = (omega_a_test + 0.5*pred_dOmega) * Ts_r; % 台形積分

    pos_rmse = sqrt(mean((pred_dThetaA - true_dThetaA).^2));
    vel_rmse = sqrt(mean((pred_dOmega - true_dOmega).^2));
    r2_pos = 1 - var(pred_dThetaA - true_dThetaA) / var(true_dThetaA);
    r2_spring = 1 - var(pred_dSpring - true_dSpring) / var(true_dSpring);
    r2_omega  = 1 - var(pred_dOmega - true_dOmega) / var(true_dOmega);

    % 30Hzロールアウト・位相評価
    x0 = [sine_ds.spring_defl(2), (sine_ds.spring_defl(2)-sine_ds.spring_defl(1))/Ts_r, sine_ds.omega_a(2), sine_ds.tau(2)];
    theta_a0 = sine_ds.theta_a(2);
    N_roll = length(sine_ds.t) - 2;
    traj_theta_a = zeros(N_roll,1); traj_theta_a(1) = theta_a0;
    x = x0;
    diverged = false;
    for k=1:N_roll-1
        feat = [x, 1];
        d_spring = feat * coef_spring;
        d_omega = feat * coef_omega;
        theta_a_next = traj_theta_a(k) + (x(3) + 0.5*d_omega)*Ts_r;
        spring_next = x(1) + d_spring;
        omega_next = x(3) + d_omega;
        rate_next = d_spring / Ts_r;
        tau_next = sine_ds.tau(min(k+2, length(sine_ds.tau)));
        x = [spring_next, rate_next, omega_next, tau_next];
        traj_theta_a(k+1) = theta_a_next;
        if ~isfinite(theta_a_next) || abs(theta_a_next) > 100
            diverged = true;
            traj_theta_a = traj_theta_a(1:k+1);
            break;
        end
    end
    n_valid = length(traj_theta_a);
    true_theta_a_roll = sine_ds.theta_a(2:1+n_valid);
    rollout_rmse = sqrt(mean((traj_theta_a - true_theta_a_roll).^2));

    n_skip = round(0.5/Ts_r);
    if diverged || n_valid <= n_skip + 10
        phase_err_deg = NaN;
    else
        t_roll = sine_ds.t(2:1+n_valid);
        tau_roll = sine_ds.tau(3:2+n_valid);
        surrogate_phase = fft_phase_at_freq(t_roll(n_skip+1:end), traj_theta_a(n_skip+1:end), 30);
        input_phase = fft_phase_at_freq(t_roll(n_skip+1:end), tau_roll(n_skip+1:end), 30);
        phase_diff = (surrogate_phase - input_phase) - true_phase_deg;
        phase_err_deg = abs(mod(phase_diff+180,360)-180);
    end

    amp_check = max(abs(true_theta_a_roll));
    practically_diverged = diverged || (max(abs(traj_theta_a)) > 10*amp_check);

    results.(rid) = struct('rate_hz', rate_hz, 'pos_rmse', pos_rmse, 'vel_rmse', vel_rmse, ...
        'r2_pos', r2_pos, 'r2_spring', r2_spring, 'r2_omega', r2_omega, ...
        'phase_err_deg', phase_err_deg, 'rollout_rmse', rollout_rmse, 'diverged', practically_diverged);

    fprintf('[%s] 位置RMSE=%.6f rad (%.4f mrad) 速度RMSE=%.6f rad/s (%.4f mrad/s) R2_pos=%.4f R2_spring=%.4f 位相誤差=%.2f deg ロールアウトRMSE=%.5f diverged=%d\n', ...
        rid, pos_rmse, pos_rmse*1000, vel_rmse, vel_rmse*1000, r2_pos, r2_spring, phase_err_deg, rollout_rmse, practically_diverged);

    writematrix([sine_ds.t(2:1+n_valid), true_theta_a_roll, traj_theta_a], ...
        sprintf('data/results/phase4b_rollout_%s.csv', rid));
end

%% 3. 比較表の出力
fid = fopen('data/results/phase4b_comparison_table.csv', 'w');
fprintf(fid, 'rate_hz,pos_rmse_mrad,vel_rmse_mrad_s,r2_pos,r2_spring,phase_err_deg,rollout_rmse,diverged\n');
for ri = 1:numel(rate_ids)
    r = results.(rate_ids{ri});
    fprintf(fid, '%d,%.6f,%.6f,%.5f,%.5f,%.3f,%.6f,%d\n', r.rate_hz, r.pos_rmse*1000, r.vel_rmse*1000, r.r2_pos, r.r2_spring, r.phase_err_deg, r.rollout_rmse, r.diverged);
end
fclose(fid);

save('data/results/phase4b_results.mat', 'results', 'rate_ids', 'rates_hz');
disp('Phase4(マルコフ完備版) 再比較 完了');
