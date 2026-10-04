%% run_phase4c_rate_sweep.m
% 目的: Issue #4の追加検証。500Hz-1kHz間を600/700/800/900Hzで細かく走査し、
%       30Hz共振に対するオーバーサンプリング比と位相誤差の崩れ目を特定する。
%       （8kHzの整数約数でないレートはresampleによる有理数比リサンプリングで対応）
% 入力: src/params.m（チャープ/PTP生データはワークスペースに残っている前提、
%       無ければsrc/run_phase4_markov_complete.mの手順1を先に実行すること）
% 出力: data/results/phase4c_rate_sweep.csv
%       （図はsrc/utils/plot_phase4c.pyで生成）
% 作成日: 2026-10-04

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils'); addpath('src');

p = struct('J_m',J_m,'J_a',J_a,'k_s',k_s,'b_m',b_m,'b_a',b_a,'b_s',b_s, ...
    'Ts_plant',Ts_plant,'Ts_pos',Ts_pos);

if ~exist('chirp_tamed','var')
    chirp_tamed = generate_chirp_data(0.5, 150, 30, 0.2, p);
end
if ~exist('chirp_wide','var')
    chirp_wide = generate_chirp_data(0.5, 150, 15, 3.0, p);
end
if ~exist('ptp_raw','var')
    targets_deg = [20, -30, 45, -60, 75];
    move_times  = [0.3, 0.4, 0.5, 0.6, 0.8];
    ptp_raw = cell(1,5);
    for i = 1:5
        ptp_raw{i} = generate_ptp_data(targets_deg(i), move_times(i), 20, p);
    end
end
if ~exist('sine_raw','var')
    sine_raw = generate_chirp_data(30, 30, 2.0, 1.0, p);
end

plant_rate = 1/Ts_plant;
rates_hz = [500, 600, 700, 800, 900, 1000];

A = [0,1,0,0; -k_s/J_m,-(b_m+b_s)/J_m,k_s/J_m,b_s/J_m; 0,0,0,1; k_s/J_a,b_s/J_a,-k_s/J_a,-(b_a+b_s)/J_a];
B = [0;1/J_m;0;0]; Cm = [0 0 1 0]; D = 0;
sys_true = ss(A,B,Cm,D);
[~,true_phase] = bode(sys_true, 2*pi*30);
true_phase_deg = squeeze(true_phase);

results_sweep = struct();

for ri = 1:numel(rates_hz)
    rate_hz = rates_hz(ri);
    rid = sprintf('rate_%dhz', rate_hz);
    Ts_r = 1/rate_hz;
    fprintf('\n=== レート %dHz (30Hzとの比=%.2fx) ===\n', rate_hz, rate_hz/30);

    trajs = [{downsample_plant_data_rational(chirp_tamed, rate_hz, plant_rate)}, ...
             {downsample_plant_data_rational(chirp_wide, rate_hz, plant_rate)}];
    for i = 1:5
        trajs{end+1} = downsample_plant_data_rational(ptp_raw{i}, rate_hz, plant_rate); %#ok<SAGROW>
    end
    for ti = 1:numel(trajs)
        trajs{ti}.spring_defl = trajs{ti}.theta_m - trajs{ti}.theta_a;
    end
    sine_ds = downsample_plant_data_rational(sine_raw, rate_hz, plant_rate);
    sine_ds.spring_defl = sine_ds.theta_m - sine_ds.theta_a;

    X_all = []; dSpring_all = []; dOmega_all = []; dThetaA_all = [];
    for ti = 1:numel(trajs)
        tr = trajs{ti};
        N = length(tr.t);
        rate_fd = [NaN; diff(tr.spring_defl)/Ts_r];
        X = [tr.spring_defl(2:N-1), rate_fd(2:N-1), tr.omega_a(2:N-1), tr.tau(2:N-1)];
        dSpring = tr.spring_defl(3:N) - tr.spring_defl(2:N-1);
        dOmega  = tr.omega_a(3:N) - tr.omega_a(2:N-1);
        dThetaA = tr.theta_a(3:N) - tr.theta_a(2:N-1);
        X_all = [X_all; X]; %#ok<AGROW>
        dSpring_all = [dSpring_all; dSpring]; %#ok<AGROW>
        dOmega_all = [dOmega_all; dOmega]; %#ok<AGROW>
        dThetaA_all = [dThetaA_all; dThetaA]; %#ok<AGROW>
    end

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
    true_dThetaA = dThetaA_all(testIdx);
    omega_a_test = X_all(testIdx,3);
    pred_dThetaA = (omega_a_test + 0.5*pred_dOmega) * Ts_r;

    pos_rmse = sqrt(mean((pred_dThetaA - true_dThetaA).^2));
    vel_rmse = sqrt(mean((pred_dOmega - dOmega_all(testIdx)).^2));
    r2_pos = 1 - var(pred_dThetaA - true_dThetaA) / var(true_dThetaA);

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

    results_sweep.(rid) = struct('rate_hz', rate_hz, 'oversampling_ratio', rate_hz/30, ...
        'pos_rmse', pos_rmse, 'vel_rmse', vel_rmse, 'r2_pos', r2_pos, ...
        'phase_err_deg', phase_err_deg, 'rollout_rmse', rollout_rmse);

    fprintf('[%s] 位置RMSE=%.4f mrad 速度RMSE=%.4f mrad/s R2_pos=%.4f 位相誤差=%.2f deg\n', ...
        rid, pos_rmse*1000, vel_rmse*1000, r2_pos, phase_err_deg);
end

fid = fopen('data/results/phase4c_rate_sweep.csv', 'w');
fprintf(fid, 'rate_hz,oversampling_ratio,pos_rmse_mrad,vel_rmse_mrad_s,r2_pos,phase_err_deg,rollout_rmse\n');
for ri = 1:numel(rates_hz)
    r = results_sweep.(sprintf('rate_%dhz', rates_hz(ri)));
    fprintf(fid, '%d,%.4f,%.6f,%.6f,%.5f,%.3f,%.6f\n', r.rate_hz, r.oversampling_ratio, r.pos_rmse*1000, r.vel_rmse*1000, r.r2_pos, r.phase_err_deg, r.rollout_rmse);
end
fclose(fid);

disp('Phase4c レート走査 完了');
