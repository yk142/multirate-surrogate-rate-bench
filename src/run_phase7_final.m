%% run_phase7_final.m
% 目的: Phase 7 最終検証。Phase6で得た最適ゲインを物理モデルに適用し、
%       初期ゲインとの比較・ノッチフィルタ有無の動作確認を行う。
% 入力: src/params.m, models/controller/cascade_controller.slx,
%       data/results/phase6_results.mat
% 出力: data/results/phase7_*.csv
%       （図はsrc/utils/plot_phase7.pyで生成）
% 作成日: 2026-10-03

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils');

load('data/results/phase6_results.mat', 'x_opt_ga', 'cost_opt_ga', 'x_opt_fmin', 'cost_opt_fmin');

if cost_opt_fmin < cost_opt_ga
    gains_opt = x_opt_fmin;
    opt_label = 'fminsearch';
else
    gains_opt = x_opt_ga;
    opt_label = 'GA';
end
gains_init = [Kp_pos, Kp_vel, Ki_vel];
fprintf('初期ゲイン: Kp_pos=%.2f Kp_vel=%.3f Ki_vel=%.3f\n', gains_init);
fprintf('最適ゲイン(%s): Kp_pos=%.2f Kp_vel=%.3f Ki_vel=%.3f\n', opt_label, gains_opt);

[theta_ref, t_ref] = ptp_profile(0, pi/4, 2.0, Ts_pos, 0.5);
ptp_ref_ts = timeseries(theta_ref, t_ref);
assignin('base', 'ptp_ref_ts', ptp_ref_ts);

load_system('models/controller/cascade_controller.slx');

function [t, theta_a, tau] = run_physical(gains, notch_on)
    assignin('base', 'Kp_pos', gains(1));
    assignin('base', 'Kp_vel', gains(2));
    assignin('base', 'Ki_vel', gains(3));
    assignin('base', 'enable_notch', notch_on);
    simOut = sim('cascade_controller');
    ds = simOut.yout;
    t = ds{1}.Values.Time;
    theta_a = ds{1}.Values.Data;
    tau_raw = ds{3}.Values.Data;
    tau = interp1(ds{3}.Values.Time, tau_raw, t, 'previous', 'extrap');
end

%% 1. 初期ゲイン vs 最適ゲイン（ノッチON, 物理モデル）
fprintf('\n初期ゲインでシミュレーション中...\n');
[t_init, theta_init, tau_init] = run_physical(gains_init, true);
fprintf('最適ゲインでシミュレーション中...\n');
[t_opt, theta_opt, tau_opt] = run_physical(gains_opt, true);

writematrix([t_init, theta_init, tau_init], 'data/results/phase7_initial_gains.csv');
writematrix([t_opt, theta_opt, tau_opt], 'data/results/phase7_optimal_gains.csv');

%% 2. 最適ゲインでのノッチフィルタ有無確認
fprintf('最適ゲイン・ノッチOFFでシミュレーション中...\n');
[t_opt_nf, theta_opt_nf, ~] = run_physical(gains_opt, false);
writematrix([t_opt_nf, theta_opt_nf], 'data/results/phase7_optimal_notch_off.csv');

close_system('cascade_controller', 0);

%% 3. 性能指標の算出
theta_target = pi/4;
function m = calc_metrics(theta, t, target)
    m.settling = calc_settling_time(theta, target, t, 0.02);
    if isnan(m.settling), m.settling = t(end); end
    m.overshoot = max(0, max(theta)-target)/target*100;
    theta_ref_local = target * ones(size(theta)); % 整定後の追従誤差相当(簡易)
    m.final_err_pct = abs(theta(end)-target)/target*100;
end

m_init = calc_metrics(theta_init, t_init, theta_target);
m_opt = calc_metrics(theta_opt, t_opt, theta_target);
m_opt_nf = calc_metrics(theta_opt_nf, t_opt_nf, theta_target);

fprintf('\n=== 性能指標 ===\n');
fprintf('初期ゲイン:         整定時間=%.4fs オーバーシュート=%.2f%% 終端誤差=%.4f%%\n', m_init.settling, m_init.overshoot, m_init.final_err_pct);
fprintf('最適ゲイン:         整定時間=%.4fs オーバーシュート=%.2f%% 終端誤差=%.4f%%\n', m_opt.settling, m_opt.overshoot, m_opt.final_err_pct);
fprintf('最適ゲイン(ノッチOFF): 整定時間=%.4fs オーバーシュート=%.2f%% 終端誤差=%.4f%%\n', m_opt_nf.settling, m_opt_nf.overshoot, m_opt_nf.final_err_pct);

fid = fopen('data/results/phase7_metrics.csv', 'w');
fprintf(fid, 'config,settling_time,overshoot_pct,final_err_pct\n');
fprintf(fid, '初期ゲイン,%.6f,%.6f,%.6f\n', m_init.settling, m_init.overshoot, m_init.final_err_pct);
fprintf(fid, '最適ゲイン,%.6f,%.6f,%.6f\n', m_opt.settling, m_opt.overshoot, m_opt.final_err_pct);
fprintf(fid, '最適ゲイン(ノッチOFF),%.6f,%.6f,%.6f\n', m_opt_nf.settling, m_opt_nf.overshoot, m_opt_nf.final_err_pct);
fclose(fid);

save('data/results/phase7_results.mat', 'gains_init', 'gains_opt', 'opt_label', 'm_init', 'm_opt', 'm_opt_nf');

disp('Phase7 最終検証 完了');
