%% run_phase2_sim.m
% 目的: Phase 2 カスケード制御器のPTP応答シミュレーションとノッチフィルタ比較
% 入力: models/controller/cascade_controller.slx, src/params.m
% 出力: figures/phase2_ptp_response.png, figures/phase2_notch_comparison.png
%       整定時間・オーバーシュートの計算結果をコマンドウィンドウに表示
% 作成日: 2026-09-28

cd(fileparts(fileparts(mfilename('fullpath')))); % リポジトリルートに移動
run('src/params.m');
addpath('src/utils');

[theta_ref, t_ref] = ptp_profile(0, pi/4, 2.0, Ts_pos, 0.5);
ptp_ref_ts = timeseries(theta_ref, t_ref);
assignin('base', 'ptp_ref_ts', ptp_ref_ts);

modelPath = fullfile('models', 'controller', 'cascade_controller.slx');
load_system(modelPath);

results = struct();
for enable_notch_val = [true, false]
    enable_notch = enable_notch_val; %#ok<NASGU>
    simOut = sim('cascade_controller');
    ds = simOut.yout;
    key = ternary(enable_notch_val, 'notch_on', 'notch_off');
    t_common = ds{1}.Values.Time;
    results.(key).t        = t_common;
    results.(key).theta_a  = ds{1}.Values.Data;
    results.(key).omega_a  = ds{2}.Values.Data;
    results.(key).tau      = interp1(ds{3}.Values.Time, ds{3}.Values.Data, t_common, 'previous', 'extrap');
    results.(key).theta_ref = interp1(ds{4}.Values.Time, ds{4}.Values.Data, t_common, 'previous', 'extrap');
end
close_system('cascade_controller', 0);

theta_target = pi/4;
for f = {'notch_on','notch_off'}
    key = f{1};
    r = results.(key);
    ts = calc_settling_time(r.theta_a, theta_target, r.t, 0.02);
    ov = max(0, max(r.theta_a) - theta_target) / theta_target * 100;
    fprintf('[%s] 整定時間(2%%)=%.4f s, オーバーシュート=%.2f %%\n', key, ts, ov);
    results.(key).settling_time = ts;
    results.(key).overshoot_pct = ov;
end

%% 出力図1: 位置・速度・トルク・参照のタイムライン（4段組、notch_on使用）
r = results.notch_on;
fig1 = figure('Visible','off','Position',[100 100 800 900]);
subplot(4,1,1);
plot(r.t, r.theta_ref, '--', r.t, r.theta_a, '-', 'LineWidth', 1.2);
grid on; ylabel('\theta [rad]'); legend('参照','応答','Location','southeast');
title('Phase 2: PTP応答（位置・速度・トルク・追従誤差）');

subplot(4,1,2);
plot(r.t, r.omega_a, 'LineWidth', 1.2);
grid on; ylabel('\omega_a [rad/s]');

subplot(4,1,3);
plot(r.t, r.tau, 'LineWidth', 1.2);
grid on; ylabel('\tau [N\cdotm]');

subplot(4,1,4);
plot(r.t, r.theta_ref - r.theta_a, 'LineWidth', 1.2);
grid on; ylabel('誤差 [rad]'); xlabel('時間 [s]');

save_figure(fig1, 'phase2_ptp_response.png');

%% 出力図2: ノッチフィルタあり/なしの比較
fig2 = figure('Visible','off','Position',[100 100 800 500]);
subplot(2,1,1);
plot(results.notch_on.t, results.notch_on.theta_a, '-', ...
     results.notch_off.t, results.notch_off.theta_a, '--', 'LineWidth', 1.2);
grid on; ylabel('\theta_a [rad]');
legend('ノッチON','ノッチOFF','Location','southeast');
title('ノッチフィルタ有無による応答比較');

subplot(2,1,2);
plot(results.notch_on.t, results.notch_on.tau, '-', ...
     results.notch_off.t, results.notch_off.tau, '--', 'LineWidth', 1.2);
grid on; ylabel('\tau [N\cdotm]'); xlabel('時間 [s]');

save_figure(fig2, 'phase2_notch_comparison.png');

save(fullfile('data','results','phase2_results.mat'), 'results');

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
