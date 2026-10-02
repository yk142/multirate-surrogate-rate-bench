%% run_phase5_sim.m
% 目的: Phase 5 サロゲートによる閉ループ代替検証
%       物理モデルとサロゲート(2kHz)を同一ゲイン・同一PTP指令で閉ループ実行し比較
% 入力: src/params.m, models/controller/cascade_controller.slx,
%       models/surrogate/cascade_controller_surrogate.slx
% 出力: data/results/phase5_physical.csv, phase5_surrogate.csv
%       （図はsrc/utils/plot_phase5.pyで生成）
% 作成日: 2026-10-02

cd(fileparts(fileparts(mfilename('fullpath')))); % リポジトリルートに移動
run('src/params.m');
addpath('src/utils');
addpath('src/generated');

[theta_ref, t_ref] = ptp_profile(0, pi/4, 2.0, Ts_pos, 0.5);
ptp_ref_ts = timeseries(theta_ref, t_ref);
assignin('base', 'ptp_ref_ts', ptp_ref_ts);
enable_notch = true; %#ok<NASGU>

%% 1. 物理モデル（Phase2と同一構成）
fprintf('物理モデルで閉ループシミュレーション実行中...\n');
load_system('models/controller/cascade_controller.slx');
simOut_phys = sim('cascade_controller');
close_system('cascade_controller', 0);
ds_phys = simOut_phys.yout;
t_phys = ds_phys{1}.Values.Time;
theta_a_phys = ds_phys{1}.Values.Data;
writematrix([t_phys, theta_a_phys], 'data/results/phase5_physical.csv');

%% 2. サロゲートモデル（2kHz, Phase4最良レート）
fprintf('サロゲートモデルで閉ループシミュレーション実行中...\n');
load_system('models/surrogate/cascade_controller_surrogate.slx');
simOut_sur = sim('cascade_controller_surrogate');
close_system('cascade_controller_surrogate', 0);
ds_sur = simOut_sur.yout;
t_sur = ds_sur{1}.Values.Time;
theta_a_sur = squeeze(ds_sur{1}.Values.Data);
writematrix([t_sur, theta_a_sur], 'data/results/phase5_surrogate.csv');

%% 3. 誤差評価（サロゲートの時刻グリッドに物理モデルを補間して比較）
theta_a_phys_on_sur = interp1(t_phys, theta_a_phys, t_sur, 'linear', 'extrap');
err = theta_a_phys_on_sur - theta_a_sur;
theta_target = pi/4;
rel_err_pct = abs(err) / theta_target * 100;

fprintf('最終位置: 物理=%.4f rad, サロゲート=%.4f rad (目標=%.4f rad)\n', theta_a_phys_on_sur(end), theta_a_sur(end), theta_target);
fprintf('最大誤差=%.4f rad (%.2f%%), 終端誤差=%.4f rad (%.2f%%)\n', ...
    max(abs(err)), max(rel_err_pct), abs(err(end)), rel_err_pct(end));
fprintf('5%%基準: %s\n', ternary_local(max(rel_err_pct) < 5, '合格', '不合格'));

writematrix([t_sur, theta_a_phys_on_sur, theta_a_sur, err, rel_err_pct], 'data/results/phase5_error.csv');

disp('Phase5 シミュレーション完了');

function out = ternary_local(cond, a, b)
    if cond, out = a; else, out = b; end
end
