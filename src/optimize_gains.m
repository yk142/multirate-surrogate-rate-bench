%% optimize_gains.m
% 目的: Phase 6 ゲイン最適化シミュレーション。サロゲート(線形モデル, Phase5で
%       閉ループ誤差0%を達成)をプラントとして、ga(遺伝的アルゴリズム)と
%       fminsearchで制御ゲイン[Kp_pos, Kp_vel, Ki_vel]を最適化し比較する。
% 入力: src/params.m, data/results/surrogate_best.mat
% 出力: data/results/phase6_results.mat
%       data/results/phase6_convergence_ga.csv, phase6_convergence_fminsearch.csv
%       data/results/phase6_candidates.csv
%       （図はsrc/utils/plot_phase6.pyで生成）
% 作成日: 2026-10-03

cd(fileparts(fileparts(mfilename('fullpath'))));
run('src/params.m');
addpath('src/utils');

load('data/results/surrogate_best.mat', 'surrogate_coef_spring', 'surrogate_coef_omega', 'surrogate_Ts');
surrogate_net = struct('coef_spring', surrogate_coef_spring, 'coef_omega', surrogate_coef_omega, 'Ts', surrogate_Ts);

params_opt = struct('Ts_pos', Ts_pos, 'tau_max', tau_max, 'f_notch', f_notch, 'depth_dB', depth_dB, ...
    'Q_notch', Q_notch, 'enable_notch', true, 'sim_time', 2.0, 'theta_target', pi/4, 'move_time', 0.5);

costFcn = @(x) eval_cost(x, surrogate_net, params_opt);

lb = gain_lb; % [10, 0.1, 0.5]
ub = gain_ub; % [200, 5.0, 50.0]
x0 = [Kp_pos, Kp_vel, Ki_vel];

%% 1. 手法1: ga（遺伝的アルゴリズム）
fprintf('=== GA最適化 ===\n');
ga_history = [];
gaOutFcn = @(options,state,flag) local_ga_log(options,state,flag);
gaOptions = optimoptions('ga', 'MaxGenerations', 100, 'PopulationSize', 50, ...
    'OutputFcn', gaOutFcn, 'Display', 'iter');
[x_opt_ga, cost_opt_ga] = ga(costFcn, 3, [], [], [], [], lb, ub, [], gaOptions);
fprintf('GA最適解: Kp_pos=%.3f Kp_vel=%.3f Ki_vel=%.3f, cost=%.6f\n', x_opt_ga, cost_opt_ga);

writematrix(ga_history, 'data/results/phase6_convergence_ga.csv');

%% 2. 手法2: fminsearch（比較用, 初期値=元のゲイン）
fprintf('\n=== fminsearch最適化 ===\n');
fmin_history = [];
fminOutFcn = @(x,optimValues,state) local_fmin_log(x,optimValues,state);
fminOptions = optimset('OutputFcn', fminOutFcn, 'MaxIter', 300, 'MaxFunEvals', 1000, 'Display', 'iter');
[x_opt_fmin, cost_opt_fmin] = fminsearch(costFcn, x0, fminOptions);
fprintf('fminsearch最適解: Kp_pos=%.3f Kp_vel=%.3f Ki_vel=%.3f, cost=%.6f\n', x_opt_fmin, cost_opt_fmin);

writematrix(fmin_history, 'data/results/phase6_convergence_fminsearch.csv');

%% 3. fminsearchを複数初期値で実行し追加候補を取得（探索範囲内のランダム初期値）
rng(7);
n_extra = 3;
extra_candidates = zeros(n_extra,3);
extra_costs = zeros(n_extra,1);
for i = 1:n_extra
    x0_rand = lb + rand(1,3).*(ub-lb);
    [xo, co] = fminsearch(costFcn, x0_rand, optimset('MaxIter',300,'MaxFunEvals',1000,'Display','off'));
    xo = min(max(xo, lb), ub);
    extra_candidates(i,:) = xo;
    extra_costs(i) = co;
    fprintf('追加候補%d: Kp_pos=%.3f Kp_vel=%.3f Ki_vel=%.3f, cost=%.6f\n', i, xo, co);
end

%% 4. 候補ゲインセットの整理（少なくとも3セット）
candidates = [x_opt_ga; x_opt_fmin; extra_candidates];
costs = [cost_opt_ga; cost_opt_fmin; extra_costs];
labels = [{'GA'}, {'fminsearch(初期値=元ゲイン)'}, arrayfun(@(i) sprintf('fminsearch(乱数初期値%d)',i), 1:n_extra, 'UniformOutput', false)];

[costs_sorted, sidx] = sort(costs);
candidates_sorted = candidates(sidx,:);
labels_sorted = labels(sidx);

fid = fopen('data/results/phase6_candidates.csv', 'w');
fprintf(fid, 'label,Kp_pos,Kp_vel,Ki_vel,cost\n');
for i = 1:numel(labels_sorted)
    fprintf(fid, '%s,%.6f,%.6f,%.6f,%.8f\n', labels_sorted{i}, candidates_sorted(i,:), costs_sorted(i));
end
fclose(fid);

%% 5. 上位3候補の応答シミュレーション
top3 = candidates_sorted(1:min(3,size(candidates_sorted,1)),:);
fid2 = fopen('data/results/phase6_top3_responses.csv', 'w');
header = 't';
for i=1:size(top3,1)
    header = [header, sprintf(',theta_%d', i)]; %#ok<AGROW>
end
fprintf(fid2, '%s\n', header);

resp_mat = [];
metrics = struct('settling', {}, 'overshoot', {});
for i = 1:size(top3,1)
    [theta_out, t_out] = run_closed_loop_surrogate(top3(i,:), surrogate_net, params_opt);
    if i==1, resp_mat = t_out; end
    resp_mat = [resp_mat, theta_out]; %#ok<AGROW>
    ts = calc_settling_time(theta_out, params_opt.theta_target, t_out, 0.02);
    ov = max(0, max(theta_out)-params_opt.theta_target)/params_opt.theta_target*100;
    metrics(i).settling = ts; %#ok<SAGROW>
    metrics(i).overshoot = ov; %#ok<SAGROW>
    fprintf('候補%d: 整定時間=%.4fs オーバーシュート=%.2f%%\n', i, ts, ov);
end
writematrix(resp_mat, 'data/results/phase6_top3_responses.csv');

%% 6. パレート図用: 全候補(GA+fminsearch群)の整定時間 vs オーバーシュート
pareto_data = zeros(size(candidates,1), 2);
for i = 1:size(candidates,1)
    [theta_out, t_out] = run_closed_loop_surrogate(candidates(i,:), surrogate_net, params_opt);
    ts = calc_settling_time(theta_out, params_opt.theta_target, t_out, 0.02);
    if isnan(ts), ts = params_opt.sim_time; end
    ov = max(0, max(theta_out)-params_opt.theta_target)/params_opt.theta_target*100;
    pareto_data(i,:) = [ts, ov];
end
writematrix(pareto_data, 'data/results/phase6_pareto_data.csv');

save('data/results/phase6_results.mat', 'x_opt_ga', 'cost_opt_ga', 'x_opt_fmin', 'cost_opt_fmin', ...
    'candidates_sorted', 'costs_sorted', 'labels_sorted', 'top3', 'metrics');

disp('Phase6 ゲイン最適化 完了');

function [state, options, optchanged] = local_ga_log(options, state, flag)
    optchanged = false;
    if strcmp(flag, 'iter')
        assignin('base', 'ga_history', [evalin('base','ga_history'); state.Generation, min(state.Score)]);
    end
end

function stop = local_fmin_log(~, optimValues, flag)
    stop = false;
    if strcmp(flag, 'iter')
        assignin('base', 'fmin_history', [evalin('base','fmin_history'); optimValues.iteration, optimValues.fval]);
    end
end
