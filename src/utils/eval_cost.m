function cost = eval_cost(gains, surrogate_net, params)
% eval_cost  ゲイン最適化のコスト関数（サロゲート閉ループシミュレーションに基づく）
% 入力: gains = [Kp_pos, Kp_vel, Ki_vel], surrogate_net, params
%       (run_closed_loop_surrogateと同じ引数。REQUIREMENTS.md Section 6.2に準拠)
% 出力: cost - スカラーコスト (10*J_track + 5*J_settle + 2*J_overshoot)
% 作成日: 2026-10-03
    [theta_out, t_out] = run_closed_loop_surrogate(gains, surrogate_net, params);

    [theta_ref_raw, t_ref] = ptp_profile(0, params.theta_target, params.sim_time, params.Ts_pos, params.move_time);
    theta_ref = interp1(t_ref, theta_ref_raw, t_out, 'previous', 'extrap');

    e = theta_ref - theta_out;
    J_track = mean(e.^2);
    J_settle = calc_settling_time(theta_out, params.theta_target, t_out, 0.02);
    if isnan(J_settle)
        J_settle = params.sim_time; % 整定しなかった場合は最大ペナルティ
    end
    J_overshoot = max(0, max(theta_out) - params.theta_target) / params.theta_target;

    cost = 10*J_track + 5*J_settle + 2*J_overshoot;
end
