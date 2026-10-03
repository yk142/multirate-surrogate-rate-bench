function cost = eval_cost(gains, params)
% eval_cost  ゲイン最適化のコスト関数（Simulinkサロゲートモデルの閉ループ
% シミュレーションに基づく。REQUIREMENTS.md Section 6.2に準拠）
%
% 注記: 当初はMATLABで独自実装した高速閉ループシミュレータ
% (run_closed_loop_surrogate.m)でコスト評価していたが、Rate Transition
% ブロックの遅延等、Simulinkモデル(cascade_controller_surrogate.slx)の
% 実行セマンティクスを完全には再現できておらず、最適化されたゲインが
% 実際のSimulinkモデル・物理モデルでは不安定になる食い違いが判明した
% （Phase7検証, Issue #7参照）。そのため、Phase5で物理モデルとの完全一致
% （閉ループ誤差0%）を検証済みのcascade_controller_surrogate.slxを
% FastRestartモードで直接呼び出す方式に変更した。
%
% 入力: gains = [Kp_pos, Kp_vel, Ki_vel]
%       params - struct with fields Ts_pos, tau_max(未使用, モデル内で設定済み),
%                enable_notch, sim_time, theta_target, move_time
% 出力: cost - スカラーコスト (10*J_track + 5*J_settle + 2*J_overshoot)
% 作成日: 2026-10-03
    assignin('base', 'Kp_pos', gains(1));
    assignin('base', 'Kp_vel', gains(2));
    assignin('base', 'Ki_vel', gains(3));
    assignin('base', 'enable_notch', params.enable_notch);

    simOut = sim('cascade_controller_surrogate');
    ds = simOut.yout;
    t_out = ds{1}.Values.Time;
    theta_out = squeeze(ds{1}.Values.Data);

    [theta_ref_raw, t_ref] = ptp_profile(0, params.theta_target, params.sim_time, params.Ts_pos, params.move_time);
    theta_ref = interp1(t_ref, theta_ref_raw, t_out, 'previous', 'extrap');

    e = theta_ref - theta_out;
    J_track = mean(e.^2);
    J_settle = calc_settling_time(theta_out, params.theta_target, t_out, 0.02);
    if isnan(J_settle)
        J_settle = params.sim_time;
    end
    J_overshoot = max(0, max(theta_out) - params.theta_target) / params.theta_target;

    cost = 10*J_track + 5*J_settle + 2*J_overshoot;
end
