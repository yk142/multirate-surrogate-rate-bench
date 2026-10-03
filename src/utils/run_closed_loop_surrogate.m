function [theta_out, t_out] = run_closed_loop_surrogate(gains, surrogate_net, params)
% run_closed_loop_surrogate  サロゲート(線形モデル)をプラントとした
% カスケード制御閉ループシミュレーション（ゲイン最適化のコスト関数評価用）
%
% MATLABで直接実装することでSimulinkを介さず高速にシミュレーションできる
% （ga/fminsearchは数千回のコスト評価を必要とするため）。Phase5で検証した
% cascade_controller_surrogate.slxと数式的に同一（D=0相当: 出力は前ステップの
% 状態から取得, 代数ループを回避）のロジックを実装している。
%
% 入力: gains = [Kp_pos, Kp_vel, Ki_vel]
%       surrogate_net - struct with fields coef_spring, coef_omega, Ts
%         (data/results/surrogate_best.matの surrogate_coef_spring等)
%       params - struct with fields Ts_pos, Ts_vel, tau_max, f_notch, depth_dB,
%                Q_notch, enable_notch, sim_time, theta_target, move_time
% 出力: theta_out - theta_aの時系列（surrogate_net.Tsレート）
%       t_out - 対応する時刻ベクトル
%
% 重要: 速度ループ(PI制御・ノッチフィルタ)はparams.Ts_vel(実機の設計レート,
% 既定1kHz)でのみ更新し、tau指令はその間ZOH保持する。サロゲートの計算レート
% (surrogate_net.Ts, 既定2kHz)で毎ステップ更新すると、実際のカスケード制御器
% (cascade_controller.slx)より速い離散化レートでPI/ノッチが動作することになり、
% 同じゲイン値でも実機と異なる(より安定した)挙動を示してしまう
% （Phase6初版のバグ。Phase7で発覚し修正。詳細はIssue #6参照）。
% 作成日: 2026-10-03

    Kp_pos = gains(1);
    Kp_vel = gains(2);
    Ki_vel = gains(3);

    Ts = surrogate_net.Ts;
    coef_spring = surrogate_net.coef_spring;
    coef_omega = surrogate_net.coef_omega;

    N = round(params.sim_time / Ts) + 1;
    t_out = (0:N-1)' * Ts;

    [theta_ref_raw, t_ref] = ptp_profile(0, params.theta_target, params.sim_time, params.Ts_pos, params.move_time);
    theta_ref_vec = interp1(t_ref, theta_ref_raw, t_out, 'previous', 'extrap');

    if params.enable_notch
        [nb, na] = design_notch(params.f_notch, params.depth_dB, params.Q_notch, 1/params.Ts_vel);
    else
        nb = 1; na = 1;
    end

    pos_update_every = max(1, round(params.Ts_pos/Ts));
    vel_update_every = max(1, round(params.Ts_vel/Ts));

    state = [0;0;0;0]; % theta_a, omega_a, theta_m, spring_defl_rate
    int_err = 0;
    omega_d = 0;
    tau_cmd = 0;
    theta_out = zeros(N,1);
    omega_a_raw_prev = zeros(2,1);
    omega_a_filt_prev = zeros(2,1);

    for k = 1:N
        theta_out(k) = state(1);

        if mod(k-1, pos_update_every) == 0
            omega_d = Kp_pos * (theta_ref_vec(k) - state(1));
        end

        if mod(k-1, vel_update_every) == 0
            omega_a_raw = state(2);
            if params.enable_notch
                omega_a_filt = nb(1)*omega_a_raw + nb(2)*omega_a_raw_prev(1) + nb(3)*omega_a_raw_prev(2) ...
                               - na(2)*omega_a_filt_prev(1) - na(3)*omega_a_filt_prev(2);
                omega_a_raw_prev = [omega_a_raw; omega_a_raw_prev(1)];
                omega_a_filt_prev = [omega_a_filt; omega_a_filt_prev(1)];
            else
                omega_a_filt = omega_a_raw;
            end

            err_v = omega_d - omega_a_filt;
            int_err_new = int_err + err_v*params.Ts_vel;
            tau_cmd = Kp_vel*err_v + Ki_vel*int_err_new;
            if tau_cmd > params.tau_max || tau_cmd < -params.tau_max
                tau_cmd = max(min(tau_cmd, params.tau_max), -params.tau_max);
            else
                int_err = int_err_new; % アンチワインドアップ(飽和時は積分を更新しない)
            end
        end

        if k < N
            theta_a = state(1); omega_a = state(2); theta_m = state(3); rate = state(4);
            spring_defl = theta_m - theta_a;
            feat = [spring_defl, rate, omega_a, tau_cmd, 1];
            d_spring = feat * coef_spring;
            d_omega = feat * coef_omega;
            omega_next = omega_a + d_omega;
            theta_a_next = theta_a + (omega_a + 0.5*d_omega) * Ts;
            spring_next = spring_defl + d_spring;
            theta_m_next = theta_a_next + spring_next;
            rate_next = d_spring / Ts;
            state = [theta_a_next; omega_next; theta_m_next; rate_next];
        end
    end
end
