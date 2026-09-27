function data = generate_ptp_data(theta_end_deg, move_time, duration, params)
% generate_ptp_data  カスケード制御器(閉ループ)でPTP軌道を実行しデータ生成
% 入力: theta_end_deg - 目標位置 [deg], move_time - 移動時間 [s],
%       duration - シミュレーション全体時間 [s], params - src/params.mの変数
% 出力: data - struct with fields t, tau, theta_a, omega_a, theta_m, theta_ref
%       （物理モデルの計算サンプルレート Ts_plant で生成）
% 作成日: 2026-09-28
    theta_end = deg2rad(theta_end_deg);
    [theta_ref, t_ref] = ptp_profile(0, theta_end, duration, params.Ts_pos, move_time);
    ptp_ref_ts = timeseries(theta_ref, t_ref);
    assignin('base', 'ptp_ref_ts', ptp_ref_ts);
    assignin('base', 'enable_notch', true);

    modelPath = fullfile('models', 'controller', 'cascade_controller.slx');
    load_system(modelPath);
    simOut = sim('cascade_controller', 'StopTime', num2str(duration));
    close_system('cascade_controller', 0);

    ds = simOut.yout;
    data.t        = ds{1}.Values.Time;
    data.theta_a  = ds{1}.Values.Data;
    data.omega_a  = ds{2}.Values.Data;
    data.tau      = interp1(ds{3}.Values.Time, ds{3}.Values.Data, data.t, 'previous', 'extrap');
    data.theta_ref = interp1(ds{4}.Values.Time, ds{4}.Values.Data, data.t, 'previous', 'extrap');
    data.theta_m  = ds{5}.Values.Data;
end
