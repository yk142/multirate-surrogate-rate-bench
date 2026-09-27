function data = generate_chirp_data(f0, f1, duration, amplitude, params)
% generate_chirp_data  チャープトルク入力で物理モデルを開ループ励振しデータ生成
% 入力: f0, f1 - チャープ開始/終了周波数 [Hz], duration - 加振時間 [s],
%       amplitude - トルク振幅 [N*m], params - src/params.mの変数を含むstruct
% 出力: data - struct with fields t, tau, theta_a, omega_a, theta_m
%       （物理モデルの計算サンプルレート Ts_plant で生成、ダウンサンプルは呼び出し側）
% 作成日: 2026-09-28
    A = [0, 1, 0, 0;
         -params.k_s/params.J_m, -(params.b_m+params.b_s)/params.J_m, params.k_s/params.J_m, params.b_s/params.J_m;
         0, 0, 0, 1;
         params.k_s/params.J_a, params.b_s/params.J_a, -params.k_s/params.J_a, -(params.b_a+params.b_s)/params.J_a];
    B = [0; 1/params.J_m; 0; 0];
    C = [0 0 1 0; 0 0 0 1; 1 0 0 0]; % theta_a, omega_a, theta_m
    D = [0;0;0];
    sys = ss(A,B,C,D);

    t = (0:params.Ts_plant:duration)';
    tau = amplitude * chirp(t, f0, duration, f1);

    y = lsim(sys, tau, t);

    data.t = t;
    data.tau = tau;
    data.theta_a = y(:,1);
    data.omega_a = y(:,2);
    data.theta_m = y(:,3);
end
