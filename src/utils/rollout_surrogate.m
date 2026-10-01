function x_traj = rollout_surrogate(net, x0, tau_seq)
% rollout_surrogate  学習済みサロゲートで自由ロールアウト(自己回帰予測)を行う
% 入力: net - train_surrogateで学習したネットワーク, x0 - 初期状態[theta_a,omega_a,theta_m,tau] (1x4)
%       tau_seq - 既知の外生トルク入力列 (Nx1)。各ステップでtauは実測値に置き換える
% 出力: x_traj - ロールアウト軌道 (Nx4)
% 作成日: 2026-09-28
    N = length(tau_seq);
    x_traj = zeros(N, 4);
    x_traj(1,:) = x0;
    x_traj(1,4) = tau_seq(1);
    x = x_traj(1,:);
    for k = 1:N-1
        dx = net.predict(x);
        x_next = x + dx;
        x_next(4) = tau_seq(k+1); % tauは既知の外生入力として上書き
        x_traj(k+1,:) = x_next;
        x = x_next;
    end
end
