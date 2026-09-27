function [theta_ref, t] = ptp_profile(theta_start, theta_end, t_total, dt, move_time)
% ptp_profile  PTP(Point-to-Point)位置指令のS字（最小躍度）プロファイルを生成
% 入力:
%   theta_start - 開始位置 [rad]
%   theta_end   - 目標位置 [rad]
%   t_total     - 出力する軌道の全時間長 [s]
%   dt          - サンプリング時間 [s]
%   move_time   - 移動時間（省略時0.5s）。move_time経過後はtheta_endを保持
% 出力:
%   theta_ref   - S字プロファイルの位置指令列（列ベクトル）
%   t           - 対応する時刻ベクトル（列ベクトル, 0:dt:t_total）
% 作成日: 2026-09-28
    if nargin < 5
        move_time = 0.5;
    end

    t = (0:dt:t_total)';
    tau = min(t / move_time, 1);
    % 最小躍度（quintic）S字プロファイル: 10*tau^3 - 15*tau^4 + 6*tau^5
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    theta_ref = theta_start + (theta_end - theta_start) * s;
end
