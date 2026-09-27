function ts = calc_settling_time(theta, theta_target, t, tol)
% calc_settling_time  応答が目標値の許容誤差帯域に収束するまでの整定時間を計算
% 入力: theta - 応答の時系列, theta_target - 目標値, t - 時刻ベクトル,
%       tol - 許容誤差（目標値に対する比率, 省略時0.02 = 2%）
% 出力: ts - 整定時間 [s]（収束しなかった場合はNaN）
% 作成日: 2026-09-28
    if nargin < 4
        tol = 0.02;
    end
    band = tol * abs(theta_target);
    err = abs(theta - theta_target);
    outside = find(err > band);
    if isempty(outside)
        ts = t(1);
    elseif outside(end) == length(t)
        ts = NaN;
    else
        ts = t(outside(end) + 1);
    end
end
