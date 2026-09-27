function [b, a] = design_notch(f0, depth_dB, Q, fs)
% design_notch  RBJ Audio-EQ-Cookbook形式のピーキングノッチフィルタを設計する
% 指定周波数f0で指定深さ(depth_dB, 負値)の減衰を持つ2次IIRフィルタを返す。
% 入力: f0 - ノッチ中心周波数 [Hz], depth_dB - 減衰量 [dB]（負値）,
%       Q - Q値, fs - サンプリング周波数 [Hz]
% 出力: b, a - 離散フィルタ係数（tf形式, a(1)=1に正規化）
% 作成日: 2026-09-28
    A = 10^(depth_dB/40);
    w0 = 2*pi*f0/fs;
    alpha = sin(w0)/(2*Q);
    cosw0 = cos(w0);

    b0 = 1 + alpha*A;
    b1 = -2*cosw0;
    b2 = 1 - alpha*A;
    a0 = 1 + alpha/A;
    a1 = -2*cosw0;
    a2 = 1 - alpha/A;

    b = [b0, b1, b2] / a0;
    a = [a0, a1, a2] / a0;
end
