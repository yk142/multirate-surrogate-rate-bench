function export_linear_coefs(coef_spring, coef_omega, Ts, filename)
% export_linear_coefs  線形サロゲートモデルの係数をコード生成互換関数として書き出す
% 入力: coef_spring, coef_omega - 5x1 線形回帰係数
%       (feat=[spring_defl;spring_defl_rate;omega_a;tau;1]に対する係数)
%       Ts - サンプル時間 [s], filename - 出力.mパス
% 出力: なし。state=[theta_a;omega_a;theta_m;spring_defl_rate], tauを入力に
%       次状態を返す surrogate_predict_linear(state, tau) を書き出す
% 作成日: 2026-10-03
    fid = fopen(filename, 'w');
    fprintf(fid, 'function state_next = surrogate_predict_linear(state, tau) %%#codegen\n');
    fprintf(fid, '%% 自動生成: 線形サロゲートモデル（spring_defl_rateを含むMarkov完備な状態表現）\n');
    fprintf(fid, '%% state = [theta_a; omega_a; theta_m; spring_defl_rate]\n');
    fprintf(fid, 'Ts = %.17g;\n', Ts);
    fprintf(fid, 'coef_spring = %s;\n', mat2str(coef_spring));
    fprintf(fid, 'coef_omega = %s;\n', mat2str(coef_omega));
    fprintf(fid, 'theta_a = state(1); omega_a = state(2); theta_m = state(3); rate = state(4);\n');
    fprintf(fid, 'spring_defl = theta_m - theta_a;\n');
    fprintf(fid, 'feat = [spring_defl; rate; omega_a; tau; 1];\n');
    fprintf(fid, 'd_spring = coef_spring'' * feat;\n');
    fprintf(fid, 'd_omega = coef_omega'' * feat;\n');
    fprintf(fid, 'omega_next = omega_a + d_omega;\n');
    fprintf(fid, 'theta_a_next = theta_a + (omega_a + 0.5*d_omega) * Ts;\n');
    fprintf(fid, 'spring_next = spring_defl + d_spring;\n');
    fprintf(fid, 'theta_m_next = theta_a_next + spring_next;\n');
    fprintf(fid, 'rate_next = d_spring / Ts;\n');
    fprintf(fid, 'state_next = [theta_a_next; omega_next; theta_m_next; rate_next];\n');
    fprintf(fid, 'end\n');
    fclose(fid);
end
