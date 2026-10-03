function state_next = surrogate_predict_linear(state, tau) %#codegen
% 自動生成: 線形サロゲートモデル（spring_defl_rateを含むMarkov完備な状態表現）
% state = [theta_a; omega_a; theta_m; spring_defl_rate]
Ts = 0.00050000000000000001;
coef_spring = [-0.00884025492815297;0.0004960431473051;-1.43526530896725e-06;0.000247132782621892;7.98862822345276e-10];
coef_omega = [2.93830071061224;0.00123114647460599;-0.00200091318660573;0.000552308089870735;4.76749781194684e-08];
theta_a = state(1); omega_a = state(2); theta_m = state(3); rate = state(4);
spring_defl = theta_m - theta_a;
feat = [spring_defl; rate; omega_a; tau; 1];
d_spring = coef_spring' * feat;
d_omega = coef_omega' * feat;
omega_next = omega_a + d_omega;
theta_a_next = theta_a + (omega_a + 0.5*d_omega) * Ts;
spring_next = spring_defl + d_spring;
theta_m_next = theta_a_next + spring_next;
rate_next = d_spring / Ts;
state_next = [theta_a_next; omega_next; theta_m_next; rate_next];
end
