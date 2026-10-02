function [net_s, net_o, info] = train_surrogate_rollout_rel(X0, TauSeq, Targets, ...
    X0_val, TauSeq_val, Targets_val, x_center, x_scale, dy_scale, numEpochs, miniBatchSize)
% train_surrogate_rollout_rel  相対状態表現でのロールアウト損失学習
%
% 入力特徴量を [theta_m-theta_a(関節ねじれ角), omega_a, tau] の3次元に簡略化した
% 改良版。theta_aの絶対値は物理的に無関係（delta_theta_a = omega_a に対する
% 運動学的関係のみで決まり、theta_aの値自体はダイナミクスに影響しない）であるため、
% 絶対位置を入力から除外し並進不変にすることで、学習データに現れなかった絶対位置
% 領域への外挿に対して頑健なモデルを得る（Issue #5のロールアウト発散問題への対応）。
%
% 入力:
%   X0 - 3xN 初期状態 [spring_defl; omega_a; tau]
%   TauSeq - KxN 既知の外生トルク列
%   Targets - 2xKxN 各ステップの真の[spring_defl; omega_a]
%   x_center, x_scale - 3x1 入力正規化定数
%   dy_scale - 2x1 出力(delta)スケール定数 [d_spring_defl; d_omega_a]
% 出力:
%   net_s, net_o - 学習済みdlnetwork(spring_defl/omega_a用)
%   info - 学習記録
% 作成日: 2026-10-02

    layers = @() [
        featureInputLayer(3,'Normalization','none')
        fullyConnectedLayer(64)
        tanhLayer
        fullyConnectedLayer(64)
        tanhLayer
        fullyConnectedLayer(32)
        tanhLayer
        fullyConnectedLayer(1)
    ];
    net_s = dlnetwork(layers());
    net_o = dlnetwork(layers());

    x_center = dlarray(x_center);
    x_scale  = dlarray(x_scale);
    dy_scale = dlarray(dy_scale);

    N = size(X0,2);
    avgGrad_s = []; avgSqGrad_s = [];
    avgGrad_o = []; avgSqGrad_o = [];
    iter = 0;

    train_loss_hist = zeros(numEpochs,1);
    val_loss_hist = zeros(numEpochs,1);

    for epoch = 1:numEpochs
        perm = randperm(N);
        epochLoss = 0; nBatches = 0;
        for bstart = 1:miniBatchSize:N
            bidx = perm(bstart:min(bstart+miniBatchSize-1, N));
            x0b = dlarray(X0(:,bidx), 'CB');
            taub = TauSeq(:,bidx);
            targb = Targets(:,:,bidx);

            [loss, g_s, g_o] = dlfeval(@rollout_loss_fn_rel, net_s, net_o, ...
                x0b, taub, targb, x_center, x_scale, dy_scale);

            iter = iter + 1;
            [net_s, avgGrad_s, avgSqGrad_s] = adamupdate(net_s, g_s, avgGrad_s, avgSqGrad_s, iter, 1e-3);
            [net_o, avgGrad_o, avgSqGrad_o] = adamupdate(net_o, g_o, avgGrad_o, avgSqGrad_o, iter, 1e-3);

            epochLoss = epochLoss + double(extractdata(loss));
            nBatches = nBatches + 1;
        end
        train_loss_hist(epoch) = epochLoss / nBatches;

        x0v = dlarray(X0_val, 'CB');
        valLoss = dlfeval(@rollout_loss_fn_rel_eval, net_s, net_o, x0v, TauSeq_val, Targets_val, x_center, x_scale, dy_scale);
        val_loss_hist(epoch) = double(extractdata(valLoss));

        if mod(epoch,5)==0 || epoch==1 || epoch==numEpochs
            fprintf('  epoch %d/%d: train_loss=%.6e val_loss=%.6e\n', epoch, numEpochs, train_loss_hist(epoch), val_loss_hist(epoch));
        end
    end

    info.train_loss = train_loss_hist;
    info.val_loss = val_loss_hist;
end

function [loss, g_s, g_o] = rollout_loss_fn_rel(net_s, net_o, x0, tauSeq, targets, x_center, x_scale, dy_scale)
    loss = rollout_forward_loss_rel(net_s, net_o, x0, tauSeq, targets, x_center, x_scale, dy_scale);
    g_s = dlgradient(loss, net_s.Learnables);
    g_o = dlgradient(loss, net_o.Learnables);
end

function loss = rollout_loss_fn_rel_eval(net_s, net_o, x0, tauSeq, targets, x_center, x_scale, dy_scale)
    loss = rollout_forward_loss_rel(net_s, net_o, x0, tauSeq, targets, x_center, x_scale, dy_scale);
end

function loss = rollout_forward_loss_rel(net_s, net_o, x0, tauSeq, targets, x_center, x_scale, dy_scale)
    x = x0; % [spring_defl; omega_a; tau] 3xB
    K = size(tauSeq,1);
    loss = dlarray(0);
    for k = 1:K
        xn = (x - x_center) ./ x_scale;
        ds = forward(net_s, xn) .* dy_scale(1);
        do = forward(net_o, xn) .* dy_scale(2);
        spring_next = x(1,:) + ds;
        omega_next  = x(2,:) + do;
        tau_next = tauSeq(k,:);
        x = [spring_next; omega_next; tau_next];

        targ_k = squeeze(targets(:,k,:)); % 2xB
        err = ([spring_next; omega_next] - targ_k) ./ dy_scale;
        loss = loss + mean(err.^2, 'all');
    end
    loss = loss / K;
end
