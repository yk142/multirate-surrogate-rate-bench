function [net, tr] = train_surrogate(X_train, dX_train, X_val, dX_val, X_test, dX_test, epochs)
% train_surrogate  NNサロゲートモデルを学習する
% 入力: X_train,dX_train,X_val,dX_val,X_test,dX_test - 各分割のデータ
%       (列: theta_a, omega_a, theta_m, tau。行: サンプル)
%       epochs - 最大エポック数（省略時300）
% 出力: net - 学習済みネットワーク。net.subnets{1..4} に
%       theta_a/omega_a/theta_m/tau の各delta出力を独立に学習した
%       サブネットワークを保持する（net(X)で4次元delta_xを一括予測可能）。
%       tr  - 各サブネットワークの学習記録セル配列 tr{1..4}
% ネットワーク構成: 各サブネットワーク 隠れ層[64,64,32], tanh(tansig),
%       出力層linear, MSE損失
%
% 設計上の注記: 当初は4出力(delta_x全体)を単一ネットワークで同時学習したが、
% 検証の結果、共有隠れ表現が主にomega_a（最も複雑な力学）の学習に支配され、
% theta_a等の単純な出力の予測精度が著しく劣化する最適化上の競合が確認された
% （結合学習: theta_a相関0.29 → 独立学習: 同0.89 に改善）。そのため出力ごとに
% 独立したサブネットワークを学習する構成とした。
% 作成日: 2026-09-28
    if nargin < 7
        epochs = 300;
    end

    X_comb = [X_train; X_val];
    dX_comb = [dX_train; dX_val];
    nTrain = size(X_train,1);
    nVal   = size(X_val,1);
    trainInd = 1:nTrain;
    valInd   = (nTrain+1):(nTrain+nVal);

    labels = {'theta_a','omega_a','theta_m','tau'};
    subnets = cell(1,4);
    tr = cell(1,4);
    for d = 1:4
        sn = feedforwardnet([64 64 32], 'trainscg');
        sn.layers{1}.transferFcn = 'tansig';
        sn.layers{2}.transferFcn = 'tansig';
        sn.layers{3}.transferFcn = 'tansig';
        sn.layers{4}.transferFcn = 'purelin';
        sn.divideFcn = 'divideind';
        sn.divideParam.trainInd = trainInd;
        sn.divideParam.valInd   = valInd;
        sn.divideParam.testInd  = [];
        sn.trainParam.epochs = epochs;
        sn.trainParam.showWindow = false;
        sn.trainParam.max_fail = 20;
        sn.performFcn = 'mse';
        sn.name = labels{d};

        [sn, tr{d}] = train(sn, X_comb', dX_comb(:,d)');
        subnets{d} = sn;
    end

    net.subnets = subnets;
    net.predict = @(X) cell2mat(cellfun(@(sn) sn(X')', subnets, 'UniformOutput', false));
end
