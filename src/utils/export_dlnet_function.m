function export_dlnet_function(net, x_center, x_scale, dy_scale_component, filename, funcname)
% export_dlnet_function  dlnetwork(4->...->1, tanh隠れ層)をコード生成互換の
% スタンドアロンMATLAB関数として書き出す（入力正規化・出力スケーリングを内蔵）
% 入力: net - dlnetwork, x_center,x_scale - 4x1入力正規化定数,
%       dy_scale_component - スカラー出力スケール, filename - 出力.mパス,
%       funcname - 生成する関数名
% 出力: なし（filenameに関数を書き出す）
% 作成日: 2026-10-02
    L = net.Learnables;
    layerNames = unique(L.Layer, 'stable');
    getW = @(name) extractdata(L.Value{strcmp(L.Layer,name) & strcmp(L.Parameter,'Weights')});
    getB = @(name) extractdata(L.Value{strcmp(L.Layer,name) & strcmp(L.Parameter,'Bias')});

    fcLayers = layerNames(startsWith(layerNames, 'fc'));
    W1 = getW(fcLayers{1}); b1 = getB(fcLayers{1});
    W2 = getW(fcLayers{2}); b2 = getB(fcLayers{2});
    W3 = getW(fcLayers{3}); b3 = getB(fcLayers{3});
    W4 = getW(fcLayers{4}); b4 = getB(fcLayers{4});

    fid = fopen(filename, 'w');
    fprintf(fid, 'function y = %s(x) %%#codegen\n', funcname);
    fprintf(fid, '%% 自動生成: dlnetworkの重みを埋め込んだコード生成互換関数\n');
    fprintf(fid, '%% 入力: x - 4x1 状態ベクトル [theta_a;omega_a;theta_m;tau]\n');
    fprintf(fid, '%% 出力: y - スカラー delta (物理単位)\n');
    fprintf(fid, 'x_center = %s;\n', mat2str(x_center));
    fprintf(fid, 'x_scale = %s;\n', mat2str(x_scale));
    fprintf(fid, 'dy_scale = %.17g;\n', dy_scale_component);
    fprintf(fid, 'W1 = %s;\n', mat2str(W1));
    fprintf(fid, 'b1 = %s;\n', mat2str(b1));
    fprintf(fid, 'W2 = %s;\n', mat2str(W2));
    fprintf(fid, 'b2 = %s;\n', mat2str(b2));
    fprintf(fid, 'W3 = %s;\n', mat2str(W3));
    fprintf(fid, 'b3 = %s;\n', mat2str(b3));
    fprintf(fid, 'W4 = %s;\n', mat2str(W4));
    fprintf(fid, 'b4 = %s;\n', mat2str(b4));
    fprintf(fid, 'xn = (x - x_center) ./ x_scale;\n');
    fprintf(fid, 'h1 = tanh(W1*xn + b1);\n');
    fprintf(fid, 'h2 = tanh(W2*h1 + b2);\n');
    fprintf(fid, 'h3 = tanh(W3*h2 + b3);\n');
    fprintf(fid, 'yraw = W4*h3 + b4;\n');
    fprintf(fid, 'y = yraw * dy_scale;\n');
    fprintf(fid, 'end\n');
    fclose(fid);
end
