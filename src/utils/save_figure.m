function save_figure(fig_handle, filename)
% save_figure  Figureをfigures/にPNGとして保存し閉じる
% 入力: fig_handle - figureハンドル, filename - 保存ファイル名 (拡張子込み)
% 出力: なし（figures/<filename> にPNG保存）
% 作成日: 2026-09-28
    set(fig_handle, 'PaperPositionMode', 'auto');
    print(fig_handle, fullfile('figures', filename), '-dpng', '-r150');
    close(fig_handle);
end
