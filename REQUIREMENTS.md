# multirate-surrogate-rate-bench
## ロボットアームPTP制御 — マルチレート×NNサロゲート サンプリングレート検証
### Claude Code 向け要件定義書

---

## 0. このドキュメントの使い方

本ドキュメントはClaude Codeが自律的に読み取り、実装・検証・記録を進めるための仕様書である。
各セクションの指示に従い、**Issueの作成 → ブランチ作成 → 実装 → 検証 → 結果をIssueにコメント記録**
のサイクルを繰り返すこと。

---

## 1. プロジェクト概要

### 目的

1軸ロボットアームのPTP（Point-to-Point）位置制御において、
**NNサロゲートモデルのサンプリングレートが制御精度・学習精度・最適化効率に与える影響を定量的に明らかにする**。
あわせてマルチレートのカスケード制御構成（位置200 Hz / 速度1 kHz / プラント8 kHz）との整合性を検証し、
サロゲートを用いた制御ゲイン最適化シミュレーションを行う。

### 検証する主なレート

| 対象 | 候補レート | 検証内容 |
|------|-----------|---------|
| NNサロゲートの推論レート | 500 Hz / 1 kHz / 2 kHz / 8 kHz | 精度・学習収束・ロールアウト安定性の比較 |
| 位置ループ | 200 Hz（固定） | マルチレート整合性の確認 |
| 速度ループ | 1 kHz（固定） | 同上 |
| プラント（物理 / サロゲート） | 8 kHz（固定） | 電流ループ省略時の基準 |

### 検証の流れ

```
Phase 1: 物理モデル構築        → Simulink モデル + 周波数応答確認
Phase 2: カスケード制御器構築  → PTP ステップ応答確認
Phase 3: 学習データ生成        → チャープ・PTP データ収集
Phase 4: NNサロゲート学習      → 物理モデルとの精度比較
Phase 5: 閉ループ代替検証      → サロゲートをプラントとした制御
Phase 6: ゲイン最適化          → サロゲートベースの最適ゲイン探索
Phase 7: 最終検証              → 最適ゲインを物理モデルで評価
```

---

## 2. 技術スタック・環境

| 項目 | 内容 |
|------|------|
| シミュレーション | MATLAB / Simulink R2022b 以降 |
| NNフレームワーク | MATLAB Deep Learning Toolbox |
| 最適化 | MATLAB Optimization Toolbox（fminsearch / ga） |
| バージョン管理 | GitHub（リポジトリは事前に作成済み） |
| Issue管理 | GitHub Issues（本ドキュメントのPhaseごとに作成） |

### ディレクトリ構成

```
repo-root/
├── REQUIREMENTS.md           ← 本ファイル
├── README.md                 ← プロジェクト概要（Phase 1で作成）
├── models/
│   ├── physical/             ← 物理モデル Simulinkファイル
│   ├── controller/           ← 制御器 Simulinkファイル
│   └── surrogate/            ← サロゲート統合 Simulinkファイル
├── src/
│   ├── params.m              ← 共通パラメータ定義
│   ├── train_surrogate.m     ← NN学習スクリプト
│   ├── optimize_gains.m      ← ゲイン最適化スクリプト
│   └── utils/                ← 共通関数
├── data/
│   ├── training/             ← 学習用データ (.mat)
│   └── results/              ← 検証結果データ (.mat)
└── figures/                  ← 出力PNG（Issue添付用）
```

---

## 3. 物理モデル仕様

### 3.1 対象システム

1軸ロボットアーム（回転関節1つ）。
弾性変形による30 Hz振動モードを持つ2慣性系モデルとする。

### 3.2 物理パラメータ（`src/params.m` に定義）

```matlab
%% ロボットアームパラメータ
J_m  = 1e-3;      % モータ慣性モーメント [kg·m²]
J_a  = 5e-3;      % アーム慣性モーメント [kg·m²]
k_s  = 180;       % 関節ばね剛性 [N·m/rad]  → 共振: 30 Hz
b_m  = 0.01;      % モータ粘性摩擦 [N·m·s/rad]
b_a  = 0.02;      % アーム粘性摩擦 [N·m·s/rad]
b_s  = 0.005;     % 関節ダンピング [N·m·s/rad]

%% 確認: 共振周波数
% f_res = (1/(2*pi)) * sqrt(k_s*(1/J_m + 1/J_a)) ≈ 30 Hz になること
```

### 3.3 状態方程式

状態ベクトル: `x = [θ_m, ω_m, θ_a, ω_a]`
（θ_m: モータ角度, ω_m: モータ角速度, θ_a: アーム角度, ω_a: アーム角速度）

入力: `u = τ`（トルク指令）
出力: `y = [θ_a, ω_a]`（アーム側の位置・速度）

```
J_m * ω_m' = τ - b_m*ω_m - k_s*(θ_m - θ_a) - b_s*(ω_m - ω_a)
J_a * ω_a' =     - b_a*ω_a + k_s*(θ_m - θ_a) + b_s*(ω_m - ω_a)
θ_m' = ω_m
θ_a' = ω_a
```

### 3.4 Simulinkモデル要件

- ファイル: `models/physical/robot_arm_plant.slx`
- ソルバー: 固定ステップ、Runge-Kutta 4次
- サンプル時間: `Ts_plant = 1/8000` s（8 kHz）
- State-Space ブロックまたはTransfer Functionブロックで実装
- 入力ポート: `tau`（トルク）
- 出力ポート: `theta_a`（アーム角度）, `omega_a`（アーム角速度）, `theta_m`（モータ角度）

---

## 4. 制御器仕様

### 4.1 マルチレート カスケード制御構成

電流ループ（8 kHz）は省略し、トルクが瞬時追従すると仮定する。

```
目標位置 θd
   ↓
[位置P制御]  Ts_pos = 1/200  s  （200 Hz）
   ↓ 速度指令 ωd
[速度PI制御] Ts_vel = 1/1000 s  （1 kHz）
   ↓ トルク指令 τ
[物理モデル]  Ts_plant = 1/8000 s （8 kHz）
   ↓
θ_a, ω_a（フィードバック）
```

### 4.2 制御器パラメータ（初期値）

```matlab
%% 位置ループ (P制御)
Kp_pos = 50;       % [rad/s / rad]

%% 速度ループ (PI制御)
Kp_vel = 0.5;      % [N·m / (rad/s)]
Ki_vel = 5.0;      % [N·m / (rad/s·s)]

%% トルク飽和
tau_max = 5.0;     % [N·m]

%% ノッチフィルタ（30 Hz振動モード抑制）
f_notch  = 30;     % [Hz]
depth_dB = -20;    % [dB]
Q        = 3;      % Q値
```

### 4.3 Simulinkモデル要件

- ファイル: `models/controller/cascade_controller.slx`
- Rate Transition ブロックでマルチレートを明示的に構成する
- 速度ループにアンチワインドアップ付きPI制御を実装する
- PTP指令: ステップ入力またはS字プロファイル（`src/utils/ptp_profile.m`で生成）
- シミュレーション時間: 2.0 s
- ノッチフィルタはオン/オフを `enable_notch` フラグで切替可能にする

---

## 5. NNサロゲートモデル仕様

### 5.1 学習データ生成方針

以下の2種類の励振信号でデータを生成し結合する。

| データ種別 | 内容 | 時間 | 目的 |
|-----------|------|------|------|
| チャープ信号 | 0.5〜150 Hz、振幅±2 N·m | 30 s | 周波数特性全域をカバー |
| PTP軌道 | 様々な移動量・速度 | 20 s × 5本 | 実動作パターンをカバー |

- サンプリング: 物理モデルを1 kHz（Ts_vel）でサンプリングしてデータ化
- データ分割: 学習80% / 検証10% / テスト10%
- 保存先: `data/training/surrogate_train_data.mat`

### 5.2 NNモデル構造

```matlab
%% 入力
% x(k) = [theta_a(k), omega_a(k), theta_m(k), tau(k)]  (4次元)

%% 出力（差分出力で学習）
% delta_x(k) = x(k+1) - x(k)  → 精度向上のため絶対値でなく差分を学習

%% ネットワーク
% 種別: NARX (Nonlinear AutoRegressive with eXogenous input) または MLP
% 隠れ層: [64, 64, 32] ノード
% 活性化関数: tanh
% 出力層: linear
% 損失関数: MSE
```

- ファイル: `src/train_surrogate.m`
- 学習後のネットワークを `data/results/surrogate_model.mat` に保存
- Simulinkブロック化: `models/surrogate/surrogate_block.slx`（MATLAB Functionブロックで実装）

### 5.3 精度評価基準

以下をすべてテストデータで評価し、Issue#4にコメント記録する。

| 指標 | 合格基準 |
|------|---------|
| 位置 RMSE | < 1 mrad |
| 速度 RMSE | < 10 mrad/s |
| 30 Hz応答の位相誤差 | < 20° |
| 1秒間の自由ロールアウト誤差 | 発散しないこと |

---

## 6. ゲイン最適化仕様

### 6.1 最適化の構成

サロゲートをプラントとした閉ループシミュレーションで、
制御ゲイン `[Kp_pos, Kp_vel, Ki_vel]` を最適化する。

### 6.2 コスト関数

```matlab
function cost = eval_cost(gains, surrogate_net)
    Kp_pos = gains(1);
    Kp_vel = gains(2);
    Ki_vel = gains(3);

    % クローズドループシミュレーション実行（1 kHzで2秒間）
    [theta_out, ~] = run_closed_loop_surrogate(gains, surrogate_net);

    % コスト計算
    theta_ref = ptp_profile(0, pi/4, 2.0, 1/1000); % 45度移動
    e = theta_ref - theta_out;

    J_track    = mean(e.^2);                         % 追従誤差
    J_settle   = calc_settling_time(theta_out, pi/4); % 整定時間
    J_overshoot = max(0, max(theta_out) - pi/4) / (pi/4); % オーバーシュート率

    cost = 10*J_track + 5*J_settle + 2*J_overshoot;
end
```

### 6.3 最適化手法

```matlab
% 初期値
x0 = [Kp_pos_init, Kp_vel_init, Ki_vel_init];

% 探索範囲
lb = [10,  0.1, 0.5];
ub = [200, 5.0, 50.0];

% 手法1: 勾配なし（推奨）
options = optimoptions('ga', 'MaxGenerations', 100, 'PopulationSize', 50);
[x_opt, cost_opt] = ga(@(x)eval_cost(x, net), 3, [], [], [], [], lb, ub, [], options);

% 手法2: fminsearch（比較用）
[x_opt2, cost_opt2] = fminsearch(@(x)eval_cost(x, net), x0);
```

---

## 7. GitHub ワークフロー

### 7.1 Issue・ブランチ対応表

以下のIssueをすべて作成してから作業を開始すること。

| Issue# | タイトル | ブランチ名 | 対応Phase |
|--------|---------|-----------|-----------|
| #1 | 物理モデル構築と周波数応答確認 | `feature/physical-model` | Phase 1 |
| #2 | マルチレートカスケード制御器構築 | `feature/cascade-controller` | Phase 2 |
| #3 | サロゲート学習データ生成 | `feature/training-data` | Phase 3 |
| #4 | **NNサロゲート学習とレート別精度比較** | `feature/nn-surrogate-rate-comparison` | Phase 4 |
| #5 | サロゲートによる閉ループ代替検証 | `feature/closed-loop-surrogate` | Phase 5 |
| #6 | ゲイン最適化シミュレーション | `feature/gain-optimization` | Phase 6 |
| #7 | 最適ゲインの物理モデルによる最終検証 | `feature/final-verification` | Phase 7 |

### 7.2 作業手順（各Issueで繰り返す）

```bash
# 1. ブランチ作成
git checkout main
git pull origin main
git checkout -b feature/xxx

# 2. 実装・検証

# 3. コミット（こまめに）
git add .
git commit -m "feat: 〇〇を実装 (#Issue番号)"

# 4. プッシュ
git push origin feature/xxx

# 5. PR作成（マージはユーザーが行う）
gh pr create --title "Phase X: 〇〇" --body "Closes #Issue番号"

# 6. Issue にコメントで結果を記録（後述）
gh issue comment Issue番号 --body "$(cat result_comment.md)"
```

### 7.3 結果記録フォーマット

各IssueのコメントはMarkdown形式で以下の構成とする。

```markdown
## 検証結果

**実施日**: YYYY-MM-DD
**ブランチ**: feature/xxx

### 結果サマリー

| 指標 | 値 | 合否 |
|------|----|------|
| 共振周波数 | 30.2 Hz | ✅ |
| ... | ... | ... |

### 考察

〇〇が確認された。△△については要調査。

### 出力図

![周波数応答](figures/phase1_freq_response.png)
![ステップ応答](figures/phase1_step_response.png)
```

---

## 8. 各Phaseの詳細タスクと出力図

### Phase 1：物理モデル（Issue #1）

**タスク:**
1. `src/params.m` を作成し、全物理パラメータを定義
2. 共振周波数を `(1/(2*pi))*sqrt(k_s*(1/J_m+1/J_a))` で確認し表示
3. `models/physical/robot_arm_plant.slx` を構築
4. 周波数応答（ボード線図）を計算・描画（`linearize` または `tf` で線形化）
5. ステップ応答で30 Hz振動の存在を確認

**出力図（すべてPNGで `figures/` に保存）:**
- `phase1_bode.png` — ボード線図（τ → θ_a）、30 Hz共振を明示
- `phase1_step.png` — ステップ応答（τ = 1 N·m, 0.5 s）

---

### Phase 2：制御器（Issue #2）

**タスク:**
1. `models/controller/cascade_controller.slx` を構築
2. Rate Transition ブロックで200 Hz / 1 kHz / 8 kHz のマルチレートを明示
3. PTP指令（45°, S字プロファイル, 0.5 s移動時間）でシミュレーション
4. ノッチフィルタあり/なしの比較
5. 整定時間・オーバーシュートを計算して記録

**出力図:**
- `phase2_ptp_response.png` — 位置・速度・トルクのタイムライン（4段組）
- `phase2_notch_comparison.png` — ノッチフィルタあり/なしの比較

---

### Phase 3：学習データ生成（Issue #3）

**タスク:**
1. `src/utils/generate_chirp_data.m` — チャープ入力シミュレーション
2. `src/utils/generate_ptp_data.m` — 複数のPTP軌道シミュレーション
3. データを1 kHzにダウンサンプリング（アンチエイリアスフィルタ適用）
4. `data/training/surrogate_train_data.mat` に保存
5. データ統計（分布・パワースペクトル）を確認

**出力図:**
- `phase3_training_data_overview.png` — τ, θ_a, ω_a の時系列サンプル（3段組）
- `phase3_input_spectrum.png` — 入力トルクのパワースペクトル（0〜200 Hz）

---

### Phase 4：NNサロゲート学習とレート別精度比較（Issue #4）

**タスク:**
1. `src/train_surrogate.m` を実装し、以下の4レートでそれぞれ学習を実行する

   | レート | Δt | ブランチ内識別子 |
   |--------|----|----------------|
   | 500 Hz | 2 ms | `rate_500hz` |
   | 1 kHz  | 1 ms | `rate_1khz` |
   | 2 kHz  | 0.5 ms | `rate_2khz` |
   | 8 kHz  | 0.125 ms | `rate_8khz` |

2. 各レートで学習曲線（Train / Validation Loss）を記録
3. テストデータで精度評価（RMSE、位相誤差、ロールアウト安定性）
4. **レート別の比較表**をIssue#4コメントに記録
5. 精度評価基準（Section 5.3）との照合
6. 最良レートのモデルを `data/results/surrogate_best.mat` に保存

**レート別比較で記録する指標:**

| レート | 位置RMSE | 速度RMSE | 30Hz位相誤差 | ロールアウト | 備考 |
|--------|---------|---------|------------|------------|------|
| 500 Hz | | | | | |
| 1 kHz  | | | | | |
| 2 kHz  | | | | | |
| 8 kHz  | | | | |「自明解」への退化を確認 |

**出力図:**
- `phase4_training_curve_all_rates.png` — 4レートの学習曲線を重ね描き
- `phase4_rmse_vs_rate.png` — レート vs RMSE の棒グラフ（本プロジェクトの主要結果）
- `phase4_phase_error_vs_rate.png` — レート vs 30Hz位相誤差
- `phase4_rollout_comparison.png` — 4レートのロールアウト誤差比較
- `phase4_8khz_collapse.png` — 8kHzモデルが自明解に退化する様子（Δx ≈ 0）

---

### Phase 5：閉ループ代替検証（Issue #5）

**タスク:**
1. `models/surrogate/surrogate_block.slx` — サロゲートをSimulinkブロック化
2. 物理モデルをサロゲートに差し替えた閉ループシミュレーション
3. 同じゲイン・同じPTP指令で物理モデルとサロゲートの応答を比較
4. 差異が許容範囲内（位置誤差 < 5%）であることを確認

**出力図:**
- `phase5_comparison.png` — 物理モデル vs サロゲートの応答比較（重ね描き）
- `phase5_error.png` — 位置誤差の時系列

---

### Phase 6：ゲイン最適化（Issue #6）

**タスク:**
1. `src/optimize_gains.m` を実装
2. ga（遺伝的アルゴリズム）で最適ゲインを探索
3. fminsearch でも探索し、結果を比較
4. 最適ゲイン候補を少なくとも3セット抽出
5. 各候補のサロゲートシミュレーション結果を比較

**出力図:**
- `phase6_optimization_convergence.png` — 最適化収束曲線（世代/評価数 vs Cost）
- `phase6_optimal_response.png` — 最適ゲインでのPTP応答（3候補を重ね描き）
- `phase6_pareto.png` — 整定時間 vs オーバーシュートのパレート図（可能であれば）

---

### Phase 7：最終検証（Issue #7）

**タスク:**
1. Phase 6で選んだ最適ゲインを物理モデルに適用
2. 初期ゲインと最適ゲインの応答を比較
3. ノッチフィルタあり/なしでの動作確認
4. 最終的な性能指標をまとめる

**出力図:**
- `phase7_before_after.png` — 最適化前後のPTP応答比較
- `phase7_final_metrics.png` — 性能指標の棒グラフ（整定時間・オーバーシュート・追従誤差）

---

## 9. 共通ルール

### コード品質
- すべての `.m` ファイルの先頭に目的・入出力・作成日のコメントを記載する
- パラメータはハードコードせず `params.m` から読み込む
- Figure はすべて `figures/` に PNG で保存し、`close all` で画面を閉じる

### Figure保存の標準コード

```matlab
function save_figure(fig_handle, filename)
    set(fig_handle, 'PaperPositionMode', 'auto');
    print(fig_handle, fullfile('figures', filename), '-dpng', '-r150');
    close(fig_handle);
end
```

### Issueコメントの投稿

```bash
# Markdown ファイルを作り gh でコメント投稿
cat > result_comment.md << 'EOF'
## 検証結果
...（内容）...
EOF

gh issue comment <番号> --body-file result_comment.md
```

### コミットメッセージ規約

```
feat:  新機能追加
fix:   バグ修正
sim:   シミュレーション実行・結果記録
data:  データ生成・保存
docs:  ドキュメント更新
```

---

## 10. 作業開始チェックリスト

Claude Code は作業開始前に以下を確認すること。

- [ ] GitHubリポジトリのURLを確認し `git clone` する
- [ ] MATLABが `matlab -batch` で起動できることを確認する
- [ ] `gh` コマンド（GitHub CLI）が認証済みであることを確認する
- [ ] Issue #1〜#7 をすべて作成する
- [ ] `figures/` ディレクトリを作成する
- [ ] `src/params.m` を最初に作成し、以降すべてのスクリプトからこれを読み込む

---

*プロジェクト名: `multirate-surrogate-rate-bench`*
*このドキュメントはロボットアームPTP制御におけるNNサロゲートのサンプリングレート検証プロジェクトの要件定義書です。*
*会話の記録・技術的背景は Notion ページを参照してください。*
