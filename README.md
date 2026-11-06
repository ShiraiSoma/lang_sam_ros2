# lang_sam_ros2

---

## 構成パッケージ

| パッケージ | 役割 |
|---|---|
| `lang_sam_detector` | LangSAM(GroundingDINO+SAM2)による検出専用ノード。GPU負荷が大きい。 |
| `lang_sam_tracker` | Cutie(VOS)によるマスク伝播トラッキングノード。検出結果を初期値に毎フレーム追跡する。GPU使用。 |
| `lang_sam_client` | ロボット側で動くクライアント。カメラ画像をJPEG圧縮して追跡ノードへ送り、返ってきた検出・追跡結果を手元で描画・表示する。GPU不要。 |
| `lang_sam_person_following` | トラッキング結果を使った人追従制御ノード。 |
| `lang_sam_ground_nav` | トラッキング結果を使った地面追従+障害物回避ノード(`/cmd_vel`)。リモートPC側で動かす(後述の4.)。 |
| `lang_sam_msgs` | 検出・トラック情報を表すカスタムメッセージ定義。 |
| `lang_sam_executor` | 各構成向けのlaunchファイル・パラメータ(`config/params.yaml`)と、検出+追跡を1プロセスで起動するサーバー(`lang_sam_server_node.py`)をまとめたパッケージ。 |
| `docker/` | GPGPU計算機(gpumng2)上でLangSAM検出+Cutie追跡をリモート実行するための一式(後述)。 |

データの流れ:
```
[ロボット側]                                      [GPU側]
camera ─▶ lang_sam_client ── /lang_sam/frame (JPEG) ──▶ lang_sam_tracker (Cutie) ◀─▶ lang_sam_detector (LangSAM)
          (描画・表示)    ◀── /lang_sam/tracks, /lang_sam/track_mask (PNG), /lang_sam/detections ──
lang_sam_person_following ◀── /lang_sam/tracks
```
結果には送信したフレームと同じ`header.stamp`が付いて返ってくるので、クライアントは手元に保持している元フレームに結果を重ねて表示します(結果画像そのものは転送しません)。

---

## インストール

### lang-segment-anythingについて
本パッケージは [lang-segment-anything](https://github.com/luca-medeiros/lang-segment-anything) をベースにしたコードを`lang_sam_detector/lang_sam/`に同梱しています。別途pip installする必要はありません。

### このリポジトリのインストール
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/open-rdc/lang_sam_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 起動方法

用途に応じて3通りの構成があります。

### 1. 単一マシンで動作確認する場合
検出・追跡・追従を1台のPC上で全て動かします。GPUを1台のPCに搭載している必要があります。

```bash
ros2 launch lang_sam_executor lang_sam_executor.launch.py
```

### 2. 2台構成(同一LAN上のリモートPCで検出・追跡を動かす場合)
LangSAM検出とCutie追跡をGPUを持つリモートPCで、フレーム送信・結果表示と追従制御をロボット側PCで動かす構成です。ロボット側PCにGPUは不要です。両PCが同一LAN上にあり、同一の`ROS_DOMAIN_ID`を使用していることが前提です(DDSの標準的なマルチキャスト探索を利用するため、異なるネットワーク越しの接続には別途VPNやDiscovery Serverなどの追加設定が必要です)。

リモートPC(GPU側、`lang_sam_msgs`, `lang_sam_detector`, `lang_sam_tracker`, `lang_sam_executor`をビルド):
```bash
export ROS_DOMAIN_ID=<両PC共通のID>
ros2 launch lang_sam_executor lang_sam_server.launch.py
```

ロボット側PC(`lang_sam_client`, `lang_sam_person_following`):
```bash
export ROS_DOMAIN_ID=<両PC共通のID>
ros2 launch lang_sam_executor lang_sam_client.launch.py
```

### 3. GPGPU計算機(gpumng2)上で検出・追跡を動かす場合
自前のGPUマシンを用意できない場合、大学のGPGPU計算機システム(Slurm+Docker、`gpumng2.cle.it-chiba.ac.jp`)上でLangSAM検出とCutie追跡をジョブとして動かし、ロボットPCとはネットワーク越しに直接ROS 2トピックでやり取りする構成です。詳しくは [docker/README.md](docker/README.md) を参照してください。

概要:
```bash
# 1. サーバー側(gpumng2)にジョブを投入し、割り当てられたIPを取得
./docker/start_remote_server.sh <MARINEアカウント>

# 2. 表示されたIPでロボット側(フレーム送信・結果表示+追従)を起動
./docker/connect_client.sh <表示されたIP>
```

### 4. 地面追従+障害物回避をリモートPCで行う場合(ロボット側はROS1)
検出・追跡に加えて、地面追従+障害物回避(`lang_sam_ground_nav`)もリモートPCで動かし、速度指令だけを
ロボットへ送る構成です。ロボット側(ROS1 Noetic)は[lang_sam_ros1](https://github.com/ShiraiSoma/lang_sam_ros1)の
`lang_sam_client`のみ起動し、速度指令を含むトピックは`ros1_bridge`(`parameter_bridge`)で中継します。

リモートPC:
```bash
export ROS_DOMAIN_ID=<ブリッジと共通のID>
ros2 launch lang_sam_executor lang_sam_ground_nav.launch.py   # detector + tracker + ground_nav (/icart_mini/cmd_velへ出力)
```

- 検出対象は`lang_sam_detector`の`text_prompt`(既定: `"red ground. green pylon."`)、追従/回避に使うラベルは
  `lang_sam_ground_nav`の`ground_label`/`obstacle_label`で指定します(トラックのラベルにこの文字列を含むものを使用。
  同じラベルが複数あればbbox面積が最大のもの)。
- 地面の追従目標は`ground_target_mode`で切り替えます。`bbox_center`は地面bboxの中心、`mask_bottom`は地面マスクの
  下部(ロボットに近い側)の重心です。曲がった通路では`mask_bottom`の方が足元の向きに沿って曲がります。
- `tracks_timeout_sec`(既定0.5秒)の間`/lang_sam/tracks`が届かなければ停止指令を出します。ただし通信そのものが
  切れた場合はロボットに届かないため、足回り側にも指令タイムアウトでの停止機能があることが望ましいです。
- 速度指令の出力先は既定で`/icart_mini/cmd_vel`(ORNE-box/iCart miniの足回りが購読するトピック)です。
  別のロボットでは`cmd_vel_topic:=/cmd_vel`のように指定し、`lang_sam_ros1`の`bridge/bridge_topics.yaml`も合わせてください。
- `lang_sam_person_following`も`/cmd_vel`を出すので、同時に起動しないでください。

---

## パラメータ

主要なパラメータは`lang_sam_executor/config/params.yaml`にまとまっています。

| パラメータ | 説明 |
|---|---|
| `sam_model` | 使用するSAM2モデル(既定: `sam2.1_hiera_small`) |
| `text_prompt` | 検出対象を指定するテキストプロンプト(例: `"red pylon."`) |
| `box_threshold` / `text_threshold` | 検出の信頼度閾値。上げるほど厳格 |
| `detection_interval_sec` | 検出リクエストを送信する間隔(秒) |
| `detect_timeout_multiplier` | 検出応答のタイムアウト = `detection_interval_sec × この倍率` |
| `cutie_*` | Cutie(VOS)のメモリ・解像度・精度に関する設定 |
| `merge_iou_threshold` / `duplicate_overlap_threshold` | 検出とトラックのマージ・重複判定の閾値 |
| `jpeg_quality` / `stream_max_fps`(`lang_sam_client`) | 追跡ノードへ送るフレームのJPEG品質と送信レート上限。回線が細い場合に下げる |

各パラメータの詳細な説明は`config/params.yaml`内のコメントを参照してください。

---

## License
This project is licensed under the Apache 2.0 License
