# lang_sam_ros2

---

## 構成パッケージ

| パッケージ | 役割 |
|---|---|
| `lang_sam_detector` | LangSAM(GroundingDINO+SAM2)による検出専用ノード。GPU負荷が大きい。 |
| `lang_sam_tracker` | Cutie(VOS)によるマスク伝播トラッキングノード。検出結果を初期値に毎フレーム追跡する。 |
| `lang_sam_person_following` | トラッキング結果を使った人追従制御ノード。 |
| `lang_sam_msgs` | 検出・トラック情報を表すカスタムメッセージ定義。 |
| `lang_sam_executor` | 各構成向けのlaunchファイル・パラメータ(`config/params.yaml`)をまとめたパッケージ。 |
| `docker/` | GPGPU計算機(gpumng2)上でLangSAM検出をリモート実行するための一式(後述)。 |

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

### 2. 2台構成(同一LAN上のリモートPCで検出を動かす場合)
LangSAM検出をGPU負荷の大きいリモートPCで、Cutie追跡と追従制御をロボット側PCで動かす構成です。両PCが同一LAN上にあり、同一の`ROS_DOMAIN_ID`を使用していることが前提です(DDSの標準的なマルチキャスト探索を利用するため、異なるネットワーク越しの接続には別途VPNやDiscovery Serverなどの追加設定が必要です)。

リモートPC(GPU側、`lang_sam_detector`パッケージのみビルドすればよい):
```bash
export ROS_DOMAIN_ID=<両PC共通のID>
ros2 launch lang_sam_executor lang_sam_detector.launch.py
```

ロボット側PC(`lang_sam_tracker`, `lang_sam_person_following`):
```bash
export ROS_DOMAIN_ID=<両PC共通のID>
ros2 launch lang_sam_executor lang_sam_tracker.launch.py
```

### 3. GPGPU計算機(gpumng2)上で検出を動かす場合
自前のGPUマシンを用意できない場合、大学のGPGPU計算機システム(Slurm+Docker、`gpumng2.cle.it-chiba.ac.jp`)上でLangSAM検出をジョブとして動かし、ロボットPCとはネットワーク越しに直接ROS 2トピックでやり取りする構成です。詳しくは [docker/README.md](docker/README.md) を参照してください。

概要:
```bash
# 1. サーバー側(gpumng2)にジョブを投入し、割り当てられたIPを取得
./docker/start_remote_server.sh <MARINEアカウント>

# 2. 表示されたIPでロボット側(Cutie追跡+追従)を起動
./docker/connect_client.sh <表示されたIP>
```

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

各パラメータの詳細な説明は`config/params.yaml`内のコメントを参照してください。

---

## License
This project is licensed under the Apache 2.0 License
