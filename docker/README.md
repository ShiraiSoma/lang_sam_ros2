# GPGPU計算機(gpumng2)でLangSAM検出をリモート実行する

千葉工業大学のAI教育(GPGPU)演習用計算機システム(Slurm + rootless Docker + Harbor)上でLangSAM(GroundingDINO+SAM2)検出を実行し、ロボット側PCとはROS 2トピックで直接やり取りする構成です。

## 全体像

```
[ロボットPC]                                        [gpumng2 計算ノード(Slurmジョブ)]
lang_sam_tracker(Cutie追跡・ローカルGPU)              lang_sam_detector(ROS2ノード)
   │ detect_request(JPEG圧縮画像) ──[ROS2トピック/TCPトランスポート]──▶
   │ detections                  ◀──[ROS2トピック/TCPトランスポート]──
```

- ロボットPC側のROS 2グラフはそのまま(`lang_sam_tracker` + `lang_sam_person_following`)
- `lang_sam_detector`だけをgpumng2の計算ノード上でSlurmジョブとして動かす
- 両者はFast-DDSのTCPトランスポート(マルチキャスト不使用)で直接ROS 2トピック通信する

### なぜ通常のROS 2構成(README記載の「2台構成」)ではダメなのか
gpumng2はセキュリティ上の理由で外部からのインバウンド接続を制限しており、GPU計算はSlurmジョブ経由でしか行えません。ROS 2のデフォルトのノード発見(DDSのマルチキャスト)は別サブネット間では機能しないため、TCPトランスポート限定・ユニキャストの構成にしています(`server_profile.xml` / `client_profile.xml.template`)。

## ファイル一覧

| ファイル | 役割 |
|---|---|
| `Dockerfile`(リポジトリ直下) | ROS 2 Humble + lang_sam推論環境のコンテナイメージ定義 |
| `entrypoint.sh` | コンテナ内で`lang_sam_detector_node.py`を起動するスクリプト |
| `server_profile.xml` | 計算ノード側のFast-DDS設定(TCP:42100で待ち受け) |
| `client_profile.xml.template` | ロボット側のFast-DDS設定テンプレート(`__COMPUTE_NODE_IP__`を実IPに置換して使用) |
| `lang_sam_server.job` | gpumng2へ投入するSlurmジョブスクリプト |
| `start_remote_server.sh` | ジョブ投入〜IP取得までを自動化するスクリプト(ロボットPC側で実行) |
| `connect_client.sh` | 取得したIP宛にプロファイルを生成し、ロボット側ノードを起動するスクリプト |

## 使い方

### 事前準備
- gpumng2のMARINEアカウントでSSHログインできること
- ロボットPCとgpumng2が同一学内ネットワーク上にあること(計算ノードへの直接到達性が前提)
- `~/venv/lang_sam`にロボット側の依存(torch, cutie, supervision等)をインストール済みであること

### 1. サーバー側ジョブの起動
```bash
./docker/start_remote_server.sh <MARINEアカウント>
```
SSHパスワードの入力は最初の1回だけです(接続を使い回します)。ジョブが投入され、RUNNINGになるまで自動待機した後、割り当てられた計算ノードのIPが表示されます。

### 2. ロボット側の起動
```bash
./docker/connect_client.sh <表示されたIP>
```
`~/venv/lang_sam`を有効化し、`lang_sam_tracker` + `lang_sam_person_following`を起動します。

### 3. 終了
- ロボット側: `Ctrl+C`
- サーバー側:
  ```bash
  ssh <MARINEアカウント>@gpumng2.cle.it-chiba.ac.jp squeue -u <MARINEアカウント>
  ssh <MARINEアカウント>@gpumng2.cle.it-chiba.ac.jp scancel <JobID>
  ```

## パラメータの変更

### 検出パラメータ(サーバー側: text_prompt, sam_model, 閾値など)
`lang_sam_server.job`内で環境変数をexportすることで上書きできます(イメージの再ビルド不要)。
```bash
export TEXT_PROMPT="blue cup."
export BOX_THRESHOLD="0.3"
```

### 追跡パラメータ(ロボット側: detection_interval_sec, Cutie設定など)
`lang_sam_executor/config/params.yaml`を編集し、以下を実行してください。
```bash
colcon build --symlink-install --packages-select lang_sam_executor
```

## イメージの更新

コード(`lang_sam_detector`, `docker/entrypoint.sh`等)を変更した場合は、イメージを再ビルド・pushしてください。**同じタグを再pushしてもgpumng2側でキャッシュされ反映されないことがあるため、タグ番号を上げてください**(例: `1.5` → `1.6`)。

```bash
cd ~/lang_sam_server_ws/src/lang_sam_ros2
sudo docker build -t lang_sam_server:test .
sudo docker tag lang_sam_server:test gpumng2.cle.it-chiba.ac.jp/<プロジェクト名>/lang_sam_server:<新タグ>
sudo docker push gpumng2.cle.it-chiba.ac.jp/<プロジェクト名>/lang_sam_server:<新タグ>
```

その後、`lang_sam_server.job`内の`--container=...`のタグ番号も更新してください。

## 運用上の注意

- Slurmジョブは`research`パーティション・最大24時間で投入されます。演習モードに入ると強制終了されるので、その場合は手順1からやり直してください(計算ノードのIPが変わる可能性があります)。
- 検出リクエストの画像はJPEG圧縮して送信しています(無圧縮の生データを送るとTCPトランスポートが詰まってROS 2ノード全体がハングすることがあったため)。
- Harborの自分のプライベートプロジェクトは容量上限15GiBです。不要になった古いタグはHarbor API等で削除してください。
- ローカルでのDockerビルドはディスクを大きく消費します(1イメージあたり数GB)。`sudo docker builder prune -af` / `sudo docker image prune -af`で定期的に整理してください。
