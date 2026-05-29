# ros_lecture_group1

ROS 2 / Yasmin を使った状態機械パッケージです。Nav2で席を巡回し、各席で画像をPC側に送り、人手でタイプ判定を行います。タイプ判定された席をあとで巡回し、対象席でQR読み取り完了を待ちます。

## 現在の状態遷移

実装上の主な流れは以下です。

```text
Navigation
  -> FaceRecognition
  -> Navigation
  -> FaceRecognition
  ...
  -> Patrol
  -> Navigation
  -> Nanpa
  -> Patrol
  -> Finish
```

- `Navigation`
  - Nav2 の `/navigate_to_pose` action に goal を送ります。
  - 通常巡回時は `face_recognition` outcome で `FaceRecognition` へ進みます。
  - `Patrol` が選んだ対象席へ移動した後は `nanpa` outcome で `Nanpa` へ進みます。

- `FaceRecognition`
  - `image_raw` から画像を受け取り、`image_to_smartphone` へ送ります。
  - PC側の `human_judgment` ノードから `user_judgment` を受けます。
  - `True` の席を `target_seats` としてBlackboardに保存します。

- `Patrol`
  - `target_seats` から次に向かう対象を選びます。
  - 現在位置と席座標が分かる場合は近い順に選びます。
  - 次の対象がなければ `Finish` へ進みます。

- `Nanpa`
  - Flask server を `0.0.0.0:5000` で起動します。
  - `http://<robot-ip>:5000/completed` が叩かれるとQR完了扱いになります。
  - timeout時は対象席を `target_seats` に戻し、再試行できるようにしています。

## 主要ファイル

- `ros_lecture_group1/task_node.py`
  - Yasmin の state machine を組み立てます。

- `ros_lecture_group1/state_machine/navigation.py`
  - Nav2 action client、巡回route生成、到着情報の保存を担当します。

- `ros_lecture_group1/state_machine/face_recognition.py`
  - カメラ画像受信、PC側への画像送信、判定結果受信を担当します。

- `ros_lecture_group1/state_machine/patrol.py`
  - タイプ判定済み対象の巡回順を決めます。

- `ros_lecture_group1/state_machine/nanpa.py`
  - QR案内service呼び出し、Flask `/completed`、QR完了待ちを担当します。

- `ros_lecture_group1/human_judgment.py`
  - PC側で画像を表示し、`y` / `n` キー入力で判定を返します。

- `ros_lecture_group1/state_machine/service.py`
  - `announce_nanpa_qr` service のサンプル実装です。

- `config/params.yaml`
  - 巡回座標、timeout、Nav2 action名などを設定します。

## 実行前に必要なもの

以下が起動している必要があります。

- ROS 2 環境
- Nav2
- `NavigateToPose` action server
- robot localization / AMCL など、必要な自己位置推定
- `image_raw` をpublishするカメラノード
- PC側でGUIを表示できる環境
- `human_judgment` ノード
- 必要に応じて `announce_nanpa_qr` service ノード

依存パッケージは `package.xml` に記載しています。主な依存は以下です。

- `rclpy`
- `nav2_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `yasmin`
- `yasmin_ros`
- `cv_bridge`
- `python3-opencv`
- `python3-flask`
- `python3-werkzeug`

## 必ず埋めるべき設定

`config/params.yaml` の `navigation.seats_json` または `navigation.scan_route_json` を実環境に合わせて設定してください。デフォルトのままだと `home` 1地点だけを使う最小動作になります。

推奨は、各地点に必ず `id`, `x`, `y`, `yaw` を持たせることです。

```yaml
task_node:
  ros__parameters:
    max_seat_number: 0
    face_recognition:
      image_timeout_sec: 15.0
      judgment_timeout_sec: 60.0
      image_republish_interval_sec: 1.0
    navigation:
      action_name: /navigate_to_pose
      frame_id: map
      timeout_sec: 30.0
      action_server_timeout_sec: 10.0
      goal_response_timeout_sec: 10.0
      seats_json: '{"1": {"id": "1", "x": 1.0, "y": 0.5, "yaw": 0.0}, "2": {"id": "2", "x": 2.0, "y": 0.5, "yaw": 0.0}}'
      scan_route_json: '["1", "2"]'
      default_route_json: '[]'
      default_goal_json: '{"id": "home", "x": 0.0, "y": 0.0, "yaw": 0.0}'
```

`scan_route_json` には以下のどちらかを入れられます。

- 席IDのリスト: `["1", "2", "3"]`
- 座標dictのリスト: `[{"id": "1", "x": 1.0, "y": 0.5, "yaw": 0.0}]`

座標dictを使う場合も、巡回後の再訪問やNanpa失敗時の復元のため、`id` を必ず付けてください。

## パラメータ

- `max_seat_number`
  - 数字席IDを使う場合の最後の席番号です。
  - `0` の場合は `scan_route_json` の終端で巡回終了を判定します。

- `face_recognition.image_timeout_sec`
  - `image_raw` の画像を待つ最大秒数です。

- `face_recognition.judgment_timeout_sec`
  - `user_judgment` を待つ最大秒数です。

- `face_recognition.image_republish_interval_sec`
  - 判定待ち中に `image_to_smartphone` へ画像を再送する間隔です。

- `navigation.action_name`
  - Nav2 action名です。通常は `/navigate_to_pose` です。

- `navigation.frame_id`
  - goal pose のframeです。通常は `map` です。

- `navigation.timeout_sec`
  - Nav2移動完了待ちの最大秒数です。

- `navigation.action_server_timeout_sec`
  - Nav2 action server が見つかるまで待つ最大秒数です。

- `navigation.goal_response_timeout_sec`
  - goal送信後、accept/reject応答を待つ最大秒数です。

- `navigation.seats_json`
  - 席IDから座標への対応表です。

- `navigation.scan_route_json`
  - 最初に顔判定で巡回する順序です。

- `navigation.default_goal_json`
  - `seats_json` も `scan_route_json` も空の場合に使うfallback goalです。

## ビルド

ワークスペースのルートで実行します。

```bash
colcon build --packages-select ros_lecture_group1
source install/setup.bash
```

## 起動手順

1. Nav2、地図、自己位置推定、カメラを起動します。

2. PC側の判定ノードを起動します。

```bash
ros2 run ros_lecture_group1 human_judgment
```

画像ウィンドウで以下を押します。

- `y`: タイプ判定 True
- `n`: タイプ判定 False

3. QR案内serviceを使う場合は起動します。

```bash
ros2 run ros_lecture_group1 service
```

このserviceが無い場合でも、`Nanpa` は警告を出してQR待機に進みます。

4. State Machineを起動します。

```bash
ros2 launch ros_lecture_group1 ros_lecture_group1.launch.py
```

5. Nanpa中にQR完了を通知します。

スマホや別端末から以下へアクセスします。

```text
http://<robot-ip>:5000/completed
```

現在の実装では、完了ユーザーは固定で `user1` として扱われます。

## Topic / Service

### Subscribe

- `image_raw` (`sensor_msgs/msg/Image`)
  - `FaceRecognition` が購読します。

- `user_judgment` (`std_msgs/msg/Bool`)
  - `FaceRecognition` が購読します。

- `/navigation/command` (`std_msgs/msg/String`)
  - `Navigation` が購読します。
  - `stop` / `cancel` で停止扱いになります。

- `/navigation/goal` (`std_msgs/msg/String`)
  - `Navigation` が購読します。
  - 席ID文字列、または `{"x": 1.0, "y": 2.0, "yaw": 0.0}` のJSONを受けられます。

- `/navigation/people_detected` (`std_msgs/msg/Bool`)
  - 人検出通知用です。現在はstatus publishのみです。

- `/amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
  - `Patrol` が現在位置取得に使います。

- `qr_status` (`std_msgs/msg/String`)
  - `Nanpa` がQR完了通知を受けます。

### Publish

- `image_to_smartphone` (`sensor_msgs/msg/Image`)
  - `FaceRecognition` がPC側判定ノードへ画像を送ります。

- `/navigation/status` (`std_msgs/msg/String`)
  - `Navigation` の状態をpublishします。

- `/navigation/current_seat` (`std_msgs/msg/String`)
  - 到着した席IDをpublishします。

- `nanpa_result` (`std_msgs/msg/String`)
  - `SUCCESS` / `FAILED` をpublishします。

### Service

- `announce_nanpa_qr` (`std_srvs/srv/Trigger`)
  - `Nanpa` がQR案内用に呼びます。

## Blackboardで使う主な値

- `scan_index`
  - 顔判定巡回中の現在indexです。

- `scan_is_last`
  - 今の顔判定地点が最後かどうかです。

- `mission_phase`
  - `scan` または `patrol` です。

- `current_pose`
  - 到着したgoalの座標です。

- `current_seat_id`
  - 到着した席IDです。goalに `id` がある場合に保存されます。

- `target_seats`
  - `FaceRecognition` でTrue判定された対象リストです。

- `visited_goals`
  - 一度到着したgoal座標を席IDごとに保存します。

- `target_user_info`
  - Nanpa成功時のユーザー情報です。

## 起こりそうな問題と対処

### Nav2 action server が見つからない

`/navigation/status` に `action_server_unavailable` が出ます。

- Nav2が起動しているか確認してください。
- `navigation.action_name` が実際のaction名と一致しているか確認してください。

### Navigation が timeout する

`/navigation/status` に `timeout` が出ます。

- goal座標が地図上で到達可能か確認してください。
- `navigation.timeout_sec` を長くしてください。
- Nav2のcostmap、localization、TFを確認してください。

### 画像が来ない

`FaceRecognition` が `Timed out waiting for camera image.` を出します。

- `image_raw` がpublishされているか確認してください。
- カメラtopic名が違う場合はコード側のtopic名を合わせてください。
- QoSは `qos_profile_sensor_data` を使っています。

### PC側の判定画面が出ない

- `ros2 run ros_lecture_group1 human_judgment` を起動してください。
- GUIが使えるPC上で実行してください。
- SSH越しの場合はX転送やディスプレイ設定が必要です。
- `cv_bridge` / OpenCV が入っているか確認してください。

### `user_judgment` が返らない

`FaceRecognition` は `judgment_timeout_sec` 秒でtimeoutします。

- PC側ウィンドウで `y` または `n` を押してください。
- `ros2 topic echo /user_judgment` ではなく、topic名は相対名の `user_judgment` です。名前空間を付けて起動している場合は実topic名を確認してください。

### QR完了にならない

- Nanpaに入ってから `http://<robot-ip>:5000/completed` にアクセスしてください。
- port `5000` が他プロセスに使われていないか確認してください。
- ロボットPCのfirewallやネットワーク到達性を確認してください。

### QR timeout後に同じ対象へ戻る

仕様です。timeout時は現在対象を `target_seats` に戻し、再試行できるようにしています。

### 1地点だけ回って終了する

`seats_json` と `scan_route_json` が空だと `default_goal_json` の `home` だけを使います。複数席を回る場合は必ず `scan_route_json` を設定してください。

### idなしgoalで後段が不安定になる

座標dictに `id` が無い場合、巡回後の対象復元や再訪問が不安定になります。`scan_route_json` や `seats_json` のgoalには必ず `id` を入れてください。

## 既知の制約

- 顔認識は自動判定ではなく、PC側の人手判定です。
- QR完了ユーザーは現在 `user1` 固定です。
- Flask serverは `0.0.0.0:5000` 固定です。
- `image_raw`, `image_to_smartphone`, `user_judgment`, `qr_status` は相対topic名です。namespace付きで起動する場合は実topic名に注意してください。
- 実機でのNav2起動、カメラ、OpenCV GUI、QRアクセスまではこのリポジトリ単体では検証できません。

## 開発時の簡易確認

Python構文と空白差分の確認:

```bash
PYTHONPYCACHEPREFIX=/tmp/ros_lecture_group1_pycache python3 -m compileall -q .
git diff --check
```

ROS環境がある場合は、ビルド後に以下も確認してください。

```bash
colcon build --packages-select ros_lecture_group1
source install/setup.bash
ros2 launch ros_lecture_group1 ros_lecture_group1.launch.py
```
