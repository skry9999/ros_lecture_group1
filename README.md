# ros_lecture_group1

`spec.md` にもとづくROS2パッケージのひな型です。
このリポジトリでは、具体的な処理の中身はまだ実装していません。
各担当者が担当Stateファイルの `execute()` に処理を追加してください。

## パッケージ構成

- `ros_lecture_group1/sm_main.py`
  - 担当: 榊原, 山根, 森, 福田
  - 互換用のエントリーポイントです。
  - 処理はここには書きません。

- `ros_lecture_group1/task_node.py`
  - Yasminを使ったState Machineの遷移を組み立てて起動します。
  - ROS2の `Node` はここで1つ作成し、各 `State` に渡して使います。

- `ros_lecture_group1/state_machine/navigation.py`
  - `NavigationState` を定義します。

- `ros_lecture_group1/state_machine/face_recognition.py`
  - `FaceRecognitionState` を定義します。

- `ros_lecture_group1/state_machine/patrol.py`
  - `PatrolState` を定義します。

- `ros_lecture_group1/state_machine/nanpa.py`
  - `NanpaState` を定義します。

- `ros_lecture_group1/state_machine/standard.py`
  - `FinishState`, `ExceptionState` を定義します。

- `ros_lecture_group1/state_machine/outcomes.py`
  - State間で使うoutcome名を定義します。

- `launch/ros_lecture_group1.launch.py`
  - State Machineノードを起動するlaunchファイルです。

- `config/params.yaml`
  - State Machineノードと各Stateで使うパラメータを書くファイルです。

## 想定State

- `Navigation`
  - ライブ前・ライブ後に席へナビゲーションし、会場内を巡回する。

- `Face_Recognition`
  - 席にいる人の顔を認識し、タイプの顔かどうかを判定する。

- `Patrol`
  - タイプと判定された人の席情報をもとに、現在位置から近い順に巡回する。

- `Nanpa`
  - QRコードを読み取らせ、読み取られたら成功として完了通知を出す。

## Yasminでの実装方針

`task_node.py` にYasminの `StateMachine` を作成しています。
各Stateは `yasmin.State` を継承し、`execute(blackboard)` の中に各状態の入口処理を書きます。
Nodeを継承したクラスは作らず、各Stateの `__init__` で共通のROS2 `Node` を受け取ります。

例:

```python
class NavigationState(State):
    def __init__(self, node):
        super().__init__(outcomes=["next", "except"])
        self.node = node

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Navigation State started")
        return "next"
```

現時点では具体処理は実装せず、以下のような遷移名だけを用意しています。

- `Navigation`
  - 成功: `FaceRecognition`
  - 例外: `Exception`

- `FaceRecognition`
  - タイプの人を見つけた: `Navigation`
  - 見つからなかった: `Navigation`
  - 例外: `Exception`

- `Patrol`
  - 次の対象がいる: `Nanpa`
  - 対象が残っていない: `Finish`
  - 例外: `Exception`

- `Nanpa`
  - QRコード読み取り成功: `Patrol`
  - 失敗またはタイムアウト: `Patrol`
  - 例外: `Exception`

`blackboard` には、各State間で共有したい情報を書きます。
例として、現在位置、席ID、タイプ判定された対象者リスト、次の目的地、ナンパ結果などを入れる想定です。

## ビルド

ワークスペースのルートで以下を実行します。

```bash
colcon build --packages-select ros_lecture_group1
source install/setup.bash
```

## 起動

```bash
ros2 launch ros_lecture_group1 ros_lecture_group1.launch.py
```

## 実装時に決めること

- 各State内でTopic, Service, Actionのどれを使うか。
- 各メッセージ型を標準メッセージで足りる形にするか、独自msg/srv/actionを作るか。
- 会場マップと席情報のファイル形式。
- 顔認識の入力画像Topic名。
- QRコード読み取り結果の受け取り方。
- Yasmin State Machineをライブ前とライブ後でどう切り替えるか。
