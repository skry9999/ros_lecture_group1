#!/usr/bin/env python3
"""Nanpa states for ros_lecture_group1."""

import threading
import time
from werkzeug.serving import make_server

import rclpy

from flask import Flask

from rclpy.node import Node

from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS

from std_msgs.msg import String
from std_srvs.srv import Trigger


# ==========================================================
# Flask Server
# ==========================================================

app = Flask(__name__)

global_node = None
_flask_started = False
_flask_server_ready = False
_flask_server_error = None
_pending_completed_data = None


@app.route("/completed", methods=["GET", "POST"])
def completed():

    print("completed accessed")

    global global_node, _pending_completed_data

    if global_node is not None and hasattr(global_node, "qr_status_pub"):

        msg = String()

        # completed:user1
        msg.data = "completed:user1"

        global_node.qr_status_pub.publish(msg)
        _pending_completed_data = msg.data

        global_node.get_logger().info(
            "Flask: QR completed published"
        )

    else:
        _pending_completed_data = "completed:user1"

    return "ok"


# ==========================================================
# NanpaState
# ==========================================================

class NanpaState(State):
    """Nanpa State."""

    def __init__(self, node):

        super().__init__(
            outcomes=[
                NANPA_SUCCESS,
                NANPA_FAILED,
                EXCEPT
            ]
        )

        self.node = node

        self.scan_completed = False
        self.qr_data = None
        self.pending_qr_data = None

        # ==================================================
        # Publisher
        # ==================================================

        self.result_pub = self.node.create_publisher(
            String,
            "nanpa_result",
            10
        )

        # Flask -> qr_status publish用
        self.node.qr_status_pub = self.node.create_publisher(
            String,
            "qr_status",
            10
        )

        # global_node をセット & Flask を起動（未起動の場合のみ）
        global global_node, _flask_started
        global_node = node
        if not _flask_started:
            _flask_started = True
            flask_thread = threading.Thread(target=run_flask, daemon=True)
            flask_thread.start()
            self._wait_for_flask_ready(timeout_sec=5.0)

        # ==================================================
        # Subscriber
        # ==================================================

        self.subscription = self.node.create_subscription(
            String,
            "qr_status",
            self.listener_callback,
            10
        )

        # ==================================================
        # Service Client
        # ==================================================

        self.announce_client = self.node.create_client(
            Trigger,
            "announce_nanpa_qr"
        )

    def _wait_for_flask_ready(self, timeout_sec: float) -> bool:
        """Flask serverが起動完了するまで短時間待つ。"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if _flask_server_ready:
                return True
            if _flask_server_error is not None:
                return False
            time.sleep(0.1)
        return _flask_server_ready

    # ======================================================
    # QR Callback
    # ======================================================

    def listener_callback(self, msg):

        raw_data = msg.data

        self.node.get_logger().info(
            f"受信: {raw_data}"
        )

        if raw_data.startswith("completed"):

            self.node.get_logger().info(
                f"QR読み込み成功: {raw_data}"
            )

            if ":" in raw_data:

                self.qr_data = raw_data.split(
                    ":",
                    1
                )[1]

            else:

                self.qr_data = "unknown_user"

            self.pending_qr_data = self.qr_data
            self.scan_completed = True

    @staticmethod
    def _blackboard_get(
        blackboard: Blackboard,
        key: str,
        default=None,
    ):
        """Blackboardから値を取得する。"""
        if hasattr(blackboard, 'get'):
            return blackboard.get(key, default)
        try:
            return blackboard[key]
        except (KeyError, TypeError):
            return getattr(blackboard, key, default)

    @staticmethod
    def _blackboard_set(
        blackboard: Blackboard,
        key: str,
        value,
    ) -> None:
        """Blackboardへ値を保存する。"""
        try:
            blackboard[key] = value
        except TypeError:
            setattr(blackboard, key, value)

    def _restore_failed_target(self, blackboard: Blackboard) -> None:
        """QR失敗時に現在の対象席をtarget_seatsへ戻す。"""
        target_seat = self._blackboard_get(blackboard, 'current_seat_id', None)
        if not target_seat:
            target_seat = self._blackboard_get(blackboard, 'target_seat_id', None)
        if not target_seat:
            return

        target_seats = self._blackboard_get(blackboard, 'target_seats', [])
        if not isinstance(target_seats, list):
            target_seats = []
        if target_seat not in target_seats:
            target_seats.insert(0, target_seat)
        self._blackboard_set(blackboard, 'target_seats', target_seats)

    # ======================================================
    # Execute
    # ======================================================

    def execute(self, blackboard: Blackboard):

        self.node.get_logger().info(
            "Nanpa State started"
        )

        self.node.get_logger().info(
            "QRコード読み込み待機中..."
        )

        if not _flask_server_ready:
            self.node.get_logger().error(
                f"Flask server is not ready: {_flask_server_error}"
            )
            self._restore_failed_target(blackboard)
            return EXCEPT

        # 初期化
        global _pending_completed_data
        pending_data = self.pending_qr_data or _pending_completed_data
        if pending_data is not None:
            if str(pending_data).startswith("completed"):
                self.qr_data = str(pending_data).split(":", 1)[1] \
                    if ":" in str(pending_data) else "unknown_user"
            else:
                self.qr_data = pending_data
            self.scan_completed = True
        else:
            self.scan_completed = False
            self.qr_data = None

        # ==================================================
        # 案内Service
        # ==================================================

        if self.announce_client.wait_for_service(
            timeout_sec=1.0
        ):

            req = Trigger.Request()

            self.announce_client.call_async(req)

            self.node.get_logger().info(
                "QR案内Service呼び出し"
            )

            time.sleep(0.5)

        else:

            self.node.get_logger().warn(
                "案内Serviceなし"
            )

        # ==================================================
        # QR待機
        # ==================================================

        timeout_duration = 60.0

        start_time = time.time()

        while not self.scan_completed:

            rclpy.spin_once(
                self.node,
                timeout_sec=0.1
            )

            # timeout
            if (
                time.time() - start_time
            ) > timeout_duration:

                self.node.get_logger().warn(
                    "タイムアウト"
                )

                fail_msg = String()

                fail_msg.data = "FAILED"

                self.result_pub.publish(
                    fail_msg
                )

                self._restore_failed_target(blackboard)
                return NANPA_FAILED

            time.sleep(0.1)

        # ==================================================
        # Success
        # ==================================================

        self.node.get_logger().info(
            "ナンパ成功"
        )
        self.node.get_logger().info(
            f"user: {self.qr_data}"
        )

        self._blackboard_set(blackboard, "target_user_info", self.qr_data)
        self.pending_qr_data = None
        _pending_completed_data = None

        success_msg = String()

        success_msg.data = "SUCCESS"

        self.result_pub.publish(
            success_msg
        )

        return NANPA_SUCCESS


# ==========================================================
# Flask起動
# ==========================================================

def run_flask():
    global _flask_server_ready, _flask_server_error
    try:
        server = make_server("0.0.0.0", 5000, app)
        _flask_server_ready = True
        _flask_server_error = None
        server.serve_forever()
    except Exception as err:
        _flask_server_ready = False
        _flask_server_error = err


# ==========================================================
# Main
# ==========================================================

def main():

    global global_node, _flask_started

    rclpy.init()

    node = Node("nanpa_node")

    global_node = node

    # Flask起動
    flask_thread = threading.Thread(
        target=run_flask,
        daemon=True
    )

    flask_thread.start()
    _flask_started = True

    time.sleep(2)

    # State作成
    state = NanpaState(node)

    blackboard = Blackboard()

    outcome = state.execute(
        blackboard
    )

    print(outcome)

    rclpy.shutdown()
    # global global_node

    # rclpy.init()

    # node = Node("nanpa_node")

    # global_node = node

    # # Flask起動
    # flask_thread = threading.Thread(
    #     target=run_flask,
    #     daemon=True
    # )

    # flask_thread.start()

    # # State作成
    # state = NanpaState(node)

    # blackboard = Blackboard()

    # outcome = state.execute(
    #     blackboard
    # )

    # print(outcome)

    # rclpy.shutdown()


if __name__ == "__main__":

    main()
