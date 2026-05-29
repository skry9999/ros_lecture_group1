#!/usr/bin/env python3
"""Nanpa states for ros_lecture_group1."""

import threading
import time

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


@app.route("/completed", methods=["GET", "POST"])
def completed():

    print("completed accessed")

    global global_node

    if global_node is not None:

        msg = String()

        # completed:user1
        msg.data = "completed:user1"

        global_node.qr_status_pub.publish(msg)

        global_node.get_logger().info(
            "Flask: QR completed published"
        )

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

        # global_node をセット & Flask を起動（未起動の場合のみ）
        global global_node, _flask_started
        global_node = node
        if not _flask_started:
            _flask_started = True
            flask_thread = threading.Thread(target=run_flask, daemon=True)
            flask_thread.start()
            time.sleep(1)

        self.scan_completed = False
        self.qr_data = None

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

        # ==================================================
        # Service Client
        # ==================================================

        self.announce_client = self.node.create_client(
            Trigger,
            "announce_nanpa_qr"
        )

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

            self.scan_completed = True

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

        # 初期化
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

        blackboard[
            "target_user_info"
        ] = self.qr_data

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
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False
    )


# ==========================================================
# Main
# ==========================================================

def main():

    global global_node

    rclpy.init()

    node = Node("nanpa_node")

    global_node = node

    # Flask起動
    flask_thread = threading.Thread(
        target=run_flask,
        daemon=True
    )

    flask_thread.start()

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
