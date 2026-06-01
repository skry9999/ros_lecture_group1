#!/usr/bin/env python3

import threading

import rclpy
from flask import Flask
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from werkzeug.serving import make_server


app = Flask(__name__)
_flask_node = None
_flask_server_ready = False
_flask_server_error = None


@app.route("/completed", methods=["GET", "POST"])
def completed():
    """QRコードのURLが開かれたときにNanpaStateへ完了通知を送る。"""
    global _flask_node

    if _flask_node is None:
        return "server not ready", 503

    msg = String()
    msg.data = "completed:user1"
    _flask_node.qr_status_pub.publish(msg)
    _flask_node.get_logger().info(
        "Flask: QR completed published: completed:user1"
    )

    return "ok"


class AnnounceNode(Node):

    def __init__(self):

        super().__init__(
            "announce_nanpa_qr_server"
        )

        global _flask_node
        _flask_node = self

        self.qr_status_pub = self.create_publisher(
            String,
            "qr_status",
            10
        )

        self.srv = self.create_service(
            Trigger,
            "announce_nanpa_qr",
            self.callback
        )

        self._flask_thread = threading.Thread(
            target=self.run_flask,
            daemon=True
        )
        self._flask_thread.start()

        self.get_logger().info(
            "announce_nanpa_qr service ready"
        )
        self.get_logger().info(
            "Flask server starting on 0.0.0.0:5000"
        )

    def run_flask(self):
        """QR完了通知用のFlask serverを起動する。"""
        global _flask_server_ready, _flask_server_error

        try:
            server = make_server(
                "0.0.0.0",
                5000,
                app
            )
            _flask_server_ready = True
            _flask_server_error = None
            self.get_logger().info(
                "Flask server ready on 0.0.0.0:5000"
            )
            server.serve_forever()
        except Exception as err:
            _flask_server_ready = False
            _flask_server_error = err
            self.get_logger().error(
                f"Flask server failed: {err}"
            )

    def callback(
        self,
        request,
        response
    ):

        # ここに音声案内を書く
        self.get_logger().info(
            "QRコードを読んでください！"
        )

        response.success = True
        if _flask_server_ready:
            response.message = (
                "announce success"
            )
        else:
            response.message = (
                f"announce success, but flask not ready: {_flask_server_error}"
            )

        return response


def main():

    rclpy.init()

    node = AnnounceNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
