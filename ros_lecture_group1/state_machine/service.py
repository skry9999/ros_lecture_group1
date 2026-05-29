#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_srvs.srv import Trigger


class AnnounceNode(Node):

    def __init__(self):

        super().__init__(
            "announce_nanpa_qr_server"
        )

        self.srv = self.create_service(
            Trigger,
            "announce_nanpa_qr",
            self.callback
        )

        self.get_logger().info(
            "announce_nanpa_qr service ready"
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

        response.message = (
            "announce success"
        )

        return response


def main():

    rclpy.init()

    node = AnnounceNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":

    main()