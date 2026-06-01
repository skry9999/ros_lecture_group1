#!/usr/bin/env python3
"""Nanpa states for ros_lecture_group1."""

import json
import os
import time
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS


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
        self.latest_image = None
        self.bridge = CvBridge()
        self._declare_parameters()
        self.presence_image_timeout_sec = float(
            self.node.get_parameter(
                'nanpa_presence.image_timeout_sec',
            ).value,
        )
        self.gemini_model = str(
            self.node.get_parameter('nanpa_presence.gemini_model').value,
        )
        self.jpeg_quality = int(
            self.node.get_parameter('nanpa_presence.jpeg_quality').value,
        )
        self.api_key_env = str(
            self.node.get_parameter('nanpa_presence.api_key_env').value,
        )
        self.qr_timeout_sec = float(
            self.node.get_parameter('nanpa.qr_timeout_sec').value,
        )

        # ==================================================
        # Publisher
        # ==================================================

        self.result_pub = self.node.create_publisher(
            String,
            "nanpa_result",
            10
        )

        # ==================================================
        # Subscriber
        # ==================================================

        self.subscription = self.node.create_subscription(
            String,
            "qr_status",
            self.listener_callback,
            10
        )
        self.image_sub = self.node.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )

        # ==================================================
        # Service Client
        # ==================================================

        self.announce_client = self.node.create_client(
            Trigger,
            "announce_nanpa_qr"
        )

    def _declare_parameters(self) -> None:
        """Nanpa開始前の人存在チェックで使うパラメータを宣言する。"""
        defaults = {
            'nanpa_presence.image_timeout_sec': 15.0,
            'nanpa_presence.gemini_model': 'gemini-2.5-flash',
            'nanpa_presence.jpeg_quality': 85,
            'nanpa_presence.api_key_env': 'GEMINI_API_KEY',
            'nanpa.qr_timeout_sec': 15.0,
        }
        for name, value in defaults.items():
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, value)

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

    def image_callback(self, msg: Image):
        """カメラ画像を受け取ったときの処理。"""
        if self.latest_image is None:
            self.latest_image = msg

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
            target_seat = self._blackboard_get(
                blackboard,
                'target_seat_id',
                None,
            )
        if not target_seat:
            return

        target_seats = self._blackboard_get(blackboard, 'target_seats', [])
        if not isinstance(target_seats, list):
            target_seats = []
        if target_seat not in target_seats:
            target_seats.insert(0, target_seat)
        self._blackboard_set(blackboard, 'target_seats', target_seats)

    def _wait_for_image(self, timeout_sec: float) -> Image | None:
        """最新カメラ画像を待つ。"""
        self.latest_image = None
        start_time = time.time()
        while self.latest_image is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if timeout_sec > 0 and time.time() - start_time > timeout_sec:
                return None
        return self.latest_image

    def _image_to_jpeg_bytes(self, image_msg: Image) -> bytes | None:
        """ROS ImageをGeminiへ渡すJPEG bytesに変換する。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            success, encoded = cv2.imencode(
                ".jpg",
                cv_image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
        except Exception as err:
            self.node.get_logger().error(f"Failed to encode camera image: {err}")
            return None

        if not success:
            self.node.get_logger().error("Failed to encode camera image.")
            return None
        return encoded.tobytes()

    def _ask_gemini_person_present(self, image_bytes: bytes) -> bool | None:
        """Geminiへ画像を投げ、人が存在するかをboolで返す。"""
        api_key = os.environ.get(self.api_key_env)
        if not api_key:
            self.node.get_logger().error(
                f"Environment variable {self.api_key_env} is not set."
            )
            return None

        try:
            from google import genai
            from google.genai import types
        except ImportError as err:
            self.node.get_logger().error(
                f"google-genai is not installed: {err}"
            )
            return None

        schema = {
            "type": "object",
            "properties": {
                "person_present": {
                    "type": "boolean",
                },
            },
            "required": ["person_present"],
        }
        prompt = (
            "Decide whether at least one person face is currently visible in "
            "this camera image. Return only the JSON object requested by the "
            "schema."
        )

        try:
            client = genai.Client(api_key=api_key)
            response = client.models.generate_content(
                model=self.gemini_model,
                contents=[
                    types.Part.from_bytes(
                        data=image_bytes,
                        mime_type="image/jpeg",
                    ),
                    prompt,
                ],
                config={
                    "response_mime_type": "application/json",
                    "response_json_schema": schema,
                },
            )
            text = response.text or ""
            result = json.loads(text)
            value = result.get("person_present")
            if isinstance(value, bool):
                return value
            self.node.get_logger().error(
                f"Gemini response did not contain a boolean: {text}"
            )
            return None
        except Exception as err:
            self.node.get_logger().error(f"Gemini person check failed: {err}")
            return None
        finally:
            if 'client' in locals() and hasattr(client, 'close'):
                client.close()

    def _check_person_presence(self) -> bool | None:
        """Nanpa前に対象席に人がいるかを確認する。"""
        self.node.get_logger().info("人存在チェック用の画像を待機中...")
        image_msg = self._wait_for_image(self.presence_image_timeout_sec)
        if image_msg is None:
            self.node.get_logger().error(
                "Timed out waiting for camera image for person check."
            )
            return None

        image_bytes = self._image_to_jpeg_bytes(image_msg)
        if image_bytes is None:
            return None

        self.node.get_logger().info("Geminiで人存在チェック中...")
        return self._ask_gemini_person_present(image_bytes)

    def _publish_failed_result(self, reason: str) -> None:
        """Nanpaを行わずに次席へ進む場合の結果をpublishする。"""
        fail_msg = String()
        fail_msg.data = reason
        self.result_pub.publish(fail_msg)

    def _log_large_nanpa_result(self, title: str) -> None:
        """Nanpa結果をログで大きく目立つように表示する。"""
        border = "=" * 72
        side = "#" * 20
        self.node.get_logger().info(
            "\n"
            f"{border}\n"
            f"{side}        {title}        {side}\n"
            f"{border}"
        )

    # ======================================================
    # Execute
    # ======================================================

    def execute(self, blackboard: Blackboard):

        self.node.get_logger().info(
            "Nanpa State started"
        )

        person_present = self._check_person_presence()
        if person_present is not True:
            self.node.get_logger().info(
                "人が確認できないため、この席のNanpaをスキップします。"
            )
            self.pending_qr_data = None
            self.scan_completed = False
            self.qr_data = None
            self._publish_failed_result("NO_PERSON")
            return NANPA_FAILED

        # 初期化
        self.scan_completed = False
        self.qr_data = None
        self.pending_qr_data = None

        self.node.get_logger().info(
            f"QRコード読み込み待機中... timeout={self.qr_timeout_sec:.1f}s"
        )

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

        timeout_duration = self.qr_timeout_sec

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
                    f"QR読み込みタイムアウト: {timeout_duration:.1f}s"
                )
                self._log_large_nanpa_result("ナンパ失敗")

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

        self._log_large_nanpa_result("ナンパ成功")
        self.node.get_logger().info(
            f"user: {self.qr_data}"
        )

        self._blackboard_set(blackboard, "target_user_info", self.qr_data)
        self.pending_qr_data = None

        success_msg = String()

        success_msg.data = "SUCCESS"

        self.result_pub.publish(
            success_msg
        )

        return NANPA_SUCCESS
