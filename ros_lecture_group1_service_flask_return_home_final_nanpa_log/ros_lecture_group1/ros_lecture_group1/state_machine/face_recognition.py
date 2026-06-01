#!/usr/bin/env python3
"""Face recognition states for ros_lecture_group1."""

import time

import rclpy
import yasmin
from rclpy.qos import qos_profile_sensor_data
from yasmin import Blackboard
from yasmin import State
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT

class FaceRecognitionState(State):
    """Face Recognition Stateのひな型。"""

    def __init__(self, node):
        super().__init__(
            outcomes=[NEXT, "patrol", EXCEPT],
        )
        self.node = node
        self.latest_image = None
        self.user_judgment = None
        self._declare_parameters()
        self.max_seat_number = self.node.get_parameter('max_seat_number').value
        self.image_timeout_sec = float(
            self.node.get_parameter('face_recognition.image_timeout_sec').value
        )
        self.judgment_timeout_sec = float(
            self.node.get_parameter(
                'face_recognition.judgment_timeout_sec',
            ).value
        )
        self.image_republish_interval_sec = float(
            self.node.get_parameter(
                'face_recognition.image_republish_interval_sec',
            ).value
        )

        # TODO(山根): カメラ画像を受け取るSubscriberを定義する。
        self.image_sub = self.node.create_subscription(
            msg_type=Image,
            topic="image_raw",
            callback=self.image_callback,
            qos_profile=qos_profile_sensor_data,
        )
        # TODO(山根): スマホへカメラ画像を送信するPublisherを定義する。
        self.smartphone_pub = self.node.create_publisher(
            msg_type=Image, topic="image_to_smartphone", qos_profile=10
        )
        # TODO(山根): タイプ判定結果を受け取るSubscriberを定義する。
        self.judgment_sub = self.node.create_subscription(
            msg_type=Bool, topic="user_judgment", callback=self.judgment_callback,qos_profile=10
        )

    def _declare_parameters(self) -> None:
        """FaceRecognitionで使うパラメータを宣言する。"""
        defaults = {
            'max_seat_number': 0,
            'face_recognition.image_timeout_sec': 15.0,
            'face_recognition.judgment_timeout_sec': 60.0,
            'face_recognition.image_republish_interval_sec': 1.0,
        }
        for name, value in defaults.items():
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, value)

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

    def _wait_for(
        self,
        attr_name: str,
        timeout_sec: float,
        on_wait=None,
    ) -> bool:
        """指定属性に値が入るまでROS callbackを回しながら待つ。"""
        start_time = time.time()
        while getattr(self, attr_name) is None:
            if on_wait is not None:
                on_wait()
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if timeout_sec > 0 and time.time() - start_time > timeout_sec:
                return False
        return True

    def _publish_image_periodically(self) -> None:
        """判定待ち中に画像を再送する。"""
        now = time.time()
        if (
            self.latest_image is not None
            and now - self.last_image_publish_time
            >= self.image_republish_interval_sec
        ):
            self.smartphone_pub.publish(self.latest_image)
            self.last_image_publish_time = now

    def execute(self, blackboard: Blackboard) -> str:
        """Face Recognition State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Face Recognition State started')

        # 前回の席でのデータをリセット
        self.latest_image = None
        self.user_judgment = None

        # 1. カメラ画像を取得するまで待機
        yasmin.YASMIN_LOG_INFO('Waiting for camera image...')
        if not self._wait_for('latest_image', self.image_timeout_sec):
            self.node.get_logger().error('Timed out waiting for camera image.')
            return EXCEPT
        
        # 2. 取得した画像をスマホに送信
        yasmin.YASMIN_LOG_INFO('Sending image to smartphone...')
        self.smartphone_pub.publish(self.latest_image)
        self.last_image_publish_time = time.time()

        # 3. スマホからのユーザー判断(True/False)を待機
        yasmin.YASMIN_LOG_INFO('Waiting for user judgment from smartphone...')
        if not self._wait_for(
            'user_judgment',
            self.judgment_timeout_sec,
            on_wait=self._publish_image_periodically,
        ):
            self.node.get_logger().error('Timed out waiting for user judgment.')
            return EXCEPT

        # 4. 判断結果に基づく処理
        # 現在の席IDを取得
        seat_id = self._blackboard_get(blackboard, 'current_seat_id', None)
        seat_number = self._blackboard_get(
            blackboard,
            'current_seat_number',
            0,
        )
        target_seat = seat_id or seat_number
        if target_seat in (None, 0, '0'):
            target_seat = self._blackboard_get(blackboard, 'current_pose', None)
        
        if self.user_judgment is True:
            yasmin.YASMIN_LOG_INFO('Judgment: True! Saving to blackboard.')
            
            # Blackboardに (なければ)、リスト'target_seats' を作成
            target_seats = self._blackboard_get(blackboard, 'target_seats', [])
            if target_seat not in target_seats:
                target_seats.append(target_seat)
            # 'target_seats'に現在の席番号を保存
            self._blackboard_set(blackboard, 'target_seats', target_seats)
        
        elif self.user_judgment is False:
            yasmin.YASMIN_LOG_INFO('Judgment: False. Moving to next seat.')
        
        # 5. 現在の席が最後(最大席数)かどうかで戻り値を変更する
        scan_is_last = bool(
            self._blackboard_get(blackboard, 'scan_is_last', False),
        )
        try:
            current_number = int(seat_number)
        except (TypeError, ValueError):
            current_number = 0
        max_seat_reached = (
            self.max_seat_number > 0
            and current_number >= int(self.max_seat_number)
        )
        if scan_is_last or max_seat_reached:
            yasmin.YASMIN_LOG_INFO(
                'Face recognition scan completed. Returning home before patrol.'
            )
            self._blackboard_set(blackboard, 'mission_phase', 'return_home')
            self._blackboard_set(blackboard, 'navigation_route', None)
            self._blackboard_set(blackboard, 'patrol_route', None)
            self._blackboard_set(blackboard, 'target_seat_id', None)
            return NEXT
        yasmin.YASMIN_LOG_INFO('Returning N (Navigation) to move next.')
        return NEXT
    
    def image_callback(self, msg: Image):
        """カメラ画像を受け取ったときの処理"""
        # まだ画像を取得していない場合のみ保存する（executeのループを抜けるため）
        if self.latest_image is None:
            self.latest_image = msg

    def judgment_callback(self, msg: Bool):
        """スマホからユーザー判断を受け取ったときの処理"""
        self.user_judgment = msg.data
