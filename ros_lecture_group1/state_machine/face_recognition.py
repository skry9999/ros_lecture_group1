#!/usr/bin/env python3
"""Face recognition states for ros_lecture_group1."""

import yasmin
import time

from yasmin import Blackboard
from yasmin import State
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ros_lecture_group1.state_machine.outcomes import EXCEPT

class FaceRecognitionState(State):
    """Face Recognition Stateのひな型。"""

    def __init__(self, node):
        super().__init__(
            outcomes=["navigation", "patrol", EXCEPT],
        )
        self.node = node
        self.latest_image = None
        self.user_judgment = None
        self.max_seat_number = self.node.get_parameter('max_seat_number').value

        # TODO(山根): カメラ画像を受け取るSubscriberを定義する。
        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.image_callback, qos_profile=10
        )
        # TODO(山根): スマホへカメラ画像を送信するPublisherを定義する。
        self.smartphone_pub = self.node.create_publisher(
            msg_type=Image, topic="image_to_smartphone", qos_profile=10
        )
        # TODO(山根): タイプ判定結果を受け取るSubscriberを定義する。
        self.judgment_sub = self.node.create_subscription(
            msg_type=Bool, topic="user_judgment", callback=self.judgment_callback,qos_profile=10
        )

    def execute(self, blackboard: Blackboard) -> str:
        """Face Recognition State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Face Recognition State started')

        # 前回の席でのデータをリセット
        self.latest_image = None
        self.user_judgment = None

        # 1. カメラ画像を取得するまで待機
        yasmin.YASMIN_LOG_INFO('Waiting for camera image...')
        while self.latest_image is None:
            time.sleep(0.1)
        
        # 2. 取得した画像をスマホに送信
        yasmin.YASMIN_LOG_INFO('Sending image to smartphone...')
        self.smartphone_pub.publish(self.latest_image)

        # 3. スマホからのユーザー判断(True/False)を待機
        yasmin.YASMIN_LOG_INFO('Waiting for user judgment from smartphone...')
        while self.user_judgment is None:
            time.sleep(0.1)

        # 4. 判断結果に基づく処理
        # 現在の席番号を取得
        seat_number = blackboard.get('current_seat_number', 0)
        
        if self.user_judgment is True:
            yasmin.YASMIN_LOG_INFO('Judgment: True! Saving to blackboard.')
            
            # Blackboardに (なければ)、リスト'target_seats' を作成
            if 'target_seats' not in blackboard:
                blackboard['target_seats'] = []
            # 'target_seats'に現在の席番号を保存
            blackboard['target_seats'].append(seat_number)
        
        elif self.user_judgment is False:
            yasmin.YASMIN_LOG_INFO('Judgment: False. Moving to next seat.')
        
        # 5. 現在の席が最後(最大席数)かどうかで戻り値を変更する
        if seat_number == self.max_seat_number:
            yasmin.YASMIN_LOG_INFO('Reached maximum seat number. Returning P (Patrol).')  
            return "patrol"
        else:
            yasmin.YASMIN_LOG_INFO('Returning N (Navigation) to move next.')
            return "navigation"

        return EXCEPT
    
    def image_callback(self, msg: Image):
        """カメラ画像を受け取ったときの処理"""
        # まだ画像を取得していない場合のみ保存する（executeのループを抜けるため）
        if self.latest_image is None:
            self.latest_image = msg

    def judgment_callback(self, msg: Bool):
        """スマホからユーザー判断を受け取ったときの処理"""
        self.user_judgment = msg.data
