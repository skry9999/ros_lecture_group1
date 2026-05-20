#!/usr/bin/env python3
"""Nanpa states for ros_lecture_group1."""

import time
import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS
from std_msgs.msg import String
from std_srvs.srv import Trigger
class NanpaState(State):
    """Nanpa Stateのひな型。"""

    def __init__(self, node):
        super().__init__(outcomes=[NANPA_SUCCESS, NANPA_FAILED, EXCEPT])
        self.node = node

        self.scan_completed = False
        self.qr_data = None
    #サブスクライバーの定義
        self.subscription = self.node.create_subscription(
            String,
            "qr_status",
            self.listener_callback,
            10
        )
    #パブリッシャーの定義   
        self.result_pub = self.node.create_publisher(
            String,
            "nanpa_result",
            10
        )
    # 例：ロボットに「QR読んで！」と発話させる案内用サービス
        self.announce_client = self.node.create_client(
                Trigger, 
                "announce_nanpa_qr"
        )
    def listener_callback(self, msg):
        """相手がQRコードを読み込んで、Webページ等でアクションを起こした時の処理"""
        
        raw_data = msg.data
        if raw_data.startswith("completed"):
            self.node.get_logger().info(f"相手がQRコードを読み込みました！受信データ: {raw_data}")
            
            # 文字列を分解して、QRコードの具体的な中身（ユーザーIDなど）を抽出する
            # 例: "completed:ID123" だったら ":" で分けて後ろ側を取得
            if ":" in raw_data:
                self.qr_data = raw_data.split(":", 1)[1]
            else:
                self.qr_data = "unknown_user" # 中身がない場合のフォールバック
                
            self.scan_completed = True

    # def execute(self, blackboard):
    #     self.node.get_logger().info("Nanpa State started")
    #     return EXCEPT


    def execute(self, blackboard: Blackboard) -> str:
        """Nanpa State実行時の入口。"""
        self.node.get_logger().info('Nanpa State started: 相手がQRを読み込むのを待っています...')

        # フラグを初期化
        self.scan_completed = False
        self.qr_data = None
        # もしServiceが利用可能なら、ここで「QRコードを読んでね」という案内を実行する
        if self.announce_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.announce_client.call_async(req)
            self.node.get_logger().info("案内Serviceを呼び出しました（発話など）")
            time.sleep(0.5)  # サービス呼び出し後の小さなウェイト（案内が始まるまでの時間）
        else:
            self.node.get_logger().warn("案内Serviceがオンラインではないため、スキップします")

        # 相手が読み込んでアクションを起こすまで待機
        timeout_duration = 60.0  # 人間がスマホを出して、カメラを起動して、ページを開く時間なので長めに（1分）
        start_time = time.time()

        while not self.scan_completed:
            # タイムアウト判定（誰も読んでくれなかった、あるいはフラれた場合）
            if (time.time() - start_time) > timeout_duration:
                self.node.get_logger().warn("タイムアウト: 制限時間内にQRコードが読まれませんでした。")
                # 「失敗（FAILED）」というメッセージを周りにパブリッシュ
                fail_msg = String()
                fail_msg.data = "FAILED"
                self.result_pub.publish(fail_msg)
                return NANPA_FAILED

            # CPU負荷を下げるためのウェイト
            time.sleep(0.1)

        # ------------------------------------------------------------------
        # 【成功】無事に相手が読み込み、リアクションしてくれた時の処理
        # ------------------------------------------------------------------
        self.node.get_logger().info("ナンパ成功！次の状態（会話など）へ遷移します。")
        
        blackboard["target_user_info"] = self.qr_data

         # 「成功（SUCCESS）」というメッセージを周りにパブリッシュ
        success_msg = String()
        success_msg.data = "SUCCESS"
        self.result_pub.publish(success_msg)
        return NANPA_SUCCESS
        # ✓TODO(福田): QRコード読み取り状態を受け取るSubscriberを定義する。
        # ✓TODO(福田): QRコード表示や案内に必要なPublisher/Serviceを定義する。
        # TODO(福田): 成功・失敗・終了を通知するPublisherを定義する。
        # TODO(福田): QRコード読み取り結果を処理する。
        # TODO(福田): ナンパ開始時の処理を追加する。
        # TODO(福田): ナンパ終了条件を判定する。
        # TODO(福田): 結果をblackboardへ保存する。
        # Optional_TODO(福田): ナンパ時の会話やジェスチャーなどの演出を追加する。
       
