#!/usr/bin/env python3
"""Patrol states for ros_lecture_group1."""

import math
import yasmin
from yasmin import Blackboard
from yasmin import State
from rclpy.node import Node
from rclpy.action import ActionClient
# TODO: 実際の席情報メッセージ型に合わせてインポートを変更
# 実際の席情報メッセージ型に合わせてインポートを変更してください（ここでは仮でPoseArrayを使用）
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT_TARGET
from ros_lecture_group1.state_machine.outcomes import NO_TARGETS


class PatrolState(State):
    """Patrol Stateのひな型。"""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT_TARGET, NO_TARGETS, EXCEPT])
        self.node = node
        
        #対象者リスト
        self.target_poses = []
        #現在の情報を保持する変数
        self.current_pose = None
        
        # TODO(森): タイプ判定された席情報を受け取るSubscriberを定義する。
        self.subscription = self.node.create_subscription(
            msg_type = PoseArray,
            topic="/judged_seat_info",
            callback=self.listener_callback,
            qos_profile=10
        )
        # 現在位置を取得するためのSubscriberを追加 (amcl_poseなどを想定)
        self.pose_subscription = self.node.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic="/amcl_pose",
            callback=self.current_pose_callback,
            qos_profile=10
        )
        
        # TODO(森): 次の目的地をNavigationへ送るAction clientを定義する。
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def listener_callback(self, msg):
        """TODO(森): 対象者の席情報を保存する。"""
        # PoseArrayを想定。独自メッセージの場合は msg.xxx など適宜変更してください
        self.target_poses = msg.poses

    def current_pose_callback(self, msg):
        """現在位置を更新する"""
        self.current_pose = msg.pose.pose

    def get_closest_target(self):
        """TODO(森): 現在位置から最も近い対象を選ぶ。"""
        if not self.target_poses:
            return None
        
        # 現在地がまだ取得できていない場合は、リストの先頭を返す
        if not self.current_pose:
            self.node.get_logger().warning("現在地が不明なため、リストの先頭をターゲットにします。")
            return self.target_poses.pop(0)

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        closest_idx = 0
        min_dist = float('inf')

        # 全ターゲットとの距離を計算し、最短のものを選ぶ
        for i, pose in enumerate(self.target_poses):
            dist = math.hypot(pose.position.x - cx, pose.position.y - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 選んだターゲットをリストから削除して返す（二度同じ場所に行かないため）
        return self.target_poses.pop(closest_idx)

    def execute(self, blackboard: Blackboard) -> str:
        """Patrol State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Patrol State started')

        try:
            # TODO(森): 対象が残っていなければNO_TARGETSを返す。
            if not self.target_poses:
                yasmin.YASMIN_LOG_INFO('Target is empty. Returning NO_TARGETS.')
                return NO_TARGETS

            # 現在位置から最も近い対象を選ぶ
            next_target_pose = self.get_closest_target()

            # Action Serverが起動しているか確認
            if not self.nav_client.wait_for_server(timeout_sec=3.0):
                self.node.get_logger().error('Nav2 Action Server is not available.')
                return EXCEPT

            # TODO(森): 次の巡回先をNavigationへ渡す。
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose = next_target_pose

            # ゴールを非同期で送信する
            self.node.get_logger().info('Sending goal to Navigation...')
            self.nav_client.send_goal_async(goal_msg)

            # TODO(森): ナンパ完了後に次の対象を選ぶ処理の入口を定義する。
            # （YASMINの設計上、ここではNEXT_TARGETを返し、移動完了やナンパアクション自体は
            # 別のStateで行い、終わったら再びこのPatrolStateに戻ってくるループ構造を想定しています）

            # TODO(森): 対象が残っていればNEXT_TARGETを返す
            return NEXT_TARGET

        except Exception as e:
            self.node.get_logger().error(f"Exception in PatrolState: {e}")
            return EXCEPT
