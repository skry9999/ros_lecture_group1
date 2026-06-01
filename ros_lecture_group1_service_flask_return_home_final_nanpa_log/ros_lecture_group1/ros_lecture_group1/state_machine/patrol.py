#!/usr/bin/env python3
"""Patrol states for ros_lecture_group1."""

import math
import json
import yasmin
from yasmin import Blackboard
from yasmin import State
from rclpy.node import Node

# amcl_poseを受け取るための型
from geometry_msgs.msg import PoseWithCovarianceStamped

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT_TARGET
from ros_lecture_group1.state_machine.outcomes import NO_TARGETS


class PatrolState(State):
    """Patrol Stateのひな型。"""

    def __init__(self, node: Node):
        super().__init__(outcomes=[NEXT_TARGET, NO_TARGETS, EXCEPT])
        self.node = node
        
        # 現在位置を保持する変数
        self.current_pose = None
        
        # 席の座標情報 (NavigationStateと同様にパラメータから取得する想定)
        self.seats = self._load_json_parameter('navigation.seats_json', {})

        # 現在位置を取得するためのSubscriber (amcl_poseなどを想定)
        self.pose_subscription = self.node.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic="/amcl_pose",
            callback=self.current_pose_callback,
            qos_profile=10
        )

    def _load_json_parameter(self, name: str, default: dict) -> dict:
        """JSON文字列で指定されたパラメータを読み込む。"""
        if not self.node.has_parameter(name):
            return default
        value = self.node.get_parameter(name).value
        if value in (None, ''):
            return default
        try:
            return json.loads(value)
        except json.JSONDecodeError as err:
            self.node.get_logger().error(f'Failed to parse parameter {name}: {err}')
            return default

    def current_pose_callback(self, msg):
        """現在位置を更新する"""
        self.current_pose = msg.pose.pose

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

    def get_closest_target_seat_id(self, target_seats: list):
        """現在位置から最も近い席IDを選び、リストから削除して返す"""
        if not target_seats:
            return None, target_seats
        
        # 現在地が不明、または席の座標データがない場合は、リストの先頭を返す
        if not self.current_pose or not self.seats:
            self.node.get_logger().warning("現在地が不明、または席データがないため、リストの先頭をターゲットにします。")
            closest_seat_id = target_seats.pop(0)
            return closest_seat_id, target_seats

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        closest_idx = 0
        min_dist = float('inf')
        found_valid_seat = False

        # 全ターゲット席との距離を計算し、最短のものを選ぶ
        for i, seat_id in enumerate(target_seats):
            # seat_idを文字列にして辞書から座標を取得
            seat_info = self.seats.get(str(seat_id))
            
            if not seat_info or 'x' not in seat_info or 'y' not in seat_info:
                self.node.get_logger().warning(f"席ID {seat_id} の座標データが見つかりません。")
                continue

            dist = math.hypot(float(seat_info['x']) - cx, float(seat_info['y']) - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                found_valid_seat = True

        if not found_valid_seat:
            self.node.get_logger().warning(
                "有効な座標データがないため、リストの先頭をターゲットにします。"
            )
            closest_seat_id = target_seats.pop(0)
            return closest_seat_id, target_seats

        # 選んだターゲットをリストから削除（二度同じ場所に行かないため）
        closest_seat_id = target_seats.pop(closest_idx)
        return closest_seat_id, target_seats


    def execute(self, blackboard: Blackboard) -> str:
        """Patrol State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Patrol State started')

        try:
            # 1. Blackboardから対象者リスト（FaceRecognitionで保存された席IDのリスト）を取得
            target_seats = self._blackboard_get(blackboard, 'target_seats', [])
            visited_goals = self._blackboard_get(blackboard, 'visited_goals', {})
            if isinstance(visited_goals, dict):
                self.seats.update(visited_goals)

            # 対象が残っていなければ，最後に初期位置へ戻ってから終了する。
            if not target_seats:
                yasmin.YASMIN_LOG_INFO(
                    'Target is empty. Returning to initial position before finish.'
                )
                self._blackboard_set(blackboard, 'mission_phase', 'final_return_home')
                self._blackboard_set(blackboard, 'navigation_route', None)
                self._blackboard_set(blackboard, 'patrol_route', None)
                self._blackboard_set(blackboard, 'target_seat_id', None)
                return NO_TARGETS

            # 2. 現在位置から最も近い対象の席IDを選ぶ
            next_seat_id, updated_target_seats = self.get_closest_target_seat_id(target_seats)

            if next_seat_id is None:
                return NO_TARGETS

            # 3. 状態をBlackboardに書き戻す
            # 削ったあとのリストを保存（次の巡回のため）
            self._blackboard_set(
                blackboard,
                'target_seats',
                updated_target_seats,
            )
            
            # 次の目的地の席IDを保存（NavigationStateの target_seat_id に合わせる）
            self._blackboard_set(blackboard, 'target_seat_id', str(next_seat_id))
            
            # NavigationStateが 'patrol_route' を見に行く設計になっているため、そこにも設定しておく
            self._blackboard_set(
                blackboard,
                'patrol_route',
                [self.seats.get(str(next_seat_id), str(next_seat_id))],
            )
            self._blackboard_set(blackboard, 'mission_phase', 'patrol')
            self._blackboard_set(blackboard, 'navigation_route', None)

            self.node.get_logger().info(f'Next target seat selected: {next_seat_id}. Remaining targets: {len(updated_target_seats)}')

            # 4. 対象が存在したのでNEXT_TARGETを返す (この後、NavigationStateへ遷移する想定)
            return NEXT_TARGET

        except Exception as e:
            self.node.get_logger().error(f"Exception in PatrolState: {e}")
            return EXCEPT
