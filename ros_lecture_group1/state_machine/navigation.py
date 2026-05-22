#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Navigation states for ros_lecture_group1."""

import json
import math
from typing import Any

import rclpy
import tf_transformations
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import (
    NavigateToPose_Feedback,
    NavigateToPose_FeedbackMessage,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT


class NavigationState(State):
    """Nav2を使って席や巡回経路へ移動するState。"""

    def __init__(self, node: Node):
        """NavigationStateを初期化する。"""
        super().__init__(outcomes=[NEXT, EXCEPT])
        self.node = node

        self._goal_handle: ClientGoalHandle | None = None
        self._result_future: Any | None = None
        self._feedback: NavigateToPose_Feedback | None = None
        self._status: int | None = None
        self._requested_stop = False
        self._people_detected = False
        self._command = 'start'
        self._requested_goal: dict[str, Any] | str | None = None

        self._declare_parameters()
        self.action_name = self._get_parameter('navigation.action_name')
        self.frame_id = self._get_parameter('navigation.frame_id')
        self.timeout_sec = float(
            self._get_parameter('navigation.timeout_sec'),
        )

        self.seats = self._load_json_parameter('navigation.seats_json', {})
        self.default_route = self._load_json_parameter(
            'navigation.default_route_json',
            [],
        )
        self.default_goal = self._load_json_parameter(
            'navigation.default_goal_json',
            {'id': 'home', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        )

        self.nav_to_pose_client = ActionClient(
            node=self.node,
            action_type=NavigateToPose,
            action_name=self.action_name,
        )
        self.status_pub = self.node.create_publisher(
            String,
            '/navigation/status',
            10,
        )
        self.current_seat_pub = self.node.create_publisher(
            String,
            '/navigation/current_seat',
            10,
        )
        self.command_sub = self.node.create_subscription(
            String,
            '/navigation/command',
            self._command_callback,
            10,
        )
        self.goal_sub = self.node.create_subscription(
            String,
            '/navigation/goal',
            self._goal_callback,
            10,
        )
        self.people_sub = self.node.create_subscription(
            Bool,
            '/navigation/people_detected',
            self._people_callback,
            10,
        )

    def _declare_parameters(self) -> None:
        """Navigationで使うパラメータを宣言する。"""
        defaults = {
            'navigation.action_name': '/navigate_to_pose',
            'navigation.frame_id': 'map',
            'navigation.timeout_sec': 30.0,
            'navigation.seats_json': '{}',
            'navigation.default_route_json': '[]',
            'navigation.default_goal_json': (
                '{"id": "home", "x": 0.0, "y": 0.0, "yaw": 0.0}'
            ),
        }
        for name, value in defaults.items():
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, value)

    def _get_parameter(self, name: str) -> Any:
        """ROSパラメータの値を取得する。"""
        return self.node.get_parameter(name).value

    def _load_json_parameter(self, name: str, default: Any) -> Any:
        """JSON文字列で指定されたパラメータを読み込む。"""
        value = self._get_parameter(name)
        if value in (None, ''):
            return default
        try:
            return json.loads(value)
        except json.JSONDecodeError as err:
            self.node.get_logger().error(
                f'Failed to parse parameter {name}: {err}',
            )
            return default

    @staticmethod
    def _blackboard_get(
        blackboard: Blackboard,
        key: str,
        default: Any = None,
    ) -> Any:
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
        value: Any,
    ) -> None:
        """Blackboardへ値を保存する。"""
        try:
            blackboard[key] = value
        except TypeError:
            setattr(blackboard, key, value)

    def _publish_status(self, status: str) -> None:
        """Navigation状態をTopicへ通知する。"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_current_seat(self, seat_id: str) -> None:
        """現在到着した席IDをTopicへ通知する。"""
        msg = String()
        msg.data = seat_id
        self.current_seat_pub.publish(msg)

    def _command_callback(self, msg: String) -> None:
        """巡回開始・停止などの指示を受け取る。"""
        command = msg.data.strip().lower()
        self._command = command
        if command in ('stop', 'cancel'):
            self._requested_stop = True
            self.cancel_nav()
        elif command in ('start', 'resume'):
            self._requested_stop = False

    def _goal_callback(self, msg: String) -> None:
        """外部から移動先の席IDまたはPose JSONを受け取る。"""
        data = msg.data.strip()
        if not data:
            return
        try:
            self._requested_goal = json.loads(data)
        except json.JSONDecodeError:
            self._requested_goal = data

    def _people_callback(self, msg: Bool) -> None:
        """人検出結果を受け取り、必要ならNav2の再計画に任せる。"""
        self._people_detected = msg.data
        if self._people_detected:
            self.node.get_logger().debug(
                'Person detected during navigation. Nav2 will replan.',
            )
            self._publish_status('person_detected')

    def _resolve_goal(
        self,
        goal: dict[str, Any] | str,
    ) -> dict[str, Any] | None:
        """席IDまたはPose辞書をNav2へ送れるゴール辞書に変換する。"""
        if isinstance(goal, str):
            seat = self.seats.get(goal)
            if not seat:
                self.node.get_logger().error(f'Unknown seat id: {goal}')
                return None
            resolved = dict(seat)
            resolved.setdefault('id', goal)
            return resolved

        if {'x', 'y'}.issubset(goal):
            resolved = dict(goal)
            resolved.setdefault('id', '')
            resolved.setdefault('yaw', 0.0)
            return resolved

        seat_id = goal.get('id') or goal.get('seat_id')
        if seat_id and seat_id in self.seats:
            resolved = dict(self.seats[seat_id])
            resolved.setdefault('id', seat_id)
            return resolved

        self.node.get_logger().error(f'Invalid navigation goal: {goal}')
        return None

    def _make_route(self, blackboard: Blackboard) -> list[dict[str, Any]]:
        """Blackboard、Topic、パラメータから巡回経路を作成する。"""
        seats = self._blackboard_get(blackboard, 'seats', None)
        if isinstance(seats, dict):
            self.seats.update(seats)

        route_source = (
            self._blackboard_get(blackboard, 'navigation_route', None)
            or self._blackboard_get(blackboard, 'patrol_route', None)
        )
        target_seat = (
            self._blackboard_get(blackboard, 'target_seat_id', None)
            or self._blackboard_get(blackboard, 'next_seat_id', None)
        )

        if self._requested_goal:
            route_source = [self._requested_goal]
        elif target_seat:
            route_source = [target_seat]
        elif not route_source:
            route_source = self.default_route or [self.default_goal]

        if isinstance(route_source, (str, dict)):
            route_source = [route_source]

        route = []
        for goal in route_source:
            resolved = self._resolve_goal(goal)
            if resolved:
                route.append(resolved)
        return route

    def go_to_pose(self, goal: dict[str, Any]) -> bool:
        """指定した目的地までナビゲーションを開始する。"""
        x = float(goal['x'])
        y = float(goal['y'])
        yaw = float(goal.get('yaw', 0.0))

        self.node.get_logger().debug(
            "Waiting for 'NavigateToPose' action server",
        )
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(
                "'NavigateToPose' action server not available, waiting...",
            )

        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.node.get_logger().info(
            f'Navigating to goal: (x, y, yaw) = ({x}, {y}, {yaw})',
        )
        self._publish_status('navigating')

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal=goal_msg,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        self._goal_handle = send_goal_future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().error(
                f'Goal to (x, y, yaw) = ({x}, {y}, {yaw}) was rejected!',
            )
            self._publish_status('rejected')
            return False

        self._result_future = self._goal_handle.get_result_async()
        self._status = None
        self._feedback = None
        return True

    def cancel_nav(self) -> None:
        """実行中のナビゲーションをキャンセルする。"""
        if self._result_future and self._goal_handle:
            self.node.get_logger().info('Canceling current navigation.')
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, future)

    def is_nav_complete(self) -> bool:
        """ナビゲーションの完了状態を返す。"""
        if not self._result_future:
            return True

        rclpy.spin_until_future_complete(
            self.node,
            self._result_future,
            timeout_sec=0.10,
        )
        if not self._result_future.result():
            return False

        self._status = self._result_future.result().status
        if self._status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().debug(
                f'Task failed with status code: {self._status}',
            )
        else:
            self.node.get_logger().debug('Navigation succeeded!')
        return True

    def get_result(self) -> int | None:
        """保留中のアクションの結果ステータスを返す。"""
        return self._status

    def get_feedback(self) -> NavigateToPose_Feedback | None:
        """保留中のアクションのフィードバックを返す。"""
        return self._feedback

    def _feedback_callback(
        self,
        msg: NavigateToPose_FeedbackMessage,
    ) -> None:
        """NavigateToPose actionのフィードバックを受け取る。"""
        self._feedback = msg.feedback

    @staticmethod
    def _duration_to_seconds(duration) -> float:
        """ROSのDurationメッセージを秒へ変換する。"""
        return duration.sec + duration.nanosec * 1e-9

    def _wait_until_goal_finished(self) -> bool:
        """Nav2 goalの完了、停止指示、タイムアウトを監視する。"""
        while not self.is_nav_complete():
            if self._requested_stop:
                self.cancel_nav()
                self._publish_status('stopped')
                return False

            feedback = self.get_feedback()
            if not feedback:
                continue

            elapsed = self._duration_to_seconds(feedback.navigation_time)
            if math.isfinite(self.timeout_sec) and elapsed > self.timeout_sec:
                self.node.get_logger().error('Navigation timed out.')
                self.cancel_nav()
                self._publish_status('timeout')
                return False

        return self.get_result() == GoalStatus.STATUS_SUCCEEDED

    def _save_arrival(
        self,
        blackboard: Blackboard,
        goal: dict[str, Any],
    ) -> None:
        """到着した席・位置情報をBlackboardへ保存する。"""
        seat_id = str(goal.get('id', ''))
        current_pose = {
            'x': float(goal['x']),
            'y': float(goal['y']),
            'yaw': float(goal.get('yaw', 0.0)),
        }
        self._blackboard_set(blackboard, 'current_pose', current_pose)
        if seat_id:
            self._blackboard_set(blackboard, 'current_seat_id', seat_id)
            self._publish_current_seat(seat_id)

    def execute(self, blackboard: Blackboard) -> str:
        """Navigation State実行時の入口。"""
        self.node.get_logger().info('Navigation State started')

        if not rclpy.ok():
            return EXCEPT

        command = self._blackboard_get(
            blackboard,
            'navigation_command',
            self._command,
        )
        if str(command).lower() in ('stop', 'cancel'):
            self._requested_stop = True
            self._publish_status('stopped')
            return EXCEPT

        self._requested_stop = False
        route = self._make_route(blackboard)
        if not route:
            self.node.get_logger().error('No navigation route is available.')
            self._publish_status('no_route')
            return EXCEPT

        self._blackboard_set(blackboard, 'navigation_route', route)
        for goal in route:
            if not self.go_to_pose(goal):
                self._blackboard_set(blackboard, 'navigation_result', 'failed')
                return EXCEPT

            if not self._wait_until_goal_finished():
                result = self.get_result()
                self._blackboard_set(
                    blackboard,
                    'navigation_result',
                    f'failed:{result}',
                )
                return EXCEPT

            self._save_arrival(blackboard, goal)

        self._blackboard_set(blackboard, 'navigation_result', 'succeeded')
        self._publish_status('succeeded')
        return NEXT

    def goToPose(self, x: float, y: float, yaw: float) -> bool:
        """提供サンプルと同じ引数で目的地へ移動する。"""
        return self.go_to_pose({'x': x, 'y': y, 'yaw': yaw})

    def cancelNav(self) -> None:
        """提供サンプルと同じ名前でナビゲーションをキャンセルする。"""
        self.cancel_nav()

    def isNavComplete(self) -> bool:
        """提供サンプルと同じ名前で完了状態を返す。"""
        return self.is_nav_complete()

    def getResult(self) -> int | None:
        """提供サンプルと同じ名前で結果ステータスを返す。"""
        return self.get_result()

    def getFeedback(self) -> NavigateToPose_Feedback | None:
        """提供サンプルと同じ名前でフィードバックを返す。"""
        return self.get_feedback()
