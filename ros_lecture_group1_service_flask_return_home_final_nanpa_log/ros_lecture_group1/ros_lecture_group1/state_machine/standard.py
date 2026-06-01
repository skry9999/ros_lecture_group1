#!/usr/bin/env python3
"""Standard states for ros_lecture_group1."""

import time

import rclpy
import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import NEXT


class FinishState(State):
    """Final state."""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Finish the state machine."""
        del blackboard
        yasmin.YASMIN_LOG_INFO('Finished ros_lecture_group1 state machine')
        return NEXT


class WaitBeforePatrolState(State):
    """初期位置へ戻ったあと，Patrol開始前に短時間待機するState。"""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT])
        self.node = node
        param_name = 'state_machine.before_patrol_wait_sec'
        if not self.node.has_parameter(param_name):
            self.node.declare_parameter(param_name, 3.0)
        self.wait_sec = float(self.node.get_parameter(param_name).value)

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

    def execute(self, blackboard: Blackboard) -> str:
        """Patrol開始前に指定秒数だけ待機する。"""
        wait_sec = max(0.0, float(self.wait_sec))
        self.node.get_logger().info(
            f'Waiting {wait_sec:.1f} seconds before patrol.',
        )
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < wait_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self._blackboard_set(blackboard, 'mission_phase', 'patrol')
        self._blackboard_set(blackboard, 'navigation_route', None)
        self._blackboard_set(blackboard, 'patrol_route', None)
        self.node.get_logger().info('Starting patrol after return-home wait.')
        return NEXT


class ExceptionState(State):
    """Fallback state."""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Handle an exception path."""
        del blackboard
        self.node.get_logger().info('An exception occurred')
        return NEXT
