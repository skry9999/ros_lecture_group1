#!/usr/bin/env python3
"""Navigation states for ros_lecture_group1."""

import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT


class NavigationState(State):
    """Navigation Stateのひな型。"""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT, EXCEPT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Navigation State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Navigation State started')

        # TODO(榊原): 必要なPublisher/Subscriber/Action clientを定義する。
        # TODO(榊原): マップ情報と席情報を読み込む処理を追加する。
        # TODO(榊原): 巡回開始・停止の指示を受け取る処理を追加する。
        # TODO(榊原): 巡回経路を作成する。
        # TODO(榊原): 指定された席へ移動する。
        # TODO(榊原): 人を回避するNavigation連携処理を追加する。
        # TODO(榊原): 必要ならblackboardに現在位置や席IDを書く。
        return EXCEPT
