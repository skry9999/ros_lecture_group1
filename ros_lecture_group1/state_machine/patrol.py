#!/usr/bin/env python3
"""Patrol states for ros_lecture_group1."""

import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NEXT_TARGET
from ros_lecture_group1.state_machine.outcomes import NO_TARGETS


class PatrolState(State):
    """Patrol Stateのひな型。"""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT_TARGET, NO_TARGETS, EXCEPT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Patrol State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Patrol State started')

        # TODO(森): タイプ判定された席情報を受け取るSubscriberを定義する。
        # TODO(森): 次の目的地をNavigationへ送るPublisher/Action clientを定義する。
        # TODO(森): ナンパ完了後に次の対象を選ぶ処理の入口を定義する。
        # TODO(森): 対象者の席情報を保存する。
        # TODO(森): 現在位置から最も近い対象を選ぶ。
        # TODO(森): 次の巡回先をNavigationへ渡す。
        # TODO(森): 対象が残っていればNEXT_TARGET、残っていなければNO_TARGETSを返す。
        return EXCEPT
