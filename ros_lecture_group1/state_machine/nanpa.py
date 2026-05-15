#!/usr/bin/env python3
"""Nanpa states for ros_lecture_group1."""

import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS


class NanpaState(State):
    """Nanpa Stateのひな型。"""

    def __init__(self, node):
        super().__init__(outcomes=[NANPA_SUCCESS, NANPA_FAILED, EXCEPT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Nanpa State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Nanpa State started')

        # TODO(福田): QRコード読み取り状態を受け取るSubscriberを定義する。
        # TODO(福田): QRコード表示や案内に必要なPublisher/Serviceを定義する。
        # TODO(福田): 成功・失敗・終了を通知するPublisherを定義する。
        # TODO(福田): QRコード読み取り結果を処理する。
        # TODO(福田): ナンパ開始時の処理を追加する。
        # TODO(福田): ナンパ終了条件を判定する。
        # TODO(福田): 結果をblackboardへ保存する。
        # Optional_TODO(福田): ナンパ時の会話やジェスチャーなどの演出を追加する。
        return EXCEPT
