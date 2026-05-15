#!/usr/bin/env python3
"""Face recognition states for ros_lecture_group1."""

import yasmin
from yasmin import Blackboard
from yasmin import State

from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import TARGET_FOUND
from ros_lecture_group1.state_machine.outcomes import TARGET_NOT_FOUND


class FaceRecognitionState(State):
    """Face Recognition Stateのひな型。"""

    def __init__(self, node):
        super().__init__(
            outcomes=[TARGET_FOUND, TARGET_NOT_FOUND, EXCEPT],
        )
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Face Recognition State実行時の入口。"""
        yasmin.YASMIN_LOG_INFO('Face Recognition State started')

        # TODO(山根): カメラ画像を受け取るSubscriberを定義する。
        # TODO(山根): 顔検出・特徴量比較に必要なPublisher/Subscriberなどを定義する。
        # TODO(山根): タイプ判定結果を送るPublisherを定義する。
        # TODO(山根): カメラ画像を受け取ったときの処理を追加する。
        # TODO(山根): タイプの顔かどうかを判定する。
        # TODO(山根): ユーザー判断を受け取る処理を追加する。
        # TODO(山根): タイプの人の席情報をblackboardへ保存する。
        return EXCEPT
