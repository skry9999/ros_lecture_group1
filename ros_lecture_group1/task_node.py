#!/usr/bin/env python3
"""State machine node for ros_lecture_group1."""

import rclpy
import yasmin
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from ros_lecture_group1.state_machine.face_recognition import (
    FaceRecognitionState,
)
from ros_lecture_group1.state_machine.nanpa import NanpaState
from ros_lecture_group1.state_machine.navigation import NavigationState
from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import EXIT
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS
from ros_lecture_group1.state_machine.outcomes import NEXT
from ros_lecture_group1.state_machine.outcomes import NEXT_TARGET
from ros_lecture_group1.state_machine.outcomes import NO_TARGETS
from ros_lecture_group1.state_machine.outcomes import TARGET_FOUND
from ros_lecture_group1.state_machine.outcomes import TARGET_NOT_FOUND
from ros_lecture_group1.state_machine.patrol import PatrolState
from ros_lecture_group1.state_machine.standard import ExceptionState
from ros_lecture_group1.state_machine.standard import FinishState


def main(args=None) -> None:
    """Run the ros_lecture_group1 state machine."""
    yasmin.YASMIN_LOG_INFO('Starting ros_lecture_group1 state machine')

    rclpy.init(args=args)
    node = rclpy.create_node('ros_lecture_group1_state_machine')
    set_ros_loggers(node)

    sm = StateMachine(outcomes=[EXIT])
    sm.add_state(
        'Navigation',
        NavigationState(node),
        transitions={
            NEXT: 'FaceRecognition',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'FaceRecognition',
        FaceRecognitionState(node),
        transitions={
            TARGET_FOUND: 'Navigation',
            TARGET_NOT_FOUND: 'Navigation',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'Patrol',
        PatrolState(node),
        transitions={
            NEXT_TARGET: 'Nanpa',
            NO_TARGETS: 'Finish',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'Nanpa',
        NanpaState(node),
        transitions={
            NANPA_SUCCESS: 'Patrol',
            NANPA_FAILED: 'Patrol',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'Finish',
        FinishState(node),
        transitions={NEXT: EXIT},
    )
    sm.add_state(
        'Exception',
        ExceptionState(node),
        transitions={NEXT: EXIT},
    )

    sm.set_start_state('Nanpa')

    viewer = YasminViewerPub(
        sm,
        'ros_lecture_group1_state_machine',
        rate=4,
        node=node,
    )

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        viewer.cleanup()
        del sm
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
