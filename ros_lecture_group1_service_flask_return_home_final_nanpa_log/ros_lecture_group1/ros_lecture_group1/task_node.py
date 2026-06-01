#!/usr/bin/env python3
"""State machine node for ros_lecture_group1."""

import rclpy
import yasmin
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers

from ros_lecture_group1.state_machine.face_recognition import (
    FaceRecognitionState,
)
from ros_lecture_group1.state_machine.nanpa import NanpaState
from ros_lecture_group1.state_machine.navigation import NavigationState
from ros_lecture_group1.state_machine.outcomes import EXCEPT
from ros_lecture_group1.state_machine.outcomes import EXIT
from ros_lecture_group1.state_machine.outcomes import FACE_RECOGNITION
from ros_lecture_group1.state_machine.outcomes import FINISH
from ros_lecture_group1.state_machine.outcomes import NANPA
from ros_lecture_group1.state_machine.outcomes import NANPA_FAILED
from ros_lecture_group1.state_machine.outcomes import NANPA_SUCCESS
from ros_lecture_group1.state_machine.outcomes import NEXT
from ros_lecture_group1.state_machine.outcomes import NEXT_TARGET
from ros_lecture_group1.state_machine.outcomes import NO_TARGETS
from ros_lecture_group1.state_machine.patrol import PatrolState
from ros_lecture_group1.state_machine.standard import ExceptionState
from ros_lecture_group1.state_machine.standard import FinishState
from ros_lecture_group1.state_machine.standard import WaitBeforePatrolState


def main(args=None) -> None:
    """Run the ros_lecture_group1 state machine."""
    yasmin.YASMIN_LOG_INFO('Starting ros_lecture_group1 state machine')

    rclpy.init(args=args)
    node = rclpy.create_node('task_node')
    set_ros_loggers(node)

    sm = StateMachine(outcomes=[EXIT])
    sm.add_state(
        'Navigation',
        NavigationState(node),
        transitions={
            FACE_RECOGNITION: 'FaceRecognition',
            NANPA: 'Nanpa',
            NEXT: 'WaitBeforePatrol',
            FINISH: 'Finish',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'FaceRecognition',
        FaceRecognitionState(node),
        transitions={
            NEXT: 'Navigation',
            "patrol": 'Patrol',
            EXCEPT: 'Exception',
        },
    )
    sm.add_state(
        'WaitBeforePatrol',
        WaitBeforePatrolState(node),
        transitions={NEXT: 'Patrol'},
    )
    sm.add_state(
        'Patrol',
        PatrolState(node),
        transitions={
            NEXT_TARGET: 'Navigation',
            NO_TARGETS: 'Navigation',
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

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        del sm
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
