#!/usr/bin/env python3
"""Standard states for ros_lecture_group1."""

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


class ExceptionState(State):
    """Fallback state."""

    def __init__(self, node):
        super().__init__(outcomes=[NEXT])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Handle an exception path."""
        del blackboard
        self.node.get_logger.info('An exception occurred')
        return NEXT
