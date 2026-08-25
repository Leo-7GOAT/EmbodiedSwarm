import pytest

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.hal.robot_adapter import RobotAdapter


def test_robot_adapter_cannot_be_instantiated():
    with pytest.raises(TypeError):
        RobotAdapter()


def test_incomplete_adapter_cannot_be_instantiated():
    class IncompleteAdapter(RobotAdapter):

        def reset(self) -> RobotState:
            return RobotState()

    with pytest.raises(TypeError):
        IncompleteAdapter()


def test_complete_adapter_can_be_instantiated():
    class CompleteAdapter(RobotAdapter):

        def reset(self) -> RobotState:
            return RobotState()

        def read_state(self) -> RobotState:
            return RobotState()

        def send_command(
            self,
            command: RobotCommand,
        ) -> None:
            pass

        def step(self, dt: float) -> RobotState:
            return RobotState()

        def shutdown(self) -> None:
            pass

    adapter = CompleteAdapter()

    assert isinstance(adapter, RobotAdapter)


def test_complete_adapter_interface():
    class CompleteAdapter(RobotAdapter):

        def __init__(self):
            self.state = RobotState()

        def reset(self) -> RobotState:
            self.state = RobotState()
            return self.state

        def read_state(self) -> RobotState:
            return self.state

        def send_command(
            self,
            command: RobotCommand,
        ) -> None:
            pass

        def step(self, dt: float) -> RobotState:
            return self.state

        def shutdown(self) -> None:
            pass

    adapter = CompleteAdapter()

    assert isinstance(adapter.reset(), RobotState)
    assert isinstance(adapter.read_state(), RobotState)
    assert isinstance(adapter.step(0.01), RobotState)