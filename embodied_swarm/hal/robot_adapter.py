from abc import ABC, abstractmethod

from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.core.robot_command import RobotCommand


class RobotAdapter(ABC):

    @abstractmethod
    def reset(self) -> RobotState:
        raise NotImplementedError

    @abstractmethod
    def read_state(self) -> RobotState:
        raise NotImplementedError

    @abstractmethod
    def send_command(
        self,
        command: RobotCommand,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self, dt: float) -> RobotState:
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        raise NotImplementedError