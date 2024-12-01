from scipy.optimize import minimize
import numpy as np
from enum import Enum, auto
import matplotlib.pyplot as plt


ARM_LENGTH = 196.15
FOREARM_LENGTH = 186.321 + 6.1


def forward_kinematics(configuration: np.ndarray, L1: float, L2: float) -> np.ndarray:


def _objective(joint_angles, target, current_angles, L1, L2):
    end_effector = forward_kinematics(joint_angles, L1, L2)
    position_error = np.linalg.norm(end_effector - target)  # Distance to target
    configuration_error = np.linalg.norm(joint_angles - current_angles)  # Stay close to current config
    return position_error + configuration_error


def closest_ik(target, current_angles, L1, L2):
    result = minimize(
        _objective,
        current_angles,  # Start from current configuration
        args=(target, current_angles, L1, L2),
        method="SLSQP",
        bounds=[(-np.pi / 2, np.pi / 2), (-3 * np.pi / 4, 3 * np.pi / 4)],  # Joint limits
        options={"disp": False},
    )
    # print(result)
    return result.x if result.success else None


class PlotterCommandType(Enum):
    MOVE_TO = auto()
    PEN_UP = auto()
    PEN_DOWN = auto()
    HOME = auto()

class PlotterCommand:
    def get_command(self) -> bytes:
        raise NotImplementedError()

    def encode(self) -> bytes:
        return self.get_command().encode("utf-8") + b"\n"


class MoveToCommand(PlotterCommand):
    target: np.ndarray

    def __init__(self, target: np.ndarray) -> None:
        self.target = target

    def get_command(self) -> bytes:
        a1 = self.target[0]
        a2 = self.target[1]
        return self.encode(f"M {a1} {a2}")

class PenUpCommand(PlotterCommand):
    def get_command(self) -> bytes:
        return self.encode("PU")

class PenDownCommand(PlotterCommand):
    def get_command(self) -> bytes:
        return self.encode("PD")

class HomeCommand(PlotterCommand):
    def get_command(self) -> bytes:
        return self.encode("H")


class Plotter:
    L1: float
    L2: float

    current_configuration: np.ndarray

    

    def __init__(
        self,
        arm_length: float = ARM_LENGTH,
        forearm_length: float = FOREARM_LENGTH,
    ) -> None:
        self.L1 = arm_length
        self.L2 = forearm_length
        self.current_configuration = np.array([0, 0])


    def forward_kinematics(self, configuration: np.ndarray) -> np.ndarray:
        assert configuration.shape == (2,)
        theta, phi = configuration

        return np.array(
            [
                self.L1 * np.cos(theta) * np.cos(phi) - self.L1 * np.sin(theta) * np.sin(phi) + self.L2 * np.cos(theta),
                self.L1 * np.sin(theta) * np.cos(phi) + self.L1 * np.cos(theta) * np.sin(phi) + self.L2 * np.sin(theta),
            ]
        )

    def inverse_kinematics(self, target: np.ndarray) -> np.ndarray:
        return closest_ik(target, self.current_configuration, self.L1, self.L2)
