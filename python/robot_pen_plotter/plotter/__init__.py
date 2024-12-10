from scipy.optimize import minimize
import numpy as np
import time
import serial
from timeit import default_timer
from enum import Enum, auto
from queue import Queue
from threading import Thread, Lock
import matplotlib.pyplot as plt


ARM_LENGTH = 196.15
FOREARM_LENGTH = 186.321 + 6.1


def _forward_kinematics(configuration: np.ndarray, l1: float, l2: float) -> np.ndarray:
    assert configuration.shape == (2,)
    theta_1, theta_2 = configuration

    return np.array(
        [
            l1 * np.cos(theta_1) * np.cos(theta_2) - l1 * np.sin(theta_1) * np.sin(theta_2) + l2 * np.cos(theta_1),
            l1 * np.sin(theta_1) * np.cos(theta_2) + l1 * np.cos(theta_1) * np.sin(theta_2) + l2 * np.sin(theta_1),
        ]
    )


def _objective(joint_angles, target, current_angles, l1, l2):
    end_effector = _forward_kinematics(joint_angles, l1, l2)
    position_error = np.linalg.norm(end_effector - target)  # Distance to target
    configuration_error = np.linalg.norm(joint_angles - current_angles)  # Stay close to current config
    return position_error + configuration_error


def _closest_ik(target, current_angles, l1, l2):
    result = minimize(
        _objective,
        current_angles,  # Start from current configuration
        args=(target, current_angles, l1, l2),
        method="SLSQP",
        bounds=[(-np.pi / 2, np.pi / 2), (-3 * np.pi / 4, 3 * np.pi / 4)],  # Joint limits
        options={"disp": False},
    )
    # print(result)
    return result.x if result.success else None


def _inverse_kinematics(target: np.ndarray, current_configuration: np.ndarray, l1: float, l2: float) -> np.ndarray:
    return _closest_ik(target, current_configuration, l1, l2)


class PlotterCommandType(Enum):
    MOVE_TO = auto()
    PEN_UP = auto()
    PEN_DOWN = auto()
    HOME = auto()


class PlotterCommand:
    def get_command(self) -> str:
        raise NotImplementedError()


class MoveToCommand(PlotterCommand):
    target: np.ndarray

    def __init__(self, target: np.ndarray) -> None:
        self.target = target

    def get_command(self) -> bytes:
        a1 = self.target[0]
        a2 = self.target[1]
        return f"M {a1:.4f} {a2:.4f}"


class PenUpCommand(PlotterCommand):
    def get_command(self) -> str:
        return "PU"


class PenDownCommand(PlotterCommand):
    def get_command(self) -> bytes:
        return "PD"


class HomeCommand(PlotterCommand):
    def get_command(self) -> str:
        return "H"


class Plotter:
    l1: float
    l2: float

    current_configuration: np.ndarray

    command_queue: Queue[PlotterCommand]
    command_queue_lock: Lock
    command_sender_thread: Thread
    done: bool

    def __init__(
        self,
        arm_length: float = ARM_LENGTH,
        forearm_length: float = FOREARM_LENGTH,
        serial_port: str = "/dev/ttyACM0",
        x_input_range: tuple[float, float] = (0, 1),
        y_input_range: tuple[float, float] = (0, 1),
    ) -> None:
        self.l1 = arm_length
        self.l2 = forearm_length
        self.serial_port = serial_port
        self.current_configuration = np.array([0, 0])

        self.command_queue = Queue()
        self.command_queue_lock = Lock()
        self.command_sender_thread = Thread(target=self._command_sender_job)
        self.command_sender_thread.start()
        self.x_input_range = x_input_range
        self.y_input_range = y_input_range
        self.done = False

    def _command_sender_job(self) -> None:
        ser = serial.Serial(
            port=self.serial_port,
            baudrate=115200,
            timeout=1,
        )

        while not self.done:
            if self.command_queue.empty():
                time.sleep(0.01)
                continue

            with self.command_queue_lock:
                command = self.command_queue.get()

            print("Sending command: ", command.get_command())

            # wait until device sends "wfc"
            while True:
                if ser.in_waiting == 0:
                    continue

                line = ser.readline()
                print(line)

                try:
                    decoded = line.decode("utf-8").strip()

                    if decoded == "wfc":
                        # clear the buffer
                        while ser.in_waiting > 0:
                            line = ser.readline()
                            print(line)

                        break

                except UnicodeDecodeError:
                    print("Received invalid UTF-8 sequence")

            command_bytes = (command.get_command() + "\n").encode("utf-8")
            print("Sending command bytes: ", command_bytes)
            ser.write(command_bytes)

    def normalize_xy(self, xy: np.ndarray) -> np.ndarray:
        x_output_range = (-200, 200)  # Width: 400
        y_output_range = (150, 350)  # Height: 200
        output_aspect_ratio = (x_output_range[1] - x_output_range[0]) / (y_output_range[1] - y_output_range[0])

        input_width = self.x_input_range[1] - self.x_input_range[0]
        input_height = self.y_input_range[1] - self.y_input_range[0]
        input_aspect_ratio = input_width / input_height

        # Scale the input to fit within output bounds while maintaining aspect ratio
        if input_aspect_ratio > output_aspect_ratio:
            # Input is wider - scale to fit width
            scale = (x_output_range[1] - x_output_range[0]) / input_width
            x_scaled = x_output_range[0] + (xy[0] - self.x_input_range[0]) * scale
            y_center = (y_output_range[0] + y_output_range[1]) / 2
            y_scaled = y_center + (xy[1] - (self.y_input_range[0] + input_height / 2)) * scale
        else:
            # Input is taller - scale to fit height
            scale = (y_output_range[1] - y_output_range[0]) / input_height
            y_scaled = y_output_range[0] + (xy[1] - self.y_input_range[0]) * scale
            x_center = (x_output_range[0] + x_output_range[1]) / 2
            x_scaled = x_center + (xy[0] - (self.x_input_range[0] + input_width / 2)) * scale

        return np.array([x_scaled, y_scaled])

    def penup(self) -> None:
        with self.command_queue_lock:
            self.command_queue.put(PenUpCommand())

    def pendown(self) -> None:
        with self.command_queue_lock:
            self.command_queue.put(PenDownCommand())

    def move_to(self, target_xy: np.ndarray) -> None:
        print("Target: ", target_xy)
        target_xy = self.normalize_xy(target_xy)
        print("Normalized: ", target_xy)
        target = _inverse_kinematics(target_xy, self.current_configuration, self.l1, self.l2)
        print("Inverse kinematics: ", target)

        self.current_configuration = target
        with self.command_queue_lock:
            self.command_queue.put(MoveToCommand(target))

    def home(self) -> None:
        with self.command_queue_lock:
            self.command_queue.put(HomeCommand())

    def wait_until_done(self) -> None:
        while not self.command_queue.empty():
            time.sleep(0.01)

        self.done = True
        self.command_sender_thread.join()
