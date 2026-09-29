from enum import Enum
from typing import NamedTuple

from commands2 import Subsystem
from wpimath.kinematics import ChassisSpeeds

from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum


class DummySubsystem(Subsystem):
    pass


class CommandTypes(Enum):
    CLAIM_COMMAND = 0
    DRIVE_TRANSLATION_COMMAND = 1
    DRIVE_TURN_COMMAND = 2
    DRIVE_TURN_TRANSLATION_COMMAND = 3
    DRIVE_CHASSIS_SPEEDS_COMMAND = 4


class ClaimCommand(NamedTuple):
    id: int
    claim_translation: bool = False
    claim_turn: bool = False

    command_type: int = CommandTypes.CLAIM_COMMAND.value


class DriveTranslationCommand(NamedTuple):
    """speeds should be between -1, and 1"""

    id: int
    x_speed: float
    y_speed: float

    command_type: int = CommandTypes.DRIVE_TRANSLATION_COMMAND.value


class DriveTurnCommand(NamedTuple):
    """turn speed should be between -1, and 1"""

    id: int
    turn_speed: float

    command_type: int = CommandTypes.DRIVE_TURN_COMMAND.value


class DriveTurnAndTranslationCommand(NamedTuple):
    """speeds should be between -1, and 1"""

    id: int
    x_speed: float
    y_speed: float
    turn_speed: float

    command_type: int = CommandTypes.DRIVE_TURN_TRANSLATION_COMMAND.value


class DriveChassisSpeedsCommand(NamedTuple):
    id: int
    chassis_speeds: ChassisSpeeds = ChassisSpeeds()

    command_type: int = CommandTypes.DRIVE_CHASSIS_SPEEDS_COMMAND.value


class DriveHandler:
    def __init__(self, drive: DriveTrainMecanum):
        self.drive = drive

        self.dummy_translation = DummySubsystem()
        self.dummy_turn = DummySubsystem()

        self.command_buffer = []

        self.current_translation = -1
        self.current_turn = -1

        self.current_id = 0

    def get_id(self) -> int:
        self.current_id += 1

        return self.current_id

    def append_command(self, command: ClaimCommand) -> None:
        self.command_buffer.append(command)

    def get_dummy_translation(self) -> Subsystem:
        return self.dummy_translation

    def get_dummy_turn(self) -> Subsystem:
        return self.dummy_turn

    def periodic(self) -> None:
        using_chassis_speeds = False

        x_speed = 0
        y_speed = 0
        turn_speed = 0

        chassis_speeds = ChassisSpeeds()

        for command in self.command_buffer:
            match command.command_type:
                case CommandTypes.CLAIM_COMMAND.value:
                    if command.claim_translation:
                        self.current_translation = command.id

                    if command.claim_turn:
                        self.current_turn = command.id

                case CommandTypes.DRIVE_TRANSLATION_COMMAND.value:
                    if self.current_translation == command.id:
                        x_speed = command.x_speed
                        y_speed = command.y_speed

                case CommandTypes.DRIVE_TURN_COMMAND.value:
                    if self.current_translation == command.id:
                        turn_speed = command.turn_speed

                case CommandTypes.DRIVE_TURN_TRANSLATION_COMMAND.value:
                    if self.current_translation == command.id and self.current_turn == command.id:
                        x_speed = command.x_speed
                        y_speed = command.y_speed
                        turn_speed = command.turn_speed

                case CommandTypes.DRIVE_CHASSIS_SPEEDS_COMMAND.value:
                    if self.current_translation == command.id and self.current_turn == command.id:
                        chassis_speeds = command.chassis_speeds

        if not using_chassis_speeds:
            self.drive.drive(x_speed, y_speed, turn_speed)
        else:
            self.drive.drive_from_chassis_speeds(chassis_speeds)
