import wpilib
from commands2 import CommandScheduler

from src.robot_container import RobotContainer


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.robot_container = RobotContainer()

        self.autonomous_command = self.robot_container.get_autonomous_command()

    def autonomousInit(self) -> None:
        CommandScheduler.getInstance().schedule(self.autonomous_command)

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.autonomous_command.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def simulationPeriodic(self) -> None:
        pass
