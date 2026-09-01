import wpilib
from commands2 import CommandScheduler

from src.network_server.network_server import NetworkServer
from src.robot_container import RobotContainer


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.robot_container = RobotContainer()

        self.autonomous_command = self.robot_container.get_autonomous_command()

    def autonomousInit(self) -> None:
        self.autonomous_command = self.robot_container.get_autonomous_command()

        CommandScheduler.getInstance().schedule(self.autonomous_command)

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.autonomous_command.cancel()
        self.robot_container.zero_pose()

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
        NetworkServer.getInstance().set_float("v", wpilib.RobotController.getBatteryVoltage())

        self.robot_container.periodic()

    def simulationPeriodic(self) -> None:
        pass
