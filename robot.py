import wpilib
from commands2 import CommandScheduler

from src.robot_container import RobotContainer


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.robot_container = RobotContainer()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def disabledPeriodic(self):
        pass

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def simulationPeriodic(self):
        pass
