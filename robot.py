import typing
typing.override = lambda x: x

import os

import wpilib
from commands2 import CommandScheduler

from src.network_server.network_server import NetworkServer
from src.robot_container import RobotContainer
from src.recording.recording import Recording
from src.commands.drive_recording import DriveRecording


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")
        
        self.robot_container = RobotContainer(self)

        self.autonomous_command = self.robot_container.get_autonomous_command()

        if os.environ.get("CONSTANT_TUNING_SCRIPT")=='true':
            if wpilib.RobotBase.isSimulation():
                wpilib.simulation.DriverStationSim.setAutonomous(True)
                wpilib.simulation.DriverStationSim.setEnabled(True)
                wpilib.simulation.DriverStationSim.notifyNewData()
    
    def autonomousInit(self) -> None:
        self.autonomous_init = None
        self.autonomous_command = self.robot_container.get_autonomous_command()

        CommandScheduler.getInstance().schedule(self.autonomous_command)
        #self.autonomous_init=None

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.autonomous_command.cancel()
        self.robot_container.zero_pose()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        print(f'{self.robot_container.drive.get_pose_rms()=}')
        recording_json_filename = os.environ.get("RECORDING_JSON", None)
        if recording_json_filename is not None:
            self.autonomous_command.cancel()
            self.test_command = DriveRecording(self.robot_container.drive, Recording.from_json_file(recording_json_filename))
            CommandScheduler.getInstance().schedule(self.test_command)

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
