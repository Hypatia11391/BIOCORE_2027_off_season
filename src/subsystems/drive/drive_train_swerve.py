import src.subsystems.drive.swerve_constants as consts
from src.subsystems.drive.swerve_module import SwerveModule
from src.navx.navx import Navx

from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Pose3d
from wpimath.estimator import SwerveDrive4PoseEstimator3d
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.util import DriveFeedforwards
from wpilib import DriverStation

from typing import override


class DriveTrainSwerve(Subsystem):
    def __init__(self, pose_estimator: SwerveDrive4PoseEstimator3d, navx: Navx) -> None:
        self.pose_estimator = pose_estimator
        self.navx = navx
        
        self.fl_module = SwerveModule(
            consts.FL_DRIVE_ID,
            consts.FL_TURN_ID,
            consts.FL_CANCODER_ID,
            0,  # TODO: shouldn't be zero figure out how to get
        )
        
        self.fr_module = SwerveModule(
            consts.FR_DRIVE_ID,
            consts.FR_TURN_ID,
            consts.FR_CANCODER_ID,
            0,
        )
        
        self.rl_module = SwerveModule(
            consts.RL_DRIVE_ID,
            consts.RL_TURN_ID,
            consts.RL_CANCODER_ID,
            0,
        )
        
        self.rr_module = SwerveModule(
            consts.RR_DRIVE_ID,
            consts.RR_TURN_ID,
            consts.RR_CANCODER_ID,
            0,
        )
        
        self.kinematics = SwerveDrive4Kinematics(
            consts.FL_LOCATION,
            consts.FR_LOCATION,
            consts.RL_LOCATION,
            consts.RR_LOCATION,
        )
        
        config = RobotConfig.fromGUISettings()
        
        def path_planner_drive(chassis_speeds: ChassisSpeeds, drive_feedforwards: DriveFeedforwards):
            self.drive(chassis_speeds)  # NOTE: In the future if use pathplanner to run choreo paths, might be good to also use feedforwards because choreo outputs target force vectors so chassis speeds is just a conversion kind of
        
        AutoBuilder.configure(
            lambda: self.pose_estimator.getEstimatedPosition().toPose2d(),
            lambda pose: self.pose_estimator.resetPose(Pose3d(pose)),
            lambda: self.kinematics.toChassisSpeeds(self.get_module_states()),
            path_planner_drive,
            consts.PATH_PLANNER_DRIVE_CONTROLLER,
            config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue,
            self,
        )
    
    def drive(self, chassis_speeds: ChassisSpeeds, field_relative=False) -> None:
        if field_relative:
            chassis_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassis_speeds,
                self.pose_estimator.getEstimatedPosition().rotation().toRotation2d(),
            )
        
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        self.kinematics.desaturateWheelSpeeds(module_states, consts.MAX_LINEAR_SPEEDS)
        
        self.fl_module.setDesiredState(module_states[0])
        self.fr_module.setDesiredState(module_states[1])
        self.rl_module.setDesiredState(module_states[2])
        self.rr_module.setDesiredState(module_states[3])
    
    def get_module_states(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return (
            self.fl_module.get_state(),
            self.fr_module.get_state(),
            self.rl_module.get_state(),
            self.rr_module.get_state(),
        )
    
    def get_module_positions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return (
            self.fl_module.get_position(),
            self.fr_module.get_position(),
            self.rl_module.get_position(),
            self.rr_module.get_position(),
        )
    
    @override
    def periodic(self) -> None:
        self.pose_estimator.update(self.navx.get_full_rotation(), self.get_module_positions())
