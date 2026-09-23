from typing import override
from math import pi

import rev
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
import wpilib
from wpilib import DriverStation, Field2d, SmartDashboard
from wpilib.drive import MecanumDrive
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveKinematics, MecanumDriveWheelPositions, MecanumDriveWheelSpeeds
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import SimDeviceSim, LinearSystemSim_1_1_1

from src.navx.navx import Navx
from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_LEFT_LOCATION, FRONT_RIGHT_ID, FRONT_RIGHT_LOCATION, MAX_ANGULAR_SPEED, MAX_SPEED, REAR_LEFT_ID, REAR_LEFT_LOCATION, REAR_RIGHT_ID, REAR_RIGHT_LOCATION, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO


class DriveTrainMecanum(Subsystem):
    def __init__(self, pose_estimator: MecanumDrivePoseEstimator3d, navx: Navx) -> None:
        super().__init__()

        self.left_front_drive = rev.SparkMax(FRONT_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(FRONT_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(REAR_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(REAR_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        
        self.config_drive_motor(self.left_front_drive, False)
        self.config_drive_motor(self.right_front_drive, True)
        self.config_drive_motor(self.left_rear_drive, False)
        self.config_drive_motor(self.right_rear_drive, True)

        self.left_front_encoder = self.left_front_drive.getEncoder()
        self.right_front_encoder = self.right_front_drive.getEncoder()
        self.left_rear_encoder = self.left_rear_drive.getEncoder()
        self.right_rear_encoder = self.right_rear_drive.getEncoder()

        self.robot_drive = MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

        self.pose_estimator = pose_estimator
        self.navx = navx

        self.kinematics = MecanumDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            REAR_LEFT_LOCATION,
            REAR_RIGHT_LOCATION,
        )

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.get_pose_2d,
            self.reset_pose_2d,
            self.get_relative_speeds,
            lambda speeds, feedforwards: self.drive_from_chassis_speeds(speeds),
            PPHolonomicDriveController(PIDConstants(0.25, 0.0, 0.03), PIDConstants(0.25, 0.0, 0.01)),
            config,
            self.should_flip_path,
            self,
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        if wpilib.RobotBase.isSimulation():
            self._init_simulation()

    def _init_simulation(self):
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()

        wheel_plant = LinearSystemId.identifyVelocitySystemRadians(
            kV = self.battery_voltage / (MAX_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_GEAR_RATIO * (2*pi)),  # ratio of volts to speed in rad/s
            kA = 0.007,
        )
                
        self.fl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.fr_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.rl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.rr_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        
        self.fl_motor_sim = rev.SparkMaxSim(self.left_front_drive, DCMotor.NEO(1))
        self.fr_motor_sim = rev.SparkMaxSim(self.right_front_drive, DCMotor.NEO(1))
        self.rl_motor_sim = rev.SparkMaxSim(self.left_rear_drive, DCMotor.NEO(1))
        self.rr_motor_sim = rev.SparkMaxSim(self.right_rear_drive, DCMotor.NEO(1))
        
        self.navx_sim = SimDeviceSim("navX-Sensor[4]")
        self.navx_sim_yaw = self.navx_sim.getDouble("Yaw")

        self.last_sim_time = wpilib.Timer.getFPGATimestamp()
    
    def should_flip_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        print('speeds:', forward_speed, strafe_speed, turn_speed)

        # clamp = 0.25

        # forward_speed = max(min(forward_speed, clamp), -clamp)
        # strafe_speed = max(min(strafe_speed, clamp), -clamp)
        # turn_speed = max(min(turn_speed, clamp), -clamp)

        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed)

    def drive_field_oriented(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed, self.navx.get_2d_rotation())

    def drive_from_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        forward_speed = speeds.vx
        strafe_speed = speeds.vy
        turn_speed = speeds.omega

        forward_speed_percent = forward_speed / MAX_SPEED
        strafe_speed_percent = strafe_speed / MAX_SPEED
        print(turn_speed)
        turn_speed_percent = turn_speed / MAX_ANGULAR_SPEED
        
        self.drive(forward_speed_percent, strafe_speed_percent, turn_speed_percent)
    
    @override
    def periodic(self) -> None:
        print(f'{self.get_wheel_speeds()=}')
        print(f'{self.get_relative_speeds()=}')
        print(self.pose_estimator.getEstimatedPosition().toPose2d())
        
        if self.pose_estimator is not None:
            self.pose_estimator.update(
                self.navx.get_full_rotation(),
                self.get_wheel_positions(),
            )
            self.field.setRobotPose(self.pose_estimator.getEstimatedPosition().toPose2d())
    
    @override
    def simulationPeriodic(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        tm_diff = current_time - self.last_sim_time
        self.last_sim_time = current_time

        # Update wheel linear system
        self.fl_system_sim.setInput(0, self.left_front_drive.get() * self.battery_voltage)
        self.fr_system_sim.setInput(0, self.right_front_drive.get() * self.battery_voltage)
        self.rl_system_sim.setInput(0, self.left_rear_drive.get() * self.battery_voltage)
        self.rr_system_sim.setInput(0, self.right_rear_drive.get() * self.battery_voltage)

        self.fl_system_sim.update(tm_diff)
        self.fr_system_sim.update(tm_diff)
        self.rl_system_sim.update(tm_diff)
        self.rr_system_sim.update(tm_diff)

        fl_out = self.fl_system_sim.getOutput(0)
        fr_out = self.fr_system_sim.getOutput(0)
        rl_out = self.rl_system_sim.getOutput(0)
        rr_out = self.rr_system_sim.getOutput(0)
        
        # Update encoders
        # Rev library is apparently horrible so we need to multiply by the velocity conversion factor manually and hope it matches the position conversion factor
        radps_to_rpm = 60 / (2*pi)
        conversion_factor = radps_to_rpm * self.left_front_drive.configAccessor.encoder.getVelocityConversionFactor()
        voltage = wpilib.RobotController.getBatteryVoltage()
        
        self.fl_motor_sim.iterate(fl_out * conversion_factor, self.battery_voltage, tm_diff)
        self.fr_motor_sim.iterate(fr_out * conversion_factor, self.battery_voltage, tm_diff)
        self.rl_motor_sim.iterate(rl_out * conversion_factor, self.battery_voltage, tm_diff)
        self.rr_motor_sim.iterate(rr_out * conversion_factor, self.battery_voltage, tm_diff)
        
        # Update simulated navx gyro heading
        # We convert from rad/s to deg/frame
        # wpilib is CCW positive, navx is CW positive, so we subtract
        self.navx_sim_yaw.set(self.navx_sim_yaw.get() - self.get_relative_speeds().omega*tm_diff*(180/pi))
       
    def get_wheel_positions(self) -> MecanumDriveWheelPositions:
        positions = MecanumDriveWheelPositions()

        positions.frontLeft = self.left_front_encoder.getPosition()
        positions.frontRight = self.right_front_encoder.getPosition()
        positions.rearLeft = self.left_rear_encoder.getPosition()
        positions.rearRight = self.right_rear_encoder.getPosition()

        return positions

    def zero_encoder_positions(self) -> None:
        self.left_front_encoder.setPosition(0)
        self.right_front_encoder.setPosition(0)
        self.left_rear_encoder.setPosition(0)
        self.right_rear_encoder.setPosition(0)

    def get_wheel_speeds(self) -> MecanumDriveWheelSpeeds:
        return MecanumDriveWheelSpeeds(
            self.left_front_encoder.getVelocity(),
            self.right_front_encoder.getVelocity(),
            self.left_rear_encoder.getVelocity(),
            self.right_rear_encoder.getVelocity(),
        )

    def get_relative_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_wheel_speeds())

    def get_pose_2d(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition().toPose2d()

    def get_pose_3d(self) -> Pose3d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose_2d(self, pose: Pose2d) -> None:
        self.pose_estimator.resetPose(Pose3d(pose))

    def reset_pose_3d(self, pose: Pose3d) -> None:
        self.pose_estimator.resetPose(pose)

    def config_drive_motor(self, motor: rev.SparkMax, inverted: bool) -> None:
        config = rev.SparkMaxConfig()

        config.inverted(inverted)

        conversion_ratio = WHEEL_CIRCUMFERENCE / WHEEL_GEAR_RATIO
        config.encoder.positionConversionFactor(conversion_ratio)
        config.encoder.velocityConversionFactor(conversion_ratio / 60)

        motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
