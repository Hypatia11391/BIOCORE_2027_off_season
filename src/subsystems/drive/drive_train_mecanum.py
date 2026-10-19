from math import pi, sqrt
from typing import override

import rev
import wpilib
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.util import DriveFeedforwards
from wpilib import DriverStation, Field2d, SmartDashboard
from wpilib.drive import MecanumDrive
from wpilib.simulation import SimDeviceSim
from wpimath.controller import PIDController
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveKinematics, MecanumDriveWheelPositions, MecanumDriveWheelSpeeds
from wpimath.system.plant import DCMotor, LinearSystemId

from src.navx.navx import Navx
from src.network_server.network_server import NetworkServer
from src.simulation.mecanum_drive_sim import MecanumDriveSim, MecanumDriveSimSpec
from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_LEFT_LOCATION, FRONT_RIGHT_ID, FRONT_RIGHT_LOCATION, MAX_SPEED, REAR_LEFT_ID, REAR_LEFT_LOCATION, REAR_RIGHT_ID, REAR_RIGHT_LOCATION, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO, WHEEL_RADIUS
from src.subsystems.drive.logged_spark_max import LoggedSparkMax


class DriveTrainMecanum(Subsystem):
    def __init__(self, pose_estimator: MecanumDrivePoseEstimator3d, navx: Navx, robot, field: Field2d) -> None:
        self.traj = []

        super().__init__()

        self.robot = robot

        self.left_front_drive = LoggedSparkMax(FRONT_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = LoggedSparkMax(FRONT_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = LoggedSparkMax(REAR_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = LoggedSparkMax(REAR_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.config_drive_motor(self.left_front_drive, True)
        self.config_drive_motor(self.right_front_drive, False)
        self.config_drive_motor(self.left_rear_drive, True)
        self.config_drive_motor(self.right_rear_drive, False)

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

        def path_planner_drive(speeds: ChassisSpeeds, feedforwards: DriveFeedforwards):
            self.drive_from_chassis_speeds(speeds)
            field.getObject("target-velocity").setPose(Pose2d(speeds.vx, speeds.vy, speeds.omega))

        AutoBuilder.configure(
            self.get_pose_2d,
            self.reset_pose_2d,
            self.get_relative_speeds,
            path_planner_drive,
            PPHolonomicDriveController(PIDConstants(2.5, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0)),  # PIDConstants(0.03, 0.0, 0.022)),
            config,
            self.should_flip_path,
            self,
        )

        self.target_pose = Pose2d()

        @PathPlannerLogging.setLogTargetPoseCallback
        def f(pose):
            self.target_pose = pose

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        if wpilib.RobotBase.isSimulation():
            self._init_simulation()

        kP = 0.3
        kI = 0
        kD = 0

        self.fl_pid = PIDController(kP, kI, kD)
        self.fr_pid = PIDController(kP, kI, kD)
        self.rl_pid = PIDController(kP, kI, kD)
        self.rr_pid = PIDController(kP, kI, kD)

    def _init_simulation(self):
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()

        # wheel_plant = LinearSystemId.identifyVelocitySystemRadians(
        #     kV=self.battery_voltage / (MAX_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_GEAR_RATIO * (2 * pi)),  # ratio of volts to speed in rad/s
        #     kA=0.0007820606231689453,  # self.battery_voltage / (MAX_ACCELERATION / WHEEL_CIRCUMFERENCE * WHEEL_GEAR_RATIO * (2*pi)),
        # )

        # self.fl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        # self.fr_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        # self.rl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        # self.rr_system_sim = LinearSystemSim_1_1_1(wheel_plant)

        # self.fl_motor_sim = rev.SparkMaxSim(self.left_front_drive, DCMotor.NEO(1))
        # self.fr_motor_sim = rev.SparkMaxSim(self.right_front_drive, DCMotor.NEO(1))
        # self.rl_motor_sim = rev.SparkMaxSim(self.left_rear_drive, DCMotor.NEO(1))
        # self.rr_motor_sim = rev.SparkMaxSim(self.right_rear_drive, DCMotor.NEO(1))

        self.fl_spark_sim = rev.SparkMaxSim(self.left_front_drive, DCMotor.NEO(1))
        self.fr_spark_sim = rev.SparkMaxSim(self.right_front_drive, DCMotor.NEO(1))
        self.rl_spark_sim = rev.SparkMaxSim(self.left_rear_drive, DCMotor.NEO(1))
        self.rr_spark_sim = rev.SparkMaxSim(self.right_rear_drive, DCMotor.NEO(1))

        self.fl_encoder_sim = self.fl_spark_sim.getRelativeEncoderSim()
        self.fr_encoder_sim = self.fr_spark_sim.getRelativeEncoderSim()
        self.rl_encoder_sim = self.rl_spark_sim.getRelativeEncoderSim()
        self.rr_encoder_sim = self.rr_spark_sim.getRelativeEncoderSim()

        kV = 2.4
        kA = 2.0

        wheel_plant_fl = LinearSystemId.identifyVelocitySystemRadians(
            kV=kV,
            kA=kA,
        )
        wheel_plant_fr = LinearSystemId.identifyVelocitySystemRadians(
            kV=kV,
            kA=kA,
        )
        wheel_plant_rl = LinearSystemId.identifyVelocitySystemRadians(
            kV=kV,
            kA=kA,
        )
        wheel_plant_rr = LinearSystemId.identifyVelocitySystemRadians(
            kV=kV,
            kA=kA,
        )

        plants = (wheel_plant_fl, wheel_plant_fr, wheel_plant_rl, wheel_plant_rr)

        config = RobotConfig.fromGUISettings()
        spec = MecanumDriveSimSpec(
            gear_ratio=WHEEL_GEAR_RATIO,
            moi=config.MOI,
            mass=config.massKG,
            wheel_radius=WHEEL_RADIUS,
            wheel_positions=(
                FRONT_LEFT_LOCATION,
                FRONT_RIGHT_LOCATION,
                REAR_LEFT_LOCATION,
                REAR_RIGHT_LOCATION,
            ),
            max_volts=12,
        )

        self.drive_sim = MecanumDriveSim(plants, spec)

        self.navx_sim = SimDeviceSim("navX-Sensor[4]")
        self.navx_sim_yaw = self.navx_sim.getDouble("Yaw")

        self.last_sim_time = wpilib.Timer.getFPGATimestamp()

    def reset_simulation(self, kA=None, kV=None):
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()

        self.fl_spark_sim = rev.SparkMaxSim(self.left_front_drive, DCMotor.NEO(1))
        self.fr_spark_sim = rev.SparkMaxSim(self.right_front_drive, DCMotor.NEO(1))
        self.rl_spark_sim = rev.SparkMaxSim(self.left_rear_drive, DCMotor.NEO(1))
        self.rr_spark_sim = rev.SparkMaxSim(self.right_rear_drive, DCMotor.NEO(1))

        self.fl_encoder_sim = self.fl_spark_sim.getRelativeEncoderSim()
        self.fr_encoder_sim = self.fr_spark_sim.getRelativeEncoderSim()
        self.rl_encoder_sim = self.rl_spark_sim.getRelativeEncoderSim()
        self.rr_encoder_sim = self.rr_spark_sim.getRelativeEncoderSim()

        self.drive_sim.reset()

        self.navx_sim_yaw.set(0.0)

        self.last_sim_time = wpilib.Timer.getFPGATimestamp()

    def should_flip_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        print("speeds:", forward_speed, strafe_speed, turn_speed)

        # clamp = 0.25

        # forward_speed = max(min(forward_speed, clamp), -clamp)
        # strafe_speed = max(min(strafe_speed, clamp), -clamp)
        # turn_speed = max(min(turn_speed, clamp), -clamp)

        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed)
        # print(f"fl applied output: {self.left_front_drive.getAppliedOutput()}")
        # print(f'{forward_speed=} {strafe_speed=} {turn_speed=}')
        # print(f'{self.left_front_drive.getAppliedOutput()=}')

    def drive_field_oriented(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed, self.navx.get_2d_rotation())

    def drive_from_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        # forward_speed = speeds.vx
        # strafe_speed = speeds.vy
        # turn_speed = speeds.omega

        # forward_speed_percent = forward_speed / MAX_SPEED
        # strafe_speed_percent = strafe_speed / MAX_SPEED
        # turn_speed_percent = turn_speed / MAX_ANGULAR_SPEED

        # self.drive(forward_speed_percent, strafe_speed_percent, turn_speed_percent)

        # NetworkServer.getInstance().set_float("front-left", self.left_front_drive.getAppliedOutput())
        # NetworkServer.getInstance().set_float("front-right", self.right_front_drive.getAppliedOutput())
        # NetworkServer.getInstance().set_float("rear-left", self.left_rear_drive.getAppliedOutput())
        # NetworkServer.getInstance().set_float("rear-right", self.right_rear_drive.getAppliedOutput())

        # print(f"{self.navx.get_angualar_velocity()=}")
        # print(f"{speeds.omega=}")

        wheel_speeds = self.kinematics.toWheelSpeeds(speeds)
        wheel_speeds.desaturate(MAX_SPEED)

        front_left_percent = wheel_speeds.frontLeft / MAX_SPEED
        front_right_percent = wheel_speeds.frontRight / MAX_SPEED
        rear_left_percent = wheel_speeds.rearLeft / MAX_SPEED
        rear_right_percent = wheel_speeds.rearRight / MAX_SPEED

        bad_negation = -1 if wpilib.RobotBase.isSimulation() else 1
        self.left_front_drive.set(front_left_percent * bad_negation)
        self.right_front_drive.set(front_right_percent * bad_negation)
        self.left_rear_drive.set(rear_left_percent * bad_negation)
        self.right_rear_drive.set(rear_right_percent * bad_negation)

    def drive_from_raw_motor_set_values(self, fl_percent: float, fr_percent: float, rl_percent: float, rr_percent: float):
        # print(f'driving from applied output {fl_percent=}')
        self.left_front_drive.set(fl_percent)
        self.right_front_drive.set(fr_percent)
        self.left_rear_drive.set(rl_percent)
        self.right_rear_drive.set(rr_percent)

    @override
    def periodic(self) -> None:
        # print(f'{self.get_wheel_speeds()=}')
        # print(f'{self.get_relative_speeds()=}')
        # print(self.pose_estimator.getEstimatedPosition().toPose2d())

        if self.pose_estimator is not None:
            self.pose_estimator.update(
                self.navx.get_full_rotation(),
                self.get_wheel_positions(),
            )
            self.field.setRobotPose(self.pose_estimator.getEstimatedPosition().toPose2d())

        if DriverStation.isAutonomous():
            self.robot_drive.setSafetyEnabled(False)
        else:
            self.robot_drive.setSafetyEnabled(True)

        speeds = self.get_relative_speeds()
        self.traj.append(
            {
                "set-values": [
                    self.left_front_drive.last_set,
                    self.right_front_drive.last_set,
                    self.left_rear_drive.last_set,
                    self.right_rear_drive.last_set,
                ],
                "chassis-speeds": [speeds.vx, speeds.vy, speeds.omega],
            }
        )

        NetworkServer.getInstance().set_float_list(
            "drive-applied-output",
            [
                self.left_front_drive.last_set,
                self.right_front_drive.last_set,
                self.left_rear_drive.last_set,
                self.right_rear_drive.last_set,
            ],
        )

    @override
    def simulationPeriodic(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        tm_diff = current_time - self.last_sim_time
        self.last_sim_time = current_time

        # self.fl_motor_sim.iterate(fl_out * conversion_factor, self.battery_voltage, tm_diff)
        # self.fr_motor_sim.iterate(fr_out * conversion_factor, self.battery_voltage, tm_diff)
        # self.rl_motor_sim.iterate(rl_out * conversion_factor, self.battery_voltage, tm_diff)
        # self.rr_motor_sim.iterate(rr_out * conversion_factor, self.battery_voltage, tm_diff)

        self.drive_sim.set_inputs(
            self.left_front_drive.get() * 12,
            self.right_front_drive.get() * 12,
            self.left_rear_drive.get() * 12,
            self.right_rear_drive.get() * 12,
        )
        self.drive_sim.update(0.02)

        # print(f"SETTING fl encoder sim position to {self.drive_sim.get_fl_position()=}, Sim encoder position is {self.fl_spark_sim.getPosition()=} normal encoder: {self.left_front_encoder.getPosition()=}")
        # print(f"SETTING fr encoder sim position to {self.drive_sim.get_fr_position()=}, Sim encoder position is {self.fr_spark_sim.getPosition()=} normal encoder: {self.right_front_encoder.getPosition()=}")
        # print(f"SETTING rl encoder sim position to {self.drive_sim.get_rl_position()=}, Sim encoder position is {self.rl_spark_sim.getPosition()=} normal encoder: {self.left_rear_encoder.getPosition()=}")
        # print(f"SETTING rr encoder sim position to {self.drive_sim.get_rr_position()=}, Sim encoder position is {self.rr_spark_sim.getPosition()=} normal encoder: {self.right_rear_encoder.getPosition()=}")

        self.fl_encoder_sim.setPosition(self.drive_sim.get_fl_position())
        self.fr_encoder_sim.setPosition(self.drive_sim.get_fr_position())
        self.rl_encoder_sim.setPosition(self.drive_sim.get_rl_position())
        self.rr_encoder_sim.setPosition(self.drive_sim.get_rr_position())

        self.fl_encoder_sim.setVelocity(self.drive_sim.get_fl_velocity())
        self.fr_encoder_sim.setVelocity(self.drive_sim.get_fr_velocity())
        self.rl_encoder_sim.setVelocity(self.drive_sim.get_rl_velocity())
        self.rr_encoder_sim.setVelocity(self.drive_sim.get_rr_velocity())

        # Update simulated navx gyro heading
        # We convert from rad/s to deg/frame
        # wpilib is CCW positive, navx is CW positive, so we subtract
        self.navx_sim_yaw.set(self.navx_sim_yaw.get() - self.get_relative_speeds().omega * tm_diff * (180 / pi))

        print(self.pose_estimator.getEstimatedPosition())

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

    def get_speeds_rms(self, target_speeds: ChassisSpeeds):
        actual_speeds = self.get_relative_speeds()
        angular_weight = 0.2
        return sqrt((actual_speeds.vx - target_speeds.vx) ** 2 + (actual_speeds.vy - target_speeds.vy) ** 2 + (angular_weight * (actual_speeds.omega - target_speeds.omega)) ** 2)

    def get_pose_2d(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition().toPose2d()

    def get_pose_3d(self) -> Pose3d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose_2d(self, pose: Pose2d) -> None:
        self.pose_estimator.resetPose(Pose3d(pose))

    def reset_pose_3d(self, pose: Pose3d) -> None:
        self.pose_estimator.resetPose(pose)

    def reset_encoders(self):
        self.left_front_encoder.setPosition(0.0)
        self.right_front_encoder.setPosition(0.0)
        self.left_rear_encoder.setPosition(0.0)
        self.right_rear_encoder.setPosition(0.0)

    def get_pose_rms(self, target_pose: Pose2d | None = None):
        target_pose = target_pose or self.target_pose
        actual_pose = self.get_pose_2d()
        translation_error = actual_pose.translation().distance(self.target_pose.translation())
        print(f"{translation_error=}")
        rotation_error = (actual_pose.rotation() - self.target_pose.rotation()).radians()
        print(f"{rotation_error=}")
        rotation_weight = 0.2
        rms = sqrt(translation_error**2 + (rotation_error * rotation_weight) ** 2)
        return rms

    def config_drive_motor(self, motor: rev.SparkMax, inverted: bool) -> None:
        config = rev.SparkMaxConfig()

        config.inverted(inverted)

        conversion_ratio = WHEEL_CIRCUMFERENCE / WHEEL_GEAR_RATIO
        config.encoder.positionConversionFactor(conversion_ratio)
        config.encoder.velocityConversionFactor(conversion_ratio / 60)

        motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
