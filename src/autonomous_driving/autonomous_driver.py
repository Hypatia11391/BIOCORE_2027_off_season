import time
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from src.autonomous_driving.pid_controller import PIDController


class AutonomousDriver:
    def __init__(self, get_pose_2d_fn, get_relative_speeds_fn, get_gyro_angle_fn, drive_from_chassis_speeds_fn, forward_pid_consts, strafe_pid_consts, turn_pid_consts):
        self.get_pose_2d_fn = get_pose_2d_fn
        self.get_relative_speeds_fn = get_relative_speeds_fn
        self.get_gyro_angle_fn = get_gyro_angle_fn
        self.drive_from_chassis_speeds_fn = drive_from_chassis_speeds_fn
        
        self.x_pid = PIDController(*forward_pid_consts)
        self.y_pid = PIDController(*strafe_pid_consts)
        self.theta_pid = PIDController(*turn_pid_consts)
        
        self.enabled = False
        self.target_pose: Pose2d | None = None
    
    def step(self, h):
        if self.enabled and self.target_pose is not None:
            pose = self.get_pose_2d_fn()
            current_speeds = self.get_relative_speeds_fn()
            
            output_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                self.x_pid.step(self.target_pose.X()-pose.X(), current_speeds.dx, h),
                self.y_pid.step(self.target_pose.Y()-pose.Y(), current_speeds.dy, h),
                self.theta_pid.step(self.target_pose.rotation.radians()-pose.rotation.radians(), current_speeds.omega, h),
                self.get_gyro_angle_fn()
            )
            
            self.drive_field_oriented_from_chassis_speeds_fn(output_speeds)
