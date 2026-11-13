from typing import override
from time import time_ns
import sys

from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions
import ntcore
import numpy as np

import src.constants as consts
from src.navx.navx import Navx
from src.network_server.network_server import NetworkServer


class PoseHandler():
    _instance = None
    
    def __init__(self, drive):
        self.robot_poses_in_camera = consts.ROBOT_POSES_IN_CAMERA
        
        self.poses = [Pose3d(), Pose3d(), Pose3d()]
        
        self.navx = Navx()
        
        self.drive = drive
        
        print("INFO: Initializing pose estimator")
        self.pose_estimators: list[MecanumDrivePoseEstimator3d] = []
        for i in range(len(self.poses)):
            single_pose_estimator = MecanumDrivePoseEstimator3d(self.drive.kinematics, self.navx.get_full_rotation(), MecanumDriveWheelPositions(), consts.STARTING_POSE)
            self.pose_estimators.append(single_pose_estimator)
        
        self.path_planner_offset = self.poses
    
    def periodic(self):
        for i in range(len(self.pose_estimators)):
            # Update Odometry
            self.pose_estimators[i].update(self.navx.get_full_rotation(), self.drive.get_wheel_positions())
            
            # Update current poses
            self.poses[i] = self.pose_estimators[i].getEstimatedPosition()
        
        self.add_new_vision_measurements()
    
    def add_new_vision_measurements(self):
        vals = NetworkServer.getInstance().get_double_array("robot_pose")
        
        if len(vals) < 15:
            return
        
        # Pull info
        timestamp, camera_id, apriltag_set_number, *data = vals  # <--------------------- ToDo: Ask Luca about what the * does.
        camera_id = int(camera_id)
        apriltag_set_number = int(apriltag_set_number)
        
        # Pull and reshape pose and uncertainty data
        pose = np.array(data[:16], dtype=np.float64).reshape((4, 4), order='F')  # order='F' specifies column-major order, which is how cpp Eigen library stores it by default and is therefore  how it is serialized on the raspberry pi side
        uncertainty = np.array(data[16:], dtype=np.float64)
        print("Camera Pose:")
        print(pose)
        
        print(f"{timestamp - NetworkServer.getInstance().get_synced_time() / 1e3=}")
        
        # Convert pose to robot in global fram (instead of camera in global frame)
        pose = self.robot_poses_in_camera[camera_id, :, :] @ pose
        
        last_line_errors = pose[3, :] - np.array([0, 0, 0, 1])
        last_line_check = np.abs(last_line_errors) > 1e-6
        if np.any(last_line_check):
            print(f"Pose matrix last line corrupted. Error val was: {last_line_errors}", file=sys.stderr)
        
        pose[3, :] = np.array([0, 0, 0, 1])
        
        # Input vision measurements
        self.pose_estimators[apriltag_set_number].addVisionMeasurement(Pose3d.fromMatrix(pose), timestamp,
                                                                       tuple(uncertainty[:3]) + (consts._TEMP_ANGULAR_POSE_UNCERTAINTY_VISION_, ))
    
    def reset_pose_offset(self):
        self.path_planner_offset = self.poses
    
    def get_pose_2d(self, frame_id):
        return self.poses[frame_id].toPose2d()
    
    def get_pose_3d(self, frame_id):
        return self.poses[frame_id]
    
    def get_pathplanner_pose_2d(self, frame_id):
        path_planner_pose_3d = self.poses[frame_id].relative_to(self.path_planner_offset)
        return path_planner_pose_3d.toPose2d()
