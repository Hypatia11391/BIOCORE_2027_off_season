from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

import ntcore
import src.constants as consts
import numpy as np
from src.navx.navx import Navx
from typing import override

class PoseHandler():
    _instance = None

    def __init__(self, drive):
        self.camera_poses_inv_ = consts.ROBOT_POSES_IN_CAMERA

        self.poses = np.array([Pose3d(), Pose3d(), Pose3d()])

        self.navx = Navx()
        
        print("INFO: Initializing vision server")
        self.pose_sub = ntcore.NetworkTableInstance.getDefault().getTable("Vision").getDoubleArrayTopic("robot_pose").subscribe([], ntcore.PubSubOptions(pollStorage=20, keepDuplicates=True))

        self.drive = drive

        print("INFO: Initializing pose estimator")
        self.pose_estimators = None
        for i in range(self.poses.shape[0]):
            single_pose_estimator = MecanumDrivePoseEstimator3d(self.drive.kinematics, self.navx.get_full_rotation(), MecanumDriveWheelPositions(), consts.STARTING_POSE)
            
            if self.pose_estimators == None:
                self.pose_estimators = np.array([single_pose_estimator])
            else:
                np.append(self.pose_estimators, single_pose_estimator)

    def periodic(self):
        for i in range(self.pose_estimators.shape[0]):
            # Update Odometry
            self.pose_estimators[i].update(
                self.navx.get_full_rotation(),
                self.drive.get_wheel_positions()
            )

            # Update current poses
            self.poses[i] = self.pose_estimator[i].getEstimatedPosition()

        self.add_new_vision_measurements()

    def add_new_vision_measurements(self):
        for update in self.pose_sub.readQueue():
            # Pull info
            timestamp, camera_id, apriltag_set_number, *data = update.value # <--------------------- ToDo: Ask Luca about what the * does.
            
            # Pull and reshape pose and uncertainty data
            pose = np.array(data[:16], dtype=np.float64).reshape((4,4), order='F')  # order='F' specifies column-major order, which is how cpp Eigen library stores it by default and is therefore  how it is serialized on the raspberry pi side
            uncertainty = np.array(data[16:], dtype=np.float64)

            # Convert pose to robot in global fram (instead of camera in global frame)
            pose = self.camera_poses_inv_[camera_id, :, :] @ pose
            
            # Input vision measurements
            self.pose_estimators[apriltag_set_number].addVisionMeasurement(
                Pose3d.from_matrix(pose),
                timestamp,
                tuple(uncertainty[:3])+(consts._TEMP_ANGULAR_POSE_UNCERTAINTY_VISION_,)
            )

    def get_pose_2d(self):
        return self.pose_estimator.getEstimatedPosition().toPose2d()

    def get_pose_3d(self):
        return self.pose_estimator.getEstimatedPosition()
