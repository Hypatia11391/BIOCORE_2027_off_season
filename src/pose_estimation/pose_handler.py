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

    def transform_cov_mat(self, cov_in, T_mat):
        R_mat = T_mat[:3, :3]

        J = np.zeros((6, 6), float)

        t_skew = np.array([[0,              -T_mat[2, 3],   T_mat[1, 3]],
                           [T_mat[2, 3],    0, -T_mat[0,    3]         ],
                           [-T_mat[1, 3],   T_mat[0, 3],    0          ]])

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

    def add_new_vision_measurements(self): # <-------------------------------------- ToDo: Fix
        for update in self.pose_sub.readQueue():
            # Pull info
            timestamp, camera_id, apriltag_set_number, *data = update.value # <--------------------- ToDo: Ask Luca about what the * does.
            
            # Pull and reshape pose and uncertainty data
            pose = np.array(data[:16], dtype=np.float64).reshape((4,4), order='F')  # order='F' specifies column-major order, which is how cpp Eigen library stores it by default and is therefore  how it is serialized on the raspberry pi side
            uncertainty = np.array(data[16:], dtype=np.float64) # <------------------------ Note this will require processing to convert to propper form. Some of the code on the c++ side may be wrong for this.
            #angle_uncertainty = 

            # Convert pose to robot in global fram (instead of camera in global frame)
            pose = self.camera_poses_inv_[camera_id, :, :] @ pose # <---------- check order
            
            self.pose_estimators[apriltag_set_number].addVisionMeasurement(
                Pose3d.from_matrix(pose),  # Im not fully sure this is the right way to construct the pose
                timestamp,
                tuple(uncertainty[:3])+(uncertainty[3:].mean(),)  # Is mean the right operation here? ----- NO!!!!
            )

    def get_pose_2d(self):
        return self.pose_estimator.getEstimatedPosition().toPose2d()

    def get_pose_3d(self):
        return self.pose_estimator.getEstimatedPosition()
