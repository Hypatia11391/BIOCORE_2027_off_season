from wpimath.geometry import Pose3d, Rotation3d
from commands2 import Subsystem
import ntcore
from typing import override


class VisionServer(Subsystem):
    def __init__(self, pose_estimator):
        print("INFO: Initializing vision server")
        self.pose_estimator = pose_estimator
        self.pose_sub = ntcore.NetworkTableInstance.getDefault().getTable("Vision").getDoubleArrayTopic("robot_pose").subscribe([], ntcore.PubSubOptions(pollStorage=20, keepDuplicates=True))
    
    @override
    def periodic(self):
        for update in self.pose_sub.readQueue() :
            # I dont know in what format the vision client will send the pose in so this may need to be changed
            x,y,z,roll,pitch,yaw, xdev,ydev,zdev,headingdev, timestamp = update.value
            print(f'Recieved measurement: {x=},{y=},{z=}, {roll=},{pitch=},{yaw=}, {xdev=},{ydev=},{zdev=},{headingdev=}, {timestamp=}')
            self.pose_estimator.addVisionMeasurement(
                Pose3d(x,y,z, Rotation3d(roll,pitch,yaw)),
                timestamp,
                (xdev,ydev,zdev,headingdev)
            )
