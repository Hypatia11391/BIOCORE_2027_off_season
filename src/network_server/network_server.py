from typing import Self  # Built into Python 3.11+. For older versions, use -> "NetworkServer":

import ntcore
from wpilib import Field2d


class NetworkServer:
    _instance = None

    def __init__(self) -> None:
        if NetworkServer._instance is not None:
            raise RuntimeError("Use NetworkServer.getInstance() instead of the constructor.")

        inst = ntcore.NetworkTableInstance.getDefault()

        self.table = inst.getTable("datatable")

        self.double_publishers: dict[str, ntcore.DoublePublisher] = {}

        self.field_publishers: dict[str, dict[str, ntcore.DoubleArrayPublisher]] = {}

        self.string_list_publishers: dict[str, ntcore.StringArrayPublisher] = {}

        self.string_subsribers: dict[str, ntcore.StringSubscriber] = {}

    @classmethod
    def getInstance(cls) -> Self:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def set_float(self, key: str, value: float) -> None:
        if key not in self.double_publishers:
            self.double_publishers[key] = self.table.getDoubleTopic(key).publish()

        self.double_publishers[key].set(value)

    def set_string_list(self, key: str, list: list[str]):
        if key not in self.string_list_publishers:
            self.string_list_publishers[key] = self.table.getStringArrayTopic(key).publish()

        self.string_list_publishers[key].set(list)

    def set_field(self, key: str, field: Field2d):
        if key not in self.field_publishers:
            self.field_publishers[key] = {}

            field_table = self.table.getSubTable(key)

            self.field_publishers[key]["robot"] = field_table.getDoubleArrayTopic("robot").publish()
            self.field_publishers[key]["velocity"] = field_table.getDoubleArrayTopic("velocity").publish()
            self.field_publishers[key]["trajectory"] = field_table.getDoubleArrayTopic("trajectory").publish()

        pose = field.getRobotPose()

        self.field_publishers[key]["robot"].set(
            [
                pose.X(),
                pose.Y(),
                pose.rotation().degrees(),
            ]
        )

        velocity = field.getObject("velocity").getPose()

        self.field_publishers[key]["velocity"].set(
            [
                velocity.X(),
                velocity.Y(),
                velocity.rotation().degrees(),
            ]
        )

        flat_trajectory = []

        trajectory_poses = field.getObject("trajectory").getPoses()

        for pose in trajectory_poses:
            flat_trajectory.append(pose.X())
            flat_trajectory.append(pose.Y())
            flat_trajectory.append(pose.rotation().degrees())

        self.field_publishers[key]["trajectory"].set(flat_trajectory)

    def get_string(self, key: str) -> str:
        if key not in self.string_subsribers:
            self.string_subsribers[key] = self.table.getStringTopic(key).subscribe("")

        return self.string_subsribers[key].get()
