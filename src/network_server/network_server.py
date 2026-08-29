from typing import Self  # Built into Python 3.11+. For older versions, use -> "NetworkServer":

import ntcore


class NetworkServer:
    _instance = None

    def __init__(self) -> None:
        if NetworkServer._instance is not None:
            raise RuntimeError("Use NetworkServer.getInstance() instead of the constructor.")

        inst = ntcore.NetworkTableInstance.getDefault()

        self.table = inst.getTable("datatable")

        self.double_publishers: dict[str, ntcore.DoublePublisher] = {}
        self.double_values: dict[str, float] = {}

        self.string_list_publishers: dict[str, ntcore.StringArrayPublisher] = {}
        self.string_list_values: dict[str, list[str]] = {}

        self.string_subsribers: dict[str, ntcore.StringSubscriber] = {}

    @classmethod
    def getInstance(cls) -> Self:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def set_float(self, key: str, value: float) -> None:
        self.double_values[key] = value

        if key not in self.double_publishers:
            self.double_publishers[key] = self.table.getDoubleTopic(key).publish()

        self.double_publishers[key].set(value)

    def set_string_list(self, key: str, list: list[str]):
        self.string_list_values[key] = list

        if key not in self.string_list_publishers:
            self.string_list_publishers[key] = self.table.getStringArrayTopic(key).publish()

        self.string_list_publishers[key].set(list)

    def get_string(self, key: str) -> str:
        if key not in self.string_subsribers:
            self.string_subsribers[key] = self.table.getStringTopic(key).subscribe("")

        return self.string_subsribers[key].get()
