from typing import Self  # Built into Python 3.11+. For older versions, use -> "NetworkServer":

import ntcore


class NetworkServer:
    _instance = None

    def __init__(self) -> None:
        if NetworkServer._instance is not None:
            raise RuntimeError("Use NetworkServer.getInstance() instead of the constructor.")

        inst = ntcore.NetworkTableInstance.getDefault()

        self.table = inst.getTable("datatable")

        self.publishers: dict[str, ntcore.DoublePublisher] = {}
        self.values = {}

    @classmethod
    def getInstance(cls) -> Self:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def set_float(self, key: str, value: float) -> None:
        self.values[key] = value

        if key not in self.publishers:
            self.publishers[key] = self.table.getDoubleTopic(key).publish()

        self.publishers[key].set(value)
