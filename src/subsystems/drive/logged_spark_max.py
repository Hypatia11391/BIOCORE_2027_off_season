from typing import override

import rev


class LoggedSparkMax(rev.SparkMax):
    @override
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.last_set = 0

    @override
    def set(self, speed: float):
        self.last_set = speed
        super().set(speed)