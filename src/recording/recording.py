import json


class Recording:
    def __init__(self, data: list[dict]):
        self.data = data
        self.i = 0

    @classmethod
    def from_json_file(cls, filename: str):
        with open(filename, 'r') as f:
            data = json.load(f)
        return cls(data)

    def next_frame(self):
        frame = self.data[self.i]
        self.i += 1
        return frame